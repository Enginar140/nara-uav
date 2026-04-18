# ARDUPLANE FİZİKSEL YAPILABİLİLİK ANALİZİ

> **Amaç:** ArduPlane kaynak kodlarının fiziksel sınırlarını ve neyin yapılabileceğini derinlemesine analiz etmek

---

## 📋 İÇİNDEKİLER

1. [Fiziksel Sınırlamalar](#1-fiziksel-sınırlamalar)
2. [Kontrol Mimarisi](#2-kontrol-mimarisi)
3. [Yapılabilirlik Analizi](#3-yapılabilirlik-analizi)
4. [Fiziksel Kanıtlar](#4-fiziksel-kanıtlar)
5. [Ne Yapılabilir?](#5-ne-yapılabilir)
6. [Mimari Seçenekleri](#6-mimari-seçenekleri)

---

## 1. FİZİKSEL SINIRLAMALAR

### 1.1 Fixed-Wing Aerodinamiği

```
┌─────────────────────────────────────────────────────────────┐
│              FIXED-WING AERODİNAMİK FİZİKİ                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  LIFT EŞİTİ:                                              │
│    L = 0.5 * ρ * v² * S * CL                               │
│                                                             │
│  DRAG EŞİTİ:                                              │
│    D = 0.5 * ρ * v² * S * CD                               │
│                                                             │
│  SÜRÜKLEME KOŞULU:                                        │
│    Lift = Weight * cos(θ)                                 │
│    Horizontal Force = Lift * tan(θ)                       │
│                                                             │
│  STALL LIMIT:                                            │
│    v_stall = √(2 * Weight / (ρ * S * CL_max))            │
│                                                             │
│  ENÜK YATAY DÖNÜŞ:                                       │
│    R_min = v² / (g * tan(φ_max))                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Değişkenler:**
- `ρ`: Hava yoğunluğu (~1.225 kg/m³)
- `v`: Hız (m/s)
- `S`: Kanat alanı (m²)
- `CL, CD`: Lift/drag katsayıları
- `θ`: Pitch angle
- `φ`: Bank angle
- `g`: Yerçekimi (9.81 m/s²)

### 1.2 ArduPlane'in Fiziksel Kısıtlamaları

**Kontrol Girişleri:**
```
┌─────────────────────────────────────────────────────────────┐
│              ARDUPLANE CONTROL INPUTS                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  DOĞRUDAN KONTROL EDİLEN:                                  │
│  ├── Roll → Aileron (hemen)                                │
│  ├── Pitch → Elevator (hemen)                             │
│  ├── Yaw → Rudder (hemen)                                 │
│  └── Throttle → Motor (hemen)                              │
│                                                             │
│  WAYPOINT İLE KONTROL EDİLEN:                             │
│  ├── L1 Navigation → Roll demand                         │
│  ├── TECS → Pitch + Throttle                              │
│  └── Slew Commands → Hedef değerleri                         │
│                                                             │
│  DOĞRUDAN KONTROL EDİLMEYEN:                             │
│  ├── X Position ❌ (aerodinamik)                          │
│  ├── Y Position ❌ (aerodinamik)                          │
│  ├── Z Position ✅ (TECS ile)                            │
│  └── X/Y Velocity ❌ (aerodinamik)                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. KONTROL MİMARİSİ

### 2.1 L1 Navigation (Lateral Control)

**Referans:** Park, Deyst, How - "A New Nonlinear Guidance Logic for Trajectory Tracking" (AIAA 2004-4900)

**Temel Formül:**
```
L1 mesafesi: d_L1 = K * damping² * period * groundSpeed
Lateral accel: a_lat = K_L1 * v² / d_L1 * sin(Nu)
Bank angle: φ = atan(a_lat / g)
```

**Kod](cpp_uav_external/ardupilot_repo/libraries/AP_L1_Control/AP_L1_Control.cpp:206-347):

```cpp
void AP_L1_Control::update_waypoint(prev_WP, next_WP, dist_min) {
    // 1. L1 distance hesapla
    float K_L1 = 4.0f * _L1_damping * _L1_damping;
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);
    
    // 2. Crosstrack error
    _crosstrack_error = A_air % AB;
    
    // 3. Nu1 (path capture angle)
    float sine_Nu1 = _crosstrack_error / MAX(_L1_dist, 0.1f);
    sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);  // ±45°
    float Nu1 = asinf(sine_Nu1);
    
    // 4. Nu2 (velocity angle relative to track)
    float Nu2 = atan2f(xtrackVel, ltrackVel);
    
    // 5. Combined steering angle
    float Nu = Nu1 + Nu2;
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);  // ±90°
    
    // 6. Lateral acceleration demand
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
}
```

**Parametreler:**
- `NAVL1_PERIOD`: 17s (varsayılan) - ne kadar düşük, o kadar agresif
- `NAVL1_DAMPING`: 0.75 - overshoot'u azaltır

### 2.2 TECS (Longitudinal Control)

**Referans:** Paul Riseborough - "Total Energy Control System for Aircraft"

**Temel Konsept:**
```
Toplam Enerji = Kinetik + Potansiyel
E = ½mv² + mgh

Throttle → Toplam enerjiyi kontrol eder
Pitch → Kinetik/potansiyel enerji dağılımını kontrol eder
```

**Kod](cpp_uav_external/ardupilot_repo/libraries/AP_TECS/AP_TECS.cpp:313-400):

```cpp
void AP_TECS::update_50hz() {
    // 1. Height ve climb rate filtrele (3. order complementary filter)
    _ahrs.get_relative_position_D_home(_height);
    // ... filtreleme ...
    
    // 2. Speed update
    _update_speed(DT);
    
    // 3. Energy quantities
    SPE = g * h;        // Specific potential energy
    SKE = 0.5 * v²;     // Specific kinetic energy
    STE = SPE + SKE;    // Specific total energy
}

void AP_TECS::update_pitch_throttle(hgt_dem, EAS_dem, ...) {
    // 1. Speed demand
    _TAS_dem = adjust_speed_demand(EAS_dem);
    
    // 2. Height demand
    _hgt_dem = adjust_height_demand(hgt_dem);
    
    // 3. Energy error
    STE_error = (SPE_dem + SKE_dem) - (SPE_est + SKE_est);
    
    // 4. Throttle (total energy control)
    _throttle_dem = _integSEBdot + STE_error * _integGain + _feedforward;
    
    // 5. Pitch (energy distribution)
    _pitch_dem = (SKE_dem - SKE_est) * _spdWeight + _feedforward;
}
```

---

## 3. YAPILABİLİRLİK ANALİZİ

### 3.1 Mevcut MAVLink Komutları

| Komut | ArduPlane | ArduCopter | Fiziksel Sebep |
|-------|-----------|------------|-----------------|
| `DO_REPOSITION` | ✅ Global pos | ✅ Global pos | Fixed-wing waypoint |
| `DO_SET_HOME` | ✅ | ✅ | Home position |
| `DO_SET_SERVO` | ✅ | ✅ | RC override |
| `SET_ATTITUDE_TARGET` | ⚠️ Roll/Pitch/Yaw | ✅ Full attitude | Body rates |
| `SET_POSITION_TARGET_GLOBAL_INT` | ⚠️ Alt only | ✅ Full 6-DOF | Altitude control |
| `SET_POSITION_TARGET_LOCAL_NED` | ⚠️ Alt only | ✅ Full 6-DOF | **X/Y mümkün değil** |
| `MAV_CMD_GUIDED_CHANGE_HEADING` | ✅ | ❌ | Slew control |
| `MAV_CMD_GUIDED_CHANGE_SPEED` | ✅ | ❌ | Slew control |
| `MAV_CMD_GUIDED_CHANGE_ALTITUDE` | ✅ | ❌ | Slew control |

### 3.2 Neden X/Y Local Yok?

**Fizikisel Sebep:**

```
Fixed-wing uçağı için X/Y local komutu anlamlı DEĞİLDİR!

Çünkü:
1. Uçak sadece ileriye doğru uçabilir
2. Yan dönüş için bank angle gerekir
3. Bank angle → lateral force → heading change
4. Bu bir ORYANTAL SİSTEM, pozisyonel değil!

Örnek:
- "100m sağa git" → Anlamasız
- "90° sağa dön, 100m git" → Anlamalı
```

**Aerodinamik Kanıt:**

```
Lateral Force = Lift * tan(φ)
              = (Weight / cos(φ)) * tan(φ)
              = Weight * tan(φ) / cos(φ)
              = m * g * tan(φ) / cos(φ)
              
Yatay ivme: a_lat = Lateral Force / m
                = g * tan(φ)
                
Bu YAPILABİLİR, ancak heading kontrolü üzerinden!
```

### 3.3 Slew Commands ile Ne Yapılabilir?

**✅ YAPILABİLİR:**

| İstek | Slew Command | Fiziksel Gerçekleşme |
|------|--------------|----------------------|
| "Sağa dön" | `CHANGE_HEADING` | Roll → Bank → Turn |
| "Hızlan" | `CHANGE_SPEED` | Throttle → TECS |
| "Yüksel" | `CHANGE_ALTITUDE` | Pitch → Climb |
| "Hedefi takip et" | `HEADING + SPEED` | Pursuit |

**❌ YAPILAMAZ (Doğrudan):**

| İstek | Neden? |
|------|--------|
| "100m sağa git" | Aerodinamik, X/Y lokal kontrol yok |
| "5m/s sağa yanla" | Aerodinamik, lateral velocity yok |
| "Ani dur" | Inertia, lift devam eder |

---

## 4. FİZİKSEL KANITLAR

### 4.1 L1 Navigation Kanıtı

**Koddan kanıt** ([`AP_L1_Control.cpp:338`](cpp_uav_external/ardupilot_repo/libraries/AP_L1_Control/AP_L1_Control.cpp:338)):

```cpp
// Lateral acceleration formülü:
_latAccDem = K_L1 * groundSpeed² / _L1_dist * sinf(Nu);

// Bank angle hesapla (nav_roll_cd):
ret = degrees(atanf(_latAccDem / (GRAVITY_MSS * cosf(pitch)))) * 100.0f;
```

**Bu formül fiziksel olarak doğrudur çünkü:**
1. Newton'un 2. yasası: F = ma
2. Dairesel hareket: F = mv²/r
3. Denge: tan(φ) = a_lat / g

### 4.2 TECS Kanıtı

**Koddan kanıt** ([`AP_TECS.cpp:297-310`](cpp_uav_external/ardupilot_repo/libraries/AP_TECS/AP_TECS.cpp:297-310)):

```cpp
// Energy balance:
SPE = g * h           // Specific potential energy
SKE = 0.5 * v²        // Specific kinetic energy
STE = SPE + SKE       // Specific total energy

// Throttle controls STE
// Pitch controls KE/PE distribution
```

**Bu enerji korunumu prensibine dayanır:**
1. Toplam enerji değiştiğinde hız/irtifa değişir
2. Throttle enerji ekler
3. Pitch kinetik/potansiyel arasını ayarlar

### 4.3 Guided Slew Kanıtı

**Koddan kanıt** ([`mode_guided.cpp:48-71`](cpp_uav_external/ardupilot_repo/ArduPlane/mode_guided.cpp:48-71)):

```cpp
// Heading slew with PID:
float error = wrap_PI(target_heading - current_heading);
float bank_limit = degrees(atan(accel_limit / GRAVITY_MSS)) * 100;
float desired_roll = g2.guidedHeading.update_error(error, delta, limit);
nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
```

**Bu fiziksel olarak güvenli çünkü:**
1. Acceleration limit → G force sınırlaması
2. Bank limit → Stall koruması
3. PID → Smooth transition

---

## 5. NE YAPILABİLİR?

### 5.1 ✅ YAPILABİLİR (Slew Commands ile)

**Heading Control:**
```cpp
// Vision tracking
float target_heading = vision->getBearingToTarget();
commander->changeHeading(target_heading, HEADING, 0.5f);
// → ArduPilot bank angle hesaplar → Servo output
```

**Speed Control:**
```cpp
// Hedefe yaklaşırken
float target_speed = 25.0f;
commander->changeAirspeed(target_speed, 3.0f);
// → TECS throttle adjust → Motor RPM
```

**Altitude Control:**
```cpp
// Yükseklik ayarı
float target_alt = 150.0f;
commander->changeAltitude(target_alt, 2.0f);
// → TECS pitch adjust → Elevator
```

### 5.2 ⚠️ KISİTEN YAPILABİLİR (Hibrit Yaklaşım)

**"100m sağa git" emulasyonu:**

```cpp
void moveRight(float distance_m) {
    // 1. Mevcut durum
    auto current = world_model_->getOwnState().position;
    
    // 2. Sağa 90° dön
    float current_heading = geo::bearing(current, current); // dummy
    float target_heading = wrap_360(current_heading + 90.0);
    commander_->changeHeading(target_heading, HEADING, 0.5);
    
    // 3. 100m git
    float estimated_time = distance_m / 20.0f; // 20 m/s
    sleep(estimated_time);
    
    // 4. Original heading'e dön
    commander_->changeHeading(current_heading, HEADING, 0.5);
}
```

**Sorunlar:**
- Rüzgar/drift → hedefi kaçır
- Hız değişimi → mesafe hatası
- Timing hassasiyeti

### 5.3 ❌ YAPILAMAZ (Doğrudan)

**Direct X/Y control:**
- Aerodinamik olarak imkansız
- ArduPilot desteklemiyor
- Alternatif: Multirotor kullan

---

## 6. MİMARİ SEÇENEKLERİ

### 6.1 Seçenek 1: Slew Commands (ÖNERİLEN)

```
┌─────────────────────────────────────────────────────────────┐
│                  SLEW COMMANDS ARCHITECTURE                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Vision Layer                                               │
│  ├── Target detection                                      │
│  ├── Bearing calculation                                    │
│  └── Distance estimation                                   │
│         │                                                   │
│         ▼                                                   │
│  Control Layer (YENİ)                                     │
│  ├── HeadingController → MAV_CMD_GUIDED_CHANGE_HEADING    │
│  ├── SpeedController    → MAV_CMD_GUIDED_CHANGE_SPEED      │
│  ├── AltitudeController → MAV_CMD_GUIDED_CHANGE_ALTITUDE   │
│  └── ProportionalNav → Pursuit logic                      │
│         │                                                   │
│         ▼                                                   │
│  Safety Layer                                                │
│  ├── SafetyGate.validateHeading()                          │
│  ├── SafetyGate.isHeadingSafe()                            │
│  └── SafetyMonitor.watchdog                                │
│         │                                                   │
│         ▼                                                   │
│  ArduPilot Plane                                             │
│  ├── GuidedState.target_heading                            │
│  ├── GuidedState.target_airspeed_cm                       │
│  ├── L1 Navigation → Roll command                         │
│  └── TECS → Pitch + Throttle                              │
│         │                                                   │
│         ▼                                                   │
│  Servos                                                     │
│  ├── Aileron → Roll                                       │
│  ├── Elevator → Pitch                                     │
│  ├── Rudder → Yaw                                         │
│  └── Motor → Throttle                                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Seçenek 2: ArduPilot Fork (DEĞİL ÖNERİLEN)

**Değişiklikler:**

```cpp
// ArduPlane/GCS_MAVLink_Plane.cpp'de:
void handle_set_position_target_local_ned(...) {
    // MEVCUT (sadece altitude):
    plane.next_WP_loc.alt += -packet.z * 100.0;
    
    // YENİ (X/Y ekle):
    if (!ignore_x && !ignore_y) {
        // X/Y offset'i heading olarak kullan
        float target_heading = atan2f(packet.y, packet.x);
        float target_distance = sqrtf(packet.x*packet.x + packet.y*packet.y);
        
        // Speed set et
        float speed = groundSpeed; // veya commanded
        commander_->changeAirspeed(speed);
        
        // Heading set et
        commander_->changeHeading(degrees(target_heading), HEADING);
        
        // Distance'yi bekle
        auto duration = target_distance / speed;
        // ... timing logic
    }
    
    // Altitude (mevcut kod)
    plane.next_WP_loc.alt += -packet.z * 100.0;
}
```

**Sorunlar:**
- Fork gerekir → güncelleme zorlaşır
- Test yükü → çok validateasyon
- Backward compatibility riski

### 6.3 Seçenek 3: Hibrit (COMPROMISE)

```cpp
class HybridPositionController {
public:
    enum class Command {
        GLOBAL_WAYPOINT,   // DO_REPOSITION
        HEADING_SLEW,      // MAV_CMD_GUIDED_CHANGE_HEADING
        VELOCITY,          // SET_POSITION_TARGET_LOCAL_NED (vel only)
        ALTITUDE_SLEW      // MAV_CMD_GUIDED_CHANGE_ALTITUDE
    };
    
    void execute(Command cmd, float x, float y, float z) {
        switch(cmd) {
            case GLOBAL_WAYPOINT:
                // Büyük hareketler (>200m)
                auto global_target = calculateGlobalTarget(x, y, z);
                commander_->flyToPosition(global_target);
                break;
                
            case HEADING_SLEW:
                // Yön kontrolü
                float heading = atan2f(y, x);
                commander_->changeHeading(degrees(heading), HEADING, 0.5);
                break;
                
            case VELOCITY:
                // Velocity control (sınırlı)
                commander_->setLocalVelocity(x, y, z);
                break;
                
            case ALTITUDE_SLEW:
                // Yükseklik kontrolü
                commander_->changeAltitude(z, 2.0);
                break;
        }
    }
};
```

---

## 7. KARŞILAŞTIRMA

### 7.1 Slew Commands vs Direct Control

| Özellik | Slew Commands | Direct X/Y |
|---------|---------------|-------------|
| **Accuracy** | Yüksek (PID) | Düşük (drift) |
| **Smoothness** | Mükemmel | Zayıf |
| **Safety** | Yüksek (rate limit) | Düşük (ani komut) |
| **Stability** | Yüksek | Düşük |
| **Complexity** | Orta | Yüksek |
| **Maintenance** | Kolay | Zor (fork) |

### 7.2 Fiziksel Gerçekçilik

```
┌─────────────────────────────────────────────────────────────┐
│              FİZİKSEL GERÇEKLİK ANALİZİ                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ❌ DOĞRUDAN X/Y LOCAL CONTROL:                             │
│     - Aerodinamik imkansız                                │
│     - Fixed-wing sadece ileri uçuş yapabilir             │
│     - Yan hareket heading + thrust ile olur                 │
│     - Bu DAHİL İLE SLEW COMMANDS ile yapılıyor              │
│                                                             │
│  ✅ HEADING SLEW:                                         │
│     - Fiziksel olarak mümkün                              │
│     - Roll → Bank → Turn                                  │
│     - PID kontrolü ile stabil                             │
│     - Rate limiting ile güvenli                           │
│                                                             │
│  ✅ SPEED SLEW:                                           │
│     - Throttle → TECS → Airspeed                         │
│     - Acceleration limiting ile smooth                    │
│                                                             │
│  ✅ ALTITUDE SLEW:                                       │
│     - Pitch → Elevator → Climb/Descent                    │
│     - Rate limiting ile güvenli                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. SONUÇLAR

### 8.1 Fiziksel Gerçekler

1. **Fixed-wing aerodinamiği:** X/Y lokal kontrol mümkün değil
2. **Slew commands:** Bu sınırlamayı aşmanın tek yolu
3. **ArduPilot tasarımı:** L1 + TECS ile fiziksel olarak doğru
4. **Alternatifler:** Multirotor kullan veya fork yap

### 8.2 Öneri

**Vision tracking için:**
```cpp
// FollowMode'da Proportional Navigation + Slew Commands hibriti

void FollowMode::updatePursuit(const UAVState& own_state, const UAVState& target) {
    // 1. Bearing hesapla
    float los_bearing = geo::bearing(own_state.position, target.position);
    
    // 2. Lead angle hesapla (PN)
    float los_rate = calculateLOSRate(los_bearing);
    float pn_offset = pn_gain_ * los_rate * t_go;
    float intercept_bearing = los_bearing + pn_offset;
    
    // 3. Slew commands gönder
    commander_->changeHeading(degrees(intercept_bearing), HEADING, 0.5);
    commander_->changeAirspeed(pursuit_speed_ms, 3.0);
    
    // 4. Altitude control
    commander_->changeAltitude(target.position.altitude, 2.0);
}
```

### 8.3 Yapılabilirlik Skoru

| Özellik | Skor | Açıklama |
|---------|------|----------|
| Heading control | 9/10 | Slew ile mükemmel |
| Speed control | 10/10 | Slew ile mükemmel |
| Altitude control | 10/10 | Slew ile mükemmel |
| X/Y position | 2/10 | Sadece heading ile yaklaşık |
| X/Y velocity | 1/10 | Çok zayıf, drift yüksek |
| Vision tracking | 8/10 | PN + slew ile iyi |

---

*Döküman v1.0 - 2026-04-14*
