# SLEW COMMANDS DERİNLEMESİ - Sıfırdan İleri Seviyeye

> **Amaç:** ArduPilot Guided Slew Commands'ı her açıdan anlamak

---

## 📋 İÇİNDEKİLER

1. [Slew Kavramı](#1-slew-kavramı)
2. [ArduPilot Implementasyonu](#2-ardupilot-implementasyonu)
3. [Code Analysis](#3-code-analysis)
4. [Matematiksel Model](#4-matematiksel-model)
5. [Pratik Örnekler](#5-pratik-örnekler)
6. [Tuning Guide](#6-tuning-guide)

---

## 1. SLEW KAVRAMI

### 1.1 Tanım

**Slew (Rate Limiting):** Bir değişkenin başka bir değere geçiş hızını sınırlama

```
Ani Geçiş (Slew Yok):
  20 m/s ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━> 30 m/s
  (0.1 saniyede)

Slew Limitli (5 m/s²):
  20 m/s ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲> 30 m/s
  (2 saniyede, yumuşak geçiş)
```

### 1.2 Neden Gerekli?

| Problem | Slew ile Çözüm |
|---------|----------------|
| **Stall Riski** | Yumuşak maneuver → Lift korunur |
| **Passenger Comfort** | Yavaş geçiş → G force azalır |
| **Structural Stress** | Rate limit → Gerilme azalır |
| **Control Stability** | Smooth → PID daha stabil |
| **Sensor Noise** | Filtering → Sarsıntı yok |

### 1.3 Fixed-Wing Özel Gerekler

```
┌─────────────────────────────────────────────────────────────┐
│              FIXED-WING AERODİNAMİKLER                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Ani Heading Değişimi:                                      │
│    270° ────────────────────────> 0°                       │
│         (Sert, 90° bank!)                                  │
│                                                             │
│  Slew'li Heading Değişimi:                                 │
│    270° ╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱> 0°                         │
│         (0.5 rad/s², 30° bank max)                         │
│                                                             │
│  Sonuç: İkinci durumda                                     │
│    - Daha az G force                                       │
│    - Daha az stall riski                                   │
│    - Daha stabil tracking                                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. ARDUPILOT IMPLEMENTASYONU

### 2.1 Slew Command'lar

**3 Ana Komut:**

```cpp
// 1. HEADING CONTROL
MAV_CMD_GUIDED_CHANGE_HEADING  // 40002
├── param1: Heading Type (0=NONE, 1=COG, 2=HEADING)
├── param2: Target Heading (0-360°)
└── param3: Acceleration Limit (rad/s², 0=maks)

// 2. SPEED CONTROL  
MAV_CMD_GUIDED_CHANGE_SPEED      // 40000
├── param1: Speed Type (0=Airspeed, 1=Groundspeed)
├── param2: Target Speed (m/s)
└── param3: Acceleration (m/s², 0=maks)

// 3. ALTITUDE CONTROL
MAV_CMD_GUIDED_CHANGE_ALTITUDE   // 40001
├── z: Target Altitude (m)
├── param1: Altitude Type (0=Absolute, ...)
├── param2: Unused
└── param3: Climb Rate (m/s, 0=maks)
```

### 2.2 GuidedState Değişkenleri

```cpp
struct GuidedState {
    // HEADING
    float target_heading = -4;           // Radyan, -4 = inactive
    float target_heading_accel_limit;    // rad/s²
    uint32_t target_heading_time_ms;     // Son güncelleme
    guided_heading_type_t target_heading_type;  // NONE, COG, HEADING
    bool target_heading_limit;           // Saturasyon flag
    
    // SPEED
    float target_airspeed_cm = -1;       // cm/s, -1 = inactive
    float target_airspeed_accel;         // cm/s²
    uint32_t target_airspeed_time_ms;
    
    // ALTITUDE
    Location target_location;            // Hedef konum
    float target_alt_rate;               // cm/s climb rate
    uint32_t target_alt_time_ms;
};
```

---

## 3. CODE ANALYSIS

### 3.1 Heading Slew - Update Loop

[`mode_guided.cpp:48-71`](cpp_uav_external/ardupilot_repo/ArduPlane/mode_guided.cpp:48-71)

```cpp
// Her kontrol döngüsünde (50Hz)
if (target_heading_type != GUIDED_HEADING_NONE) {
    // 1. Zaman adımını hesapla
    uint32_t tnow = AP_HAL::millis();
    float delta = (tnow - target_heading_time_ms) * 1e-3f;
    target_heading_time_ms = tnow;
    
    // 2. Heading hatasını hesapla
    float error = 0.0f;
    if (target_heading_type == GUIDED_HEADING_HEADING) {
        error = wrap_PI(target_heading - ahrs.get_yaw_rad());
    } else {  // COG
        Vector2f gndspeed = ahrs.groundspeed_vector();
        float cog = atan2f(-gndspeed.y, -gndspeed.x) + M_PI;
        error = wrap_PI(target_heading - cog);
    }
    
    // 3. Bank angle limitini hesapla
    // a = v²/r → r = v²/a → bank = atan(v²/(a*g))
    float bank_limit = degrees(atan(accel_limit / GRAVITY_MSS)) * 100;
    bank_limit = MIN(bank_limit, roll_limit_cd);
    
    // 4. PID ile roll komutunu hesapla
    const float desired = g2.guidedHeading.update_error(error, delta, limit_flag);
    
    // 5. Sınırla ve uygula
    nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
    update_load_factor();  // G force kontrolü
}
```

**Akış Diyagramı:**

```
Target Heading ──→ Error Calculation ──→ PID ──→ Roll Command
                      ↓                              ↓
                 Wrap to ±π                   Bank Limit
                      ↓                              ↓
                  Current                    Servo Output
                   Heading
```

### 3.2 Speed Slew - Navigation Loop

[`navigation.cpp:190-206`](cpp_uav_external/ardupilot_repo/ArduPlane/navigation.cpp:190-206)

```cpp
// Her navigasyon döngüsünde (10Hz)
if (control_mode == &mode_guided && target_airspeed_cm > 0) {
    // 1. Zaman adımı
    uint32_t now = AP_HAL::millis();
    float delta = 1e-3f * (now - target_airspeed_time_ms);
    target_airspeed_time_ms = now;
    
    // 2. Delta amount (slew step)
    // acceleration * delta_time = speed_change
    float delta_amt = 100 * delta * target_airspeed_accel;
    
    // 3. İvme yönüne göre ekle/çıkar
    target_airspeed_cm += delta_amt;
    
    // 4. Min/Max sınırla
    if (target_airspeed_accel > 0) {
        // Hızlanıyoruz → max'a kadar
        target_airspeed_cm = constrain_float(
            MIN(target_airspeed_cm, target_airspeed_cm),
            airspeed_min * 100, 
            airspeed_max * 100
        );
    } else {
        // Yavaşlıyoruz → min'e kadar
        target_airspeed_cm = constrain_float(
            MAX(target_airspeed_cm, target_airspeed_cm),
            airspeed_min * 100,
            airspeed_max * 100
        );
    }
}
```

**Slew Mantığı:**

```
Current: 20 m/s
Target:  30 m/s
Accel:   5 m/s²

Loop 1 (0.1s): 20 + (5 * 0.1) = 20.5 m/s
Loop 2 (0.1s): 20.5 + (5 * 0.1) = 21.0 m/s
Loop 3 (0.1s): 21.0 + (5 * 0.1) = 21.5 m/s
...
Loop N: 30 m/s (2 saniyede ulaşır)
```

### 3.3 Altitude Slew - Update Loop

[`mode_guided.cpp:170-202`](cpp_uav_external/ardupilot_repo/ArduPlane/mode_guided.cpp:170-202)

```cpp
void ModeGuided::update_target_altitude() {
    if (target_alt_time_ms != 0 || !target_location_alt_is_minus_one()) {
        // 1. Zaman adımı
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - target_alt_time_ms);
        target_alt_time_ms = now;
        
        // 2. Altitude delta (cm)
        float delta_amt_f = delta * target_alt_rate;
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f);
        
        // 3. Mevcut irtifa
        const Location::AltFrame target_frame = target_location.get_alt_frame();
        int32_t current_alt_cm;
        current_loc.get_alt_cm(target_frame, current_alt_cm);
        
        // 4. Geçici hedef (bir adım ileri)
        int32_t temp_alt_cm = constrain_int32(
            target_location.alt,
            current_alt_cm - delta_amt_i,    // Minimum
            current_alt_cm + delta_amt_i     // Maksimum
        );
        
        // 5. Geçici hedefi uygula
        Location temp_location = target_location;
        temp_location.set_alt_cm(temp_alt_cm, target_frame);
        plane.set_target_altitude_location(temp_location);
        
        // Bir sonraki loop'ta tekrar, hedefe ulaşana kadar
    }
}
```

**Slew Mantığı:**

```
Current: 100 m
Target:  150 m
Rate:    3 m/s

Loop 1 (0.1s): 100 + (3 * 0.1) = 100.3 m
Loop 2 (0.1s): 100.3 + (3 * 0.1) = 100.6 m
Loop 3 (0.1s): 100.6 + (3 * 0.1) = 100.9 m
...
Loop N: 150 m (16.7 saniyede ulaşır)
```

---

## 4. MATEMATİKSEL MODEL

### 4.1 Slew Rate Limiting

**Temel Formül:**

```
y[k] = y[k-1] + sign(target - y[k-1]) * min(|target - y[k-1]|, rate * dt)
```

**Parametreler:**
- `y[k]`: Mevcut output
- `target`: Hedef değer
- `rate`: Maksimum slew rate (birim/s)
- `dt`: Zaman adımı (saniye)

### 4.2 Second-Order Slew (Acceleration Limit)

**Formül:**

```
v[k] = v[k-1] + sign(target - y[k-1]) * min(|target - y[k-1]| / dt, accel * dt)
y[k] = y[k-1] + v[k] * dt
```

**Değişkenler:**
- `v[k]`: Mevcut velocity
- `accel`: Maksimum acceleration (birim/s²)
- `y[k]`: Mevcut position

### 4.3 Heading Slew için Bank Angle

**Koordinat Dönüşümü:**

```
Horizontal Force Balance:
  F_centripetal = F_gravity_horizontal
  
  m * v² / r = m * g * tan(θ)
  
  θ = atan(v² / (r * g))
    = atan(a / g)
```

**Değişkenler:**
- `θ`: Bank angle (radyan)
- `v`: Velocity (m/s)
- `r`: Turn radius (m)
- `a`: Centripetal acceleration (m/s²)
- `g`: Gravity (9.81 m/s²)

**Örnek:**

```
v = 25 m/s
a = 5 m/s²
g = 9.81 m/s²

θ = atan(5 / 9.81)
  = atan(0.51)
  ≈ 27°
```

---

## 5. PRATİK ÖRNEKLER

### 5.1 Örnek 1: Basit Heading Slew

**Senaryo:** 270°'den 0°'ye dön (sağ)

```cpp
// Komut
commander->changeHeading(0.0f, HEADING, 0.5f);

// ArduPilot içindeki akış:
// Loop 0 (t=0.000s)
//   current = 270° (4.712 rad)
//   target = 0° (0.000 rad)
//   error = wrap(0 - 4.712) = 1.571 rad (90°)
//   accel_limit = 0.5 rad/s²
//   bank_limit = atan(0.5/9.81) = 0.051 rad = 2.9°
//   PID output = ~30° (ilk adımda agresif)
//   constrained to 2.9° → nav_roll = 2.9°

// Loop 1 (t=0.020s)
//   current = 270.1°
//   error = wrap(0 - 4.710) = 1.569 rad
//   PID output = ~28°
//   constrained to 2.9° → nav_roll = 2.9°

// ... gradual turn to 0°
```

### 5.2 Örnek 2: Speed Slew

**Senaryo:** 18 m/s'den 25 m/s'ye çık

```cpp
// Komut
commander->changeAirspeed(25.0f, 3.0f);

// ArduPilot içindeki akış:
// current = 1800 cm/s
// target = 2500 cm/s
// accel = 300 cm/s²

// Loop 0 (t=0.000s, dt=0.100s)
//   delta_amt = 100 * 0.1 * 300 = 3000 cm/s
//   But: 3000 > (2500-1800) → clamp to difference
//   target_airspeed_cm = 2500 (doğrudan hedefe)

// Daha küçük ivme ile:
// accel = 100 cm/s²

// Loop 0 (dt=0.100s)
//   delta_amt = 100 * 0.1 * 100 = 1000 cm/s
//   target_airspeed_cm = 1800 + 1000 = 2800
//   constrained to [1200, 3500] → 2800

// Loop 1 (dt=0.100s)
//   target_airspeed_cm = 2800 + 1000 = 3500
//   constrained to 3500 (max)

// Loop 2 (dt=0.100s)
//   target_airspeed_cm = 3500 + 1000 = 4500
//   constrained to 3500 (max) → DONE
```

### 5.3 Örnek 3: Altitude Slew

**Senaryo:** 100m'den 150m'ye tırman (3 m/s ile)

```cpp
// Komut
commander->changeAltitude(150.0f, 3.0f);

// ArduPilot içindeki akış:
// current_alt = 10000 cm
// target_alt = 15000 cm
// climb_rate = 300 cm/s

// Loop 0 (dt=0.100s)
//   delta_amt = 0.1 * 300 = 30 cm
//   temp_alt = constrain(15000, 10000-30, 10000+30)
//            = constrain(15000, 9970, 10030)
//            = 10030 cm
//   set_target_altitude(100.30m)

// Loop 1 (dt=0.100s)
//   temp_alt = constrain(15000, 10030-30, 10030+30)
//            = 10060 cm

// ... her loop'ta 30cm artış
// 500cm / 30cm per loop ≈ 17 loop ≈ 1.7 saniye
// Ama 3 m/s rate ile 500cm / 300 cm/s ≈ 1.67 saniye ✓
```

---

## 6. TUNING GUIDE

### 6.1 Heading Tuning

**Parametreler:**

| Parametre | Varsayılan | Aralık | Etki |
|-----------|-----------|--------|------|
| `accel_limit` | 0.5 rad/s² | 0.05-1.0 | Dönüş hızı |
| `GUIDED_HEADING_P` | 0.4 | 0.1-1.0 | Response agresifliği |
| `GUIDED_HEADING_I` | 0.0 | 0.0-0.1 | Steady-state error |
| `GUIDED_HEADING_D` | 0.0 | 0.0-0.05 | Damping |

**Tuning Stratejisi:**

```
1. P'den başla (I=0, D=0)
   - Çok düşük → Yavaş response
   - Çok yüksek → Oscillation
   
2. I ekle (steady-state error için)
   - Yavaşça artır
   - Oscillasyon başlayınca azalt
   
3. D ekle (damping için)
   - Overshoot'i azalt
   - Çok yüksek → Noise amplification
```

**Örnek Tuning:**

```yaml
# Yavaş/Smooth (passenger comfort)
accel_limit: 0.3    # Yavaş
P: 0.3               # Düşük gain
I: 0.0
D: 0.01

# Orta (general purpose)
accel_limit: 0.5    # Orta
P: 0.4
I: 0.0
D: 0.0

# Hızlı/Agresif (racing)
accel_limit: 1.0    # Hızlı
P: 0.8               # Yüksek gain
I: 0.05
D: 0.02
```

### 6.2 Speed Tuning

**Parametreler:**

| Parametre | Varsayılan | Aralık | Etki |
|-----------|-----------|--------|------|
| `acceleration` | 0.0 (maks) | 0.0-10.0 m/s² | İvme limiti |

**Öneriler:**

```
Comfort:    1-2 m/s²
Normal:     3-5 m/s²
Agresif:    5-10 m/s²
```

### 6.3 Altitude Tuning

**Parametreler:**

| Parametre | Varsayılan | Aralık | Etki |
|-----------|-----------|--------|------|
| `climb_rate` | 0.0 (maks) | 0.0-10.0 m/s | Tırmanma hızı |

**Öneriler:**

```
Gentle:    1-2 m/s
Normal:    3-5 m/s
Fast:      5-10 m/s
```

---

## 7. PERFORMANS ANALİZİ

### 7.1 Gecikme

```
┌─────────────────────────────────────────────────────────────┐
│                    SLEW COMMAND LATENCY                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Companion Computer                                         │
│  │                                                          │
│  ├─ Vision Processing: ~10-30ms                            │
│  ├─ Heading Calc:      ~5ms                               │
│  ├─ MAVLink Encode:   ~2ms                               │
│  └─ Network (UDP):     ~1-5ms                             │
│                                                             │
│  ArduPilot                                                  │
│  │                                                          │
│  ├─ MAVLink Decode:   ~1ms                               │
│  ├─ Command Process:  ~1ms                               │
│  └─ GuidedState Update: ~1ms                              │
│                                                             │
│  Control Loop                                               │
│  │                                                          │
│  ├─ PID Calculation:  ~1ms                                │
│  └─ Servo Output:     ~20ms (50Hz)                        │
│                                                             │
│  TOPLAM: ~40-70ms                                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Update Rate

| Component | Rate | Period |
|-----------|------|--------|
| Vision | 30 Hz | 33 ms |
| MAVLink Send | 10-50 Hz | 20-100 ms |
| Guided Update | 50 Hz | 20 ms |
| Servo Output | 50 Hz | 20 ms |

**Bottleneck:** Vision processing veya MAVLink rate

### 7.3 Accuracy

| Command | Accuracy | Noise |
|---------|----------|-------|
| Heading | ±2° | ±0.5° |
| Speed | ±0.5 m/s | ±0.2 m/s |
| Altitude | ±1 m | ±0.3 m |

---

## 8. TROUBLESHOOTING

### 8.1 Sorun: Slow Response

**Sebepler:**
- Acceleration limit çok düşük
- PID gain çok düşük
- Update rate çok yavaş

**Çözüm:**
```cpp
// Acceleration'ı artır
commander->changeHeading(heading, HEADING, 1.0f);  // 0.5 → 1.0

// Update rate'i artır
while (tracking) {
    commander->changeHeading(heading, HEADING, 0.5);
    sleep(50ms);  // 100ms → 50ms
}
```

### 8.2 Sorun: Overshoot

**Sebepler:**
- Acceleration limit çok yüksek
- PID D termi eksik
- Update rate çok yüksek

**Çözüm:**
```cpp
// Acceleration'ı azalt
commander->changeHeading(heading, HEADING, 0.3f);  // 0.5 → 0.3

// D termi ekle
setParam("GUIDED_HEADING_D", 0.02);
```

### 8.3 Sorun: Oscillation

**Sebepler:**
- PID P termi çok yüksek
- I term gereksiz
- Noise amplification

**Çözüm:**
```cpp
// P'yi azalt
setParam("GUIDED_HEADING_P", 0.2);  // 0.4 → 0.2

// I'yi sıfırla
setParam("GUIDED_HEADING_I", 0.0);

// D'yi ekle (damping)
setParam("GUIDED_HEADING_D", 0.01);
```

---

## 9. BEST PRACTICES

### 9.1 Command Frequency

```
Heading: 10-20 Hz (50-100ms interval)
Speed:   5-10 Hz  (100-200ms interval)
Altitude: 5-10 Hz  (100-200ms interval)
```

### 9.2 Acceleration Limits

```
Smooth tracking:  0.3-0.5 rad/s²
Normal:          0.5-0.7 rad/s²
Agresif:          0.7-1.0 rad/s²
```

### 9.3 Safety

```cpp
// Her zaman validation ile
if (safety_gate_->validateHeading(heading)) {
    commander->changeHeading(heading, HEADING, accel);
} else {
    LOG_WARN("Unsafe heading rejected: {:.1f}°", heading);
}
```

---

## 10. SUMMARY

**Slew Commands = Rate Limiting + PID Control**

```
Input (Target) → Rate Limiter → PID → Output (Servo)
                   ↓
            Smooth Transition
```

**Avantajlar:**
- ✅ Yumuşak maneuver
- ✅ Stabil tracking
- ✅ G-force kontrolü
- ✅ Noise filtering

**Kullanım:**
```cpp
// Basit
commander->changeHeading(90.0);
commander->changeAirspeed(25.0);
commander->changeAltitude(150.0);

// Advanced
commander->changeHeading(90.0, HEADING, 0.5);
commander->changeAirspeed(25.0, 3.0);
commander->changeAltitude(150.0, 2.0);
```

---

*Döküman v1.0 - 2026-04-14*
