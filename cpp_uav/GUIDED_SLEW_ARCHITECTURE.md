# GUIDED SLEW COMMANDS MİMARİSİ - DETAYLI AÇIKLAMA

> **Amaç:** ArduPilot'un Guided Slew Commands özelliğini derinlemesine anlamak ve implementasyon için hazırlanmak

---

## 📋 İÇİNDEKİLER

1. [Slew Commands Nedir?](#1-slew-commands-nedir)
2. [ArduPilot İç Yapısı](#2-ardupilot-iç-yapısı)
3. [MAVLink Mesaj Akışı](#3-mavlink-mesaj-akışı)
4. [Control Loop](#4-control-loop)
5. [PID Kontrolü](#5-pid-kontrolü)
6. [Timeout Mekanizması](#6-timeout-mekanizması)
7. [Implementasyon Detayları](#7-implementasyon-detayları)
8. [Örnek Senaryolar](#8-örnek-senaryolar)

---

## 1. SLEW COMMANDS NEDİR?

### 1.1 Temel Kavram

**Slew (Rate Limiting):** Bir değerin diğerine geçiş hızıını sınırlama

```
Konumsal Komut (DO_REPOSITION):
  Başlangıç ──────────────────────────────────> Bitiş
  (Ani geçiş, sert manevra)

Slew Komutu (GUIDED_CHANGE_HEADING):
  Başlangıç ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲> Bitiş
  (Yumuşak geçiş, acceleration limitli)
```

### 1.2 Neden Gerekli?

| Problem | Slew ile Çözüm |
|---------|----------------|
| Sert manevra → Stall | Yumuşak geçiş → Stabil |
| Komut flood → Overload | Rate limit → Kontrollü |
| Ani değişim → Comfort sorun | Accel limit → Yumuşak |

### 1.3 Üç Ana Komut

```
┌─────────────────────────────────────────────────────────────┐
│                   GUIDED SLEW COMMANDS                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. HEADING (MAV_CMD 40002)                                │
│     ├── Vehicle Heading (geminin baktığı yön)              │
│     └── Course Over Ground (yerde izlediği yol)             │
│                                                             │
│  2. SPEED (MAV_CMD 40000)                                  │
│     └── Airspeed (hava hızı)                               │
│                                                             │
│  3. ALTITUDE (MAV_CMD 40001)                               │
│     └── Yükseklik + tırmanma hızı                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. ARDUPILOT İÇ YAPISI

### 2.1 GuidedState Struct

[`Plane.h:574-606`](cpp_uav_external/ardupilot_repo/ArduPlane/Plane.h:574-606)

```cpp
struct GuidedState {
    // === DİREKTK RPY KONTROLÜ (slew yok) ===
    Vector3l forced_rpy_cd;        // Roll/Pitch/Yaw (centidegrees)
    Vector3l last_forced_rpy_ms;   // Son komut zamanı
    float forced_throttle;         // Throttle (%)
    uint32_t last_forced_throttle_ms;
    
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // === SLEW KONTROLÜ (rate limitli) ===
    
    // 1. Airspeed
    float target_airspeed_cm = -1;       // Hedef hızı (cm/s)
    float target_airspeed_accel;         // İvme limiti (cm/s²)
    uint32_t target_airspeed_time_ms;    // Son güncelleme
    
    // 2. Altitude
    Location target_location;            // Hedef konum
    float target_alt_rate;               // Tırmanma hızı (cm/s)
    uint32_t target_alt_time_ms;         // Son güncelleme
    
    // 3. Heading
    float target_heading = -4;           // Hedef açı (radyan)
    float target_heading_accel_limit;    // İvme limiti (rad/s²)
    uint32_t target_heading_time_ms;     // Son güncelleme
    guided_heading_type_t target_heading_type;  // HEADING veya COG
    bool target_heading_limit;           // Saturasyon durumu
#endif
};
```

### 2.2 Heading Type'ları

[`defines.h:175-179`](cpp_uav_external/ardupilot_repo/ArduPlane/defines.h:175-179)

```cpp
enum guided_heading_type_t {
    GUIDED_HEADING_NONE = 0,     // Kapalı, normal navigasyon
    GUIDED_HEADING_COG,          // Course Over Ground (yörünge)
    GUIDED_HEADING_HEADING,      // Vehicle Heading (gemi başı)
};
```

**Fark:**

```
         COG (Course Over Ground)
              ↑
              │  Rüzgar
              │  →→→
              │
    ══════════╪═══════════  Uçak yörüngesi
              │
              ↓
      HEADING (Vehicle Heading)
```

- **HEADING:** Uçağın baktığı yön (rudder + aileron)
- **COG:** Yerde izlediği yol (rüzgarı ihmal eder)

**Kullanım:**
- Vision tracking → COG (hedefe doğru git)
- Precision maneuver → HEADING (tam açı)

---

## 3. MAVLINK MESAJ AKIŞI

### 3.1 Heading Command

```cpp
// MAV_CMD_GUIDED_CHANGE_HEADING (40002)
mavlink_msg_command_int_pack(
    sys_id, comp_id, &msg,
    target_sys, target_comp,
    MAV_FRAME_GLOBAL_INT,        // Frame (önemsiz)
    MAV_CMD_GUIDED_CHANGE_HEADING,  // Command
    0, 0,                        // Confirmation, reserved
    HEADING_TYPE_HEADING,        // param1: 0=NONE, 1=COG, 2=HEADING
    90.0f,                       // param2: Heading (derece)
    0.5f,                        // param3: Accel limit (rad/s²)
    0, 0, 0,                     // param4-6: unused
    0, 0                         // x, z: unused
);
```

**Parametre Detayları:**

| Parametre | Değer Aralığı | Açıklama |
|-----------|---------------|----------|
| param1 | 0, 1, 2 | HEADING_TYPE |
| param2 | 0-360° | Hedef heading |
| param3 | 0.05-1.0 rad/s² | İvme limiti (0 = maks) |

### 3.2 Speed Command

```cpp
// MAV_CMD_GUIDED_CHANGE_SPEED (40000)
mavlink_msg_command_int_pack(
    sys_id, comp_id, &msg,
    target_sys, target_comp,
    MAV_FRAME_GLOBAL_INT,
    MAV_CMD_GUIDED_CHANGE_SPEED,
    0, 0,
    SPEED_TYPE_AIRSPEED,        // param1: 0=Airspeed
    25.0f,                       // param2: Hedef speed (m/s)
    2.0f,                       // param3: İvme (m/s², 0=maks)
    0, 0, 0, 0, 0
);
```

### 3.3 Altitude Command

```cpp
// MAV_CMD_GUIDED_CHANGE_ALTITUDE (40001)
mavlink_msg_command_int_pack(
    sys_id, comp_id, &msg,
    target_sys, target_comp,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  // Frame!
    MAV_CMD_GUIDED_CHANGE_ALTITUDE,
    0, 0, 0, 0,
    150.0f,                     // z: Hedef yükseklik (m)
    3.0f,                       // param3: Climb rate (m/s, 0=maks)
    0, 0
);
```

---

## 4. CONTROL LOOP

### 4.1 ArduPilot update() Akışı

[`mode_guided.cpp:30-102`](cpp_uav_external/ardupilot_repo/ArduPlane/mode_guided.cpp:30-102)

```cpp
void ModeGuided::update() {
    // ┌─────────────────────────────────────────────────────┐
    // │ 1. ROLL KONTROLÜ                                    │
    // └─────────────────────────────────────────────────────┘
    
    if (forced_rpy_active) {
        // Direkt RPY komutu varsa (timeout süresince)
        nav_roll_cd = forced_rpy_cd.x;
        
    } else if (target_heading_type != GUIDED_HEADING_NONE) {
        // ✨ SLEW HEADING KONTROLÜ
        
        // 1. Heading hatasını hesapla
        float error = target_heading - current_heading;
        error = wrap_PI(error);  // -π ile +π arası
        
        // 2. Bank angle limitini hesapla
        float bank_limit = atan(accel_limit / 9.81) * (180/π) * 100;
        
        // 3. PID ile roll komutunu hesapla
        float desired_roll = g2.guidedHeading.update_error(error, delta);
        
        // 4. Sınırla ve uygula
        nav_roll_cd = constrain(desired_roll, -bank_limit, bank_limit);
        
    } else {
        // Normal navigasyon
        calc_nav_roll();
    }
    
    // ┌─────────────────────────────────────────────────────┐
    // │ 2. PITCH KONTROLÜ                                   │
    // └─────────────────────────────────────────────────────┘
    
    if (forced_rpy_active) {
        nav_pitch_cd = forced_rpy_cd.y;
    } else {
        calc_nav_pitch();  // TECS tarafından
    }
    
    // ┌─────────────────────────────────────────────────────┐
    // │ 3. THROTTLE KONTROLÜ                                │
    // └─────────────────────────────────────────────────────┘
    
    if (forced_throttle_active) {
        SRV_Channels::set_output_scaled(k_throttle, forced_throttle);
    } else {
        calc_throttle();  // TECS tarafından
    }
}
```

### 4.2 Loop Frekansı

```
┌─────────────────────────────────────────────────────────────┐
│                  ARDUPILOT CONTROL LOOPS                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Fast Loop (400Hz)                                          │
│  ├── Servo output                                          │
│  └── Rate gyros                                            │
│                                                             │
│  Medium Loop (50Hz)                                         │
│  ├── Attitude control                                      │
│  ├── Navigation (L1)                                       │
│  └── Guided update()                                       │
│                                                             │
│  Slow Loop (10Hz)                                          │
│  ├── TECS                                                  │
│  ├── Mission                                               │
│  └── Parameters                                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Guided mode 50Hz'de çalışır** → Her 20ms'de bir kontrol

---

## 5. PID KONTROLÜ

### 5.1 Heading PID

ArduPilot `AC_PID` sınıfını kullanır:

```cpp
// ArduPilot kodundan (mode_guided.cpp:65)
const float desired = plane.g2.guidedHeading.update_error(
    error,           // Heading hatası (radyan)
    delta,           // Zaman adımı (saniye)
    limit_flag       // Saturasyon kontrolü
);
```

**PID Yapısı:**

```
          ┌─────────────────────────────────────────┐
          │                                         │
Heading  ──→(+)─→[ Kp ]─┐                          │
Error       │           │                          │
            │        [ Ki ]─┐                      │
            │           │   │                     [∫]─→ Roll Command
            │           └──[Kd]─┘                  │
            │                │                     │
            └────────────────┴─────────────────────┘
            
Kp = Proportional (anlık hata)
Ki = Integral (birikmiş hata)
Kd = Derivative (hata değişim hızı)
```

### 5.2 Acceleration Limit

İvme limiti → Bank angle limit:

```
accel_limit = 0.5 rad/s²

bank_angle_max = atan(accel_limit / g)
               = atan(0.5 / 9.81)
               = atan(0.051)
               ≈ 2.9°

Gerçekte daha büyük değerler kullanılır:
- 5 m/s² → ~27° bank
- 10 m/s² → ~45° bank
```

---

## 6. TIMEOUT MEKANİZMASI

### 6.1 Timeout Parametresi

```cpp
// ArduPilot parametresi
GUIDED_TIMEOUT = 3.0  // saniye
```

### 6.2 Timeout Kontrolü

```cpp
// mode_guided.cpp:40-43
if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
    millis() - plane.guided_state.last_forced_rpy_ms.x < 
    plane.g2.guided_timeout*1000.0f) {
    // Timeout değil, komut aktif
}
```

**Akış:**

```
Komut gönder
    │
    ▼
last_command_ms = now
    │
    ├─── 3 saniye geçti mi? ──EVET──> Normal navigasyona dön
    │
    └─── HAYIR ──> Komut devam et
```

### 6.3 Timeout Sonrası

```cpp
// Komut timeout olduğunda
if (timeout) {
    // GuidedState değerleri sıfırlanmaz
    // Sadece kontrol normal navigasyona geçer
    
    nav_roll_cd = calc_nav_roll();    // L1 kontrolü
    nav_pitch_cd = calc_nav_pitch();  // TECS
    throttle = calc_throttle();       // TECS
}
```

---

## 7. IMPLEMENTASYON DETAYLARI

### 7.1 FlightCommander Genişletme

```cpp
// flight_commander.hpp
class FlightCommander {
public:
    // Mevcut...
    
    // YENİ: Slew Commands
    enum class HeadingType {
        NONE = 0,
        COG = 1,       // Course Over Ground
        HEADING = 2    // Vehicle Heading
    };
    
    bool changeHeading(
        float heading_deg,
        HeadingType type = HeadingType::HEADING,
        float accel_limit_rad_s2 = 0.5f
    );
    
    bool changeAirspeed(
        float airspeed_ms,
        float acceleration_ms2 = 0.0f  // 0 = maksimum
    );
    
    bool changeAltitude(
        float altitude_m,
        float climb_rate_ms = 0.0f,    // 0 = maksimum
        MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    );
    
private:
    struct {
        uint32_t heading_ms = 0;
        uint32_t airspeed_ms = 0;
        uint32_t altitude_ms = 0;
    } slew_times_;
};
```

### 7.2 Implementation

```cpp
// flight_commander.cpp
bool FlightCommander::changeHeading(
    float heading_deg,
    HeadingType type,
    float accel_limit_rad_s2
) {
    std::lock_guard lock(cmd_mutex_);
    
    // 1. Validation
    if (heading_deg < 0.0f || heading_deg >= 360.0f) {
        LOG_WARN("Invalid heading: {:.1f}", heading_deg);
        return false;
    }
    
    if (accel_limit_rad_s2 < 0.05f) {
        accel_limit_rad_s2 = 0.05f;  // Minimum
    }
    
    // 2. MAVLink mesajı
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_INT,
        MAV_CMD_GUIDED_CHANGE_HEADING,
        0, 0,
        static_cast<float>(type),     // param1
        heading_deg,                   // param2
        accel_limit_rad_s2,           // param3
        0, 0, 0,                      // param4-6
        0, 0                          // x, z
    );
    
    slew_times_.heading_ms = nowMs();
    commands_sent_++;
    
    LOG_INFO("Heading: {:.1f}° type={} accel={:.2f}",
             heading_deg, static_cast<int>(type), accel_limit_rad_s2);
    
    return sendMAVLinkMessage(msg);
}

bool FlightCommander::changeAirspeed(
    float airspeed_ms,
    float acceleration_ms2
) {
    std::lock_guard lock(cmd_mutex_);
    
    // Validation
    const float MIN_AIRSPEED = 12.0f;
    const float MAX_AIRSPEED = 35.0f;
    
    if (airspeed_ms < MIN_AIRSPEED || airspeed_ms > MAX_AIRSPEED) {
        LOG_WARN("Airspeed out of range: {:.1f}", airspeed_ms);
        return false;
    }
    
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_INT,
        MAV_CMD_GUIDED_CHANGE_SPEED,
        0, 0,
        SPEED_TYPE_AIRSPEED,         // param1
        airspeed_ms,                 // param2
        acceleration_ms2,            // param3
        0, 0, 0, 0, 0
    );
    
    slew_times_.airspeed_ms = nowMs();
    commands_sent_++;
    
    LOG_INFO("Airspeed: {:.1f} m/s accel={:.1f}", airspeed_ms, acceleration_ms2);
    
    return sendMAVLinkMessage(msg);
}

bool FlightCommander::changeAltitude(
    float altitude_m,
    float climb_rate_ms,
    MAV_FRAME frame
) {
    std::lock_guard lock(cmd_mutex_);
    
    // Validation
    const float MIN_ALT = 30.0f;
    const float MAX_ALT = 400.0f;
    
    if (altitude_m < MIN_ALT || altitude_m > MAX_ALT) {
        LOG_WARN("Altitude out of range: {:.1f}", altitude_m);
        return false;
    }
    
    if (climb_rate_ms < 0.0f) {
        climb_rate_ms = 0.0f;  // Maksimum
    }
    
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        frame,                           // Frame önemli!
        MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0, 0, 0,
        altitude_m,                      // z
        climb_rate_ms,                   // param3
        0, 0
    );
    
    slew_times_.altitude_ms = nowMs();
    commands_sent_++;
    
    LOG_INFO("Altitude: {:.1f} m climb={:.1f} m/s", altitude_m, climb_rate_ms);
    
    return sendMAVLinkMessage(msg);
}
```

### 7.3 Config Entegrasyonu

```yaml
# config/default.yaml
guided_slew:
  enabled: true
  
  heading:
    max_bank_angle_deg: 30.0    # Maksimum bank angle
    accel_limit: 0.5            # rad/s²
    update_rate_hz: 10.0        # Komut frekansı
    use_cog: false              # true = COG, false = HEADING
    
  speed:
    min_ms: 12.0
    max_ms: 35.0
    max_accel_ms2: 5.0
    update_rate_hz: 5.0
    
  altitude:
    min_m: 30.0
    max_m: 400.0
    max_climb_rate_ms: 5.0
    max_descent_rate_ms: 3.0
    update_rate_hz: 5.0
```

---

## 8. ÖRNEK SENARYOLAR

### 8.1 Senaryo 1: Vision-Based Takip

**Durum:** Kamera bir hedef tespit etti, sağ üstte

```
┌─────────────────────────────────────────────────────────────┐
│                  VISION-BASED TRACKING                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Kamera frame'ı analiz et                                │
│     └──> Hedef: (dx=+150px, dy=-100px)                     │
│                                                             │
│  2. Pixel offset'ı açıya çevir                              │
│     └──> bearing_to_target = +15°                          │
│                                                             │
│  3. Heading hesapla                                        │
│     current_heading = 270°                                 │
│     target_heading = 270° + 15° = 285°                    │
│                                                             │
│  4. Slew command gönder                                    │
│     commander->changeHeading(285.0, HEADING, 0.5);         │
│                                                             │
│  5. 10Hz'de tekrarla (timeout = 3s)                        │
│     while (tracking) {                                     │
│         heading = calculateHeading();                       │
│         commander->changeHeading(heading, HEADING, 0.5);    │
│         sleep(100ms);                                      │
│     }                                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 Senaryo 2: Hız Yakınlaştırma

**Durum:** Hedefe yaklaşıyor, hızlanmalı

```
// Hedef mesafe 200m → pursuit mode
if (distance < 200.0) {
    // Pursuit speed
    commander->changeAirspeed(30.0, 3.0);  // 30 m/s, 3 m/s² ivme
} else {
    // Cruise speed
    commander->changeAirspeed(20.0, 2.0);  // 20 m/s
}
```

### 8.3 Senaryo 3: Altitude Hold

**Durum:** Sabit irtifada takip

```
// Her 5 saniyede bir
if (now - last_alt_check > 5.0s) {
    float current_alt = own_state.position.altitude;
    float error = target_alt - current_alt;
    
    if (fabs(error) > 10.0) {
        // 10m fark var, düzelt
        commander->changeAltitude(target_alt, 2.0);
    }
    
    last_alt_check = now;
}
```

### 8.4 Senaryo 4: Kombo Komut

**Durum:** Tam kontrol (heading + speed + altitude)

```cpp
// Hedef: 45° açı, 25 m/s, 150m irtifa
struct TargetControl {
    float heading_deg = 45.0f;
    float airspeed_ms = 25.0f;
    float altitude_m = 150.0f;
};

void updateTracking(const TargetControl& target) {
    auto now = steady_clock::now();
    
    // Heading - 10Hz
    if (now - last_heading_time > 100ms) {
        commander_->changeHeading(
            target.heading_deg,
            FlightCommander::HEADING,
            0.5f
        );
        last_heading_time = now;
    }
    
    // Speed - 5Hz
    if (now - last_speed_time > 200ms) {
        commander_->changeAirspeed(
            target.airspeed_ms,
            3.0f
        );
        last_speed_time = now;
    }
    
    // Altitude - 5Hz
    if (now - last_alt_time > 200ms) {
        commander_->changeAltitude(
            target.altitude_m,
            2.0f
        );
        last_alt_time = now;
    }
}
```

---

## 9. GÜVENLİK

### 9.1 SafetyGate Entegrasyonu

```cpp
bool FlightCommander::changeHeading(
    float heading_deg,
    HeadingType type,
    float accel_limit_rad_s2
) {
    // Safety check
    if (safety_gate_) {
        auto own_state = world_model_->getOwnState();
        
        // 100m ileride nerede oluruz?
        auto projected = projectPosition(
            own_state.position,
            heading_deg,
            100.0  // metre
        );
        
        // Boundary check
        if (!safety_gate_->isInsideBoundary(projected)) {
            LOG_WARN("Heading {:.1f}° would violate boundary", heading_deg);
            return false;
        }
        
        // HSS check
        auto hss_zones = world_model_->getActiveHSSZones();
        if (safety_gate_->segmentIntersectsAnyHSS(
            own_state.position, projected, hss_zones, 40.0
        )) {
            LOG_WARN("Heading {:.1f}° would intersect HSS", heading_deg);
            return false;
        }
    }
    
    // Komut gönder...
}
```

### 9.2 Watchdog

```cpp
class SlewWatchdog {
public:
    void update(const std::string& command_type) {
        last_commands_[command_type] = now();
        
        // Timeout kontrolü
        for (auto& [type, last_time] : last_commands_) {
            if (now() - last_time > 3.0s) {
                LOG_WARN("Slew command timeout: {}", type);
                // Fallback
                triggerFallback(type);
            }
        }
    }
    
private:
    std::unordered_map<std::string, uint64_t> last_commands_;
};
```

---

## 10. PERFORMANS

### 10.1 Gecikme Analizi

```
┌─────────────────────────────────────────────────────────────┐
│                    LATENCY BREAKDOWN                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Vision Processing      ~10-30ms                           │
│       │                                                   │
│       ▼                                                   │
│  Heading Calculation    ~5ms                               │
│       │                                                   │
│       ▼                                                   │
│  MAVLink Encode        ~2ms                               │
│       │                                                   │
│       ▼                                                   │
│  Network (UDP)         ~1-5ms                             │
│       │                                                   │
│       ▼                                                   │
│  ArduPilot Receive      ~1ms                              │
│       │                                                   │
│       ▼                                                   │
│  Command Decode         ~1ms                              │
│       │                                                   │
│       ▼                                                   │
│  GuidedState Update     ~1ms                              │
│       │                                                   │
│       ▼                                                   │
│  PID Calculation        ~1ms                              │
│       │                                                   │
│       ▼                                                   │
│  Servo Output          ~20ms (1/50Hz)                    │
│                                                             │
│  TOPLAM: ~40-70ms (~2-3 Hz efektif)                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 10.2 Optimizasyon

**Yöntemler:**
1. **Öngörü:** Hedef pozisyonunu tahmin et
2. **Lead compensation:** Geçikmeyi dengele
3. **Rate limiting:** Komut flood'u önle

---

## 11. ÖZET

### 11.1 Avantajlar

✅ **Yumuşak Geçiş:** Acceleration limitli  
✅ **Rate Limit:** Komut flood'u yok  
✅ **Timeout Güvenliği:** 3s sonra fallback  
✅ **PID Kontrolü:** Stabil tracking  
✅ **Native Destek:** Fork gerekmez  

### 11.2 Dezavantajlar

⚠️ **X/Y Local Offset Yok** (heading ile simüle)  
⚠️ **Gecikme:** ~40-70ms toplam  
⚠️ **Parametre Bağımlılığı:** `AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`  

### 11.3 Kullanım

```cpp
// Basit kullanım
commander->changeHeading(90.0);     // 90°'ye dön
commander->changeAirspeed(25.0);    // 25 m/s
commander->changeAltitude(150.0);   // 150m

// Vision tracking
while (tracking) {
    float heading = vision->getTargetHeading();
    commander->changeHeading(heading, HEADING, 0.5);
    sleep(100ms);
}
```

---

*Döküman v1.0 - 2026-04-14*
