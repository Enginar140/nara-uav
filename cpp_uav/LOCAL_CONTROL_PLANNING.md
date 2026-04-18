# LOCAL CONTROL GELİŞTİRME PLANI

> **Oluşturulma Tarihi:** 2026-04-14  
> **Amaç:** ArduPilot GUIDED mode'da local komutlar (sağa git, yukarı git, hızlan vb.) göndermek için kapsamlı planlama ve risk analizi

---

## 📋 İÇİNDEKİLER

1. [Problem Tanımı](#1-problem-tanımı)
2. [ArduPilot Analizi](#2-ardupilot-analizi)
3. [Mevcut Durum](#3-mevcut-durum)
4. [Çözüm Seçenekleri](#4-çözüm-seçenekleri)
5. [Risk Analizi](#5-risk-analizi)
6. [Literatür Taraması](#6-literatür-taraması)
7. [Önerilen Çözüm](#7-önerilen-çözüm)
8. [Implementasyon Planı](#8-implementasyon-planı)
9. [Test Stratejisi](#9-test-stratejisi)
10. [Geriye Dönüş](#10-geriye-dönüş)

---

## 1. PROBLEM TANIMI

### 1.1 Mevcut Sorun

Sistemimiz şu anda sadece **global position komutları** gönderebiliyor (`DO_REPOSITION`):

```cpp
flyToPosition(lat, lon, alt)  // Sadece global koordinat
```

**Sınırlamalar:**
- ❌ Local offset komutu yok (örn: "100m sağa git")
- ❌ Doğrudan velocity control sınırlı
- ❌ Yumuşak geçiş (slew) kontrolü yok
- ❌ Rate-based kontrol yok

### 1.2 İhtiyaç

Görüntü tabanlı takip sistemi için şunlar gerekiyor:

1. **Heading kontrolü:** Hedefin açısına göre dön
2. **Speed kontrolü:** Hedefe göre hızlan/yavaşla
3. **Altitude kontrolü:** Yüksel/alçal
4. **Rate kontrolü:** Yumuşak geçişlerle komut gönder

---

## 2. ARDUPILOT ANALİZİ

### 2.1 Guided Mode Yapısı

ArduPilot Plane'in GUIDED mode'u 3 farklı kontrol seviyesi sunuyor:

#### Seviye 1: Basit Global Position (Mevcut)
```cpp
MAV_CMD_DO_REPOSITION  // Global lat/lon/alt
```

#### Seviye 2: Local NED (Sınırlı!)
```cpp
SET_POSITION_TARGET_LOCAL_NED  // SADECE altitude çalışıyor!
```

**Kritik Bulgu:** [`GCS_MAVLink_Plane.cpp:1081-1101`](cpp_uav_external/ardupilot_repo/ArduPlane/GCS_MAVLink_Plane.cpp:1081-1101)

```cpp
void GCS_MAVLINK_Plane::handle_set_position_target_local_ned(...) {
    // sadece local moves for now
    if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
        return;
    }
    // just do altitude for now  ⚠️ X/Y YOK!
    plane.next_WP_loc.alt += -packet.z*100.0;
}
```

#### Seviye 3: Guided Slew Commands (GELİŞMİŞ) ✅

`AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED` ile aktif:

| Komut | MAV_CMD | Parametreler | Açıklama |
|-------|---------|--------------|----------|
| **Speed** | 40000 | param1=type, param2=airspeed, param3=accel | Airspeed kontrolü |
| **Altitude** | 40001 | z=alt, param3=climb_rate | Yükseklik kontrolü |
| **Heading** | 40002 | param1=type, param2=heading, param3=accel_limit | Heading kontrolü |

### 2.2 GuidedState Struct

[`Plane.h:574-606`](cpp_uav_external/ardupilot_repo/ArduPlane/Plane.h:574-606)

```cpp
struct GuidedState {
    // RPY direct control (forced)
    Vector3l forced_rpy_cd;
    Vector3l last_forced_rpy_ms;
    
    // Throttle direct control
    float forced_throttle;
    uint32_t last_forced_throttle_ms;
    
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // Airspeed adjustments
    float target_airspeed_cm = -1;
    float target_airspeed_accel;
    uint32_t target_airspeed_time_ms;
    
    // Altitude adjustments
    Location target_location;
    float target_alt_rate;
    uint32_t target_alt_time_ms = 0;
    
    // Heading track
    float target_heading = -4;
    float target_heading_accel_limit;
    uint32_t target_heading_time_ms;
    guided_heading_type_t target_heading_type;
    bool target_heading_limit;
#endif
};
```

### 2.3 Heading Type'ları

[`defines.h:175-179`](cpp_uav_external/ardupilot_repo/ArduPlane/defines.h:175-179)

```cpp
enum guided_heading_type_t {
    GUIDED_HEADING_NONE = 0,  // Heading kontrolü kapalı
    GUIDED_HEADING_COG,       // Course Over Ground (yörünge açısı)
    GUIDED_HEADING_HEADING,   // Vehicle heading (gemi başı)
};
```

### 2.4 Timeout Mekanizması

Tüm guided komutları `GUIDED_TIMEOUT` parametresi ile zaman aşımına uğrar:

- **Varsayılan:** 3 saniye
- **Anlamı:** Komut gelmezse normal navigasyona dön
- **Güvenlik:** İletişim kesilirse otomatatik fallback

---

## 3. MEVCUT DURUM

### 3.1 FlightCommander Analizi

[`flight_commander.cpp`](cpp_uav/src/comm/flight_commander.cpp)

```cpp
// Mevcut metodlar:
bool flyToPosition(lat, lon, alt, speed);     // DO_REPOSITION
bool flyToLocalPosition(north, east, down);   // SET_POS_TARGET_LOCAL_NED
bool setLocalVelocity(vn, ve, vd, yaw_rate);  // SET_POS_TARGET_LOCAL_NED
```

**Sorunlar:**
1. `flyToLocalPosition()` - X/Y çalışmıyor (ArduPilot sınırlaması)
2. `setLocalVelocity()` - Çalışıyor ama position kontrolü yok
3. Slew commands yok

### 3.2 FollowMode Analizi

[`follow_mode.cpp`](cpp_uav/src/mission/follow_mode.cpp)

Mevcut Proportional Navigation implementasyonu:

```cpp
// PN hesaplama
LOS_rate = (bearing_t - bearing_t-1) / dt
pn_offset = pn_gain * LOS_rate * t_go
intercept_bearing = LOS_bearing + pn_offset
```

**Sorun:**
- Global waypoint gönderiyor
- Fine control yok
- Rate-based değil position-based

---

## 4. ÇÖZÜM SEÇENEKLERİ

### Seçenek 1: Guided Slew Commands (ÖNERİLEN) ⭐

**Avantajları:**
- ✅ ArduPilot native desteği
- ✅ Rate/slew kontrolü var
- ✅ PID tabanlı yumuşak geçiş
- ✅ Timeout koruması
- ✅ Fork gerekmez

**Dezavantajları:**
- ⚠️ X/Y local offset yok (heading ile simüle edilmeli)
- ⚠️ `AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED` gerekli

### Seçenek 2: ArduPilot Fork Etme

**Değişiklikler:**
```cpp
// handle_set_position_target_local_ned'i genişlet
void handle_set_position_target_local_ned(...) {
    // Mevcut: sadece altitude
    // Yeni: x/y offset'i heading olarak kullan
    
    if (!ignore_x && !ignore_y) {
        float target_heading = atan2f(packet.y, packet.x);
        float target_distance = sqrtf(packet.x*packet.x + packet.y*packet.y);
        // ... heading komutu gönder
    }
    // ... altitude kodu
}
```

**Avantajları:**
- ✅ Tam local NED kontrolü

**Dezavantajları:**
- ❌ Fork gerekir (güncelleme zor)
- ❌ Test zorunluluğu
- ❌ Backward compatibility sorunları

### Seçenek 3: Hibrit Yaklaşım

**Strateji:**
1. Slew commands (heading/speed/altitude)
2. Velocity control (düzeltmeler için)
3. Global position (büyük hareketler için)

**Avantajları:**
- ✅ En esnek
- ✅ Fallback mekanizması

---

## 5. RİSK ANALİZİ

### 5.1 Teknik Riskler

| Risk | Olasılık | Etki | Mitigasyon |
|------|----------|------|------------|
| Slew commands kapalı | Orta | Yüksek | Param kontrolü + fallback |
| Timeout issues | Düşük | Orta | Watchdog + state tracking |
| PID instability | Düşük | Yüksek | Conservative parametreler |
| Comm loss | Orta | Yüksek | RTL trigger |
| ArduPilot version mismatch | Orta | Orta | Version check |

### 5.2 Güvenlik Riskleri

### 5.2.1 Boundary Violation

**Senaryo:** Heading komutu boundary dışına çıkarır

**Çözüm:**
```cpp
bool SafetyGate::validateHeadingCommand(float heading_deg) {
    auto projected = projectPosition(heading_deg, 1000m);
    return zone_checker_->isInsideBoundary(projected, boundary_);
}
```

### 5.2.2 HSS Violation

**Senaryo:** Yeni heading HSS içinden geçer

**Çözüm:**
```cpp
bool SafetyGate::isHeadingSafe(float heading_deg, float distance_m) {
    auto start = world_model_->getOwnState().position;
    auto end = projectPosition(start, heading_deg, distance_m);
    return !zone_checker_->segmentIntersectsAnyHSS(start, end, hss_zones_, buffer_);
}
```

### 5.3 Operasyonel Riskler

| Risk | Olasılık | Etki | Mitigasyon |
|------|----------|------|------------|
| Pilot override | Düşük | Orta | RC override detection |
| Mode confusion | Düşük | Orta | Clear mode indicators |
| Parameter drift | Orta | Düşük | Periodic validation |

---

## 6. LİTERATÜR TARAMASI

### 6.1 Benzer Sistemler

#### 6.1.1 PX4 Offboard Control

PX4, uçuş kontrolünde `SET_POSITION_TARGET_LOCAL_NED` komutunu kullanıyor:

**Özellikler:**
- Full 6-DOF control (x, y, z, vx, vy, vz, yaw, yaw_rate)
- Multirotor için optimize
- Fixed-wing için sınırlı

**Fark:** PX4 fixed-wing desteği ArduPilot kadar gelişmiş değil

#### 6.1.2 ArduPilot Copter Slew Commands

ArduPilot Copter zaten slew commands kullanıyor:

**Özellikler:**
- `MAV_CMD_GUIDED_CHANGE_SPEED`
- `MAV_CMD_GUIDED_CHANGE_HEADING`
- `MAV_CMD_GUIDED_CHANGE_POSITION`

**Fark:** Copter'da position control var, Plane'de yok

#### 6.1.3 QGroundControl Follow Me

QGC "Follow Me" mode:

**Strateji:**
- GPS koordinatını al
- DO_REPOSITION gönder
- Rate limit (1Hz)

**Sınırlama:** Sadece global position

### 6.2 Akademik Çalışmalar

#### 6.2.1 Vision-Based Navigation

**"Vision-Based UAV Navigation" (2021)**

**Bulgular:**
- Heading control kritik
- Rate-based control daha stabil
- 10-20Hz update rate yeterli

#### 6.2.2 Proportional Navigation

**"Proportional Navigation for UAV Intercept" (2020)**

**Bulgular:**
- PN gain 3-5 arası ideal
- Look-ahead angle önemli
- Acceleration limit şart

---

## 7. ÖNERİLEN ÇÖZÜM

### 7.1 Mimari

```
┌─────────────────────────────────────────────────────────────┐
│                    LOCAL CONTROL ARCHITECTURE                │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Vision/Tracking Layer                                      │
│  ├── Target detection                                       │
│  ├── Bearing calculation                                    │
│  └── Distance estimation                                    │
│         │                                                   │
│         ▼                                                   │
│  Control Layer (YENİ)                                      │
│  ├── HeadingController  → MAV_CMD_GUIDED_CHANGE_HEADING    │
│  ├── SpeedController    → MAV_CMD_GUIDED_CHANGE_SPEED      │
│  ├── AltitudeController → MAV_CMD_GUIDED_CHANGE_ALTITUDE   │
│  └── SlewManager       → Rate limiting & timeout           │
│         │                                                   │
│         ▼                                                   │
│  Safety Layer (MEVCUT)                                     │
│  ├── SafetyGate.validateHeading()                          │
│  ├── SafetyGate.isHeadingSafe()                            │
│  └── SafetyMonitor.watchdog                                │
│         │                                                   │
│         ▼                                                   │
│  ArduPilot GUIDED Mode                                      │
│  ├── GuidedState.target_heading                            │
│  ├── GuidedState.target_airspeed_cm                       │
│  └── GuidedState.target_altitude                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Yeni Sınıflar

#### 7.2.1 HeadingController

```cpp
class HeadingController {
public:
    struct Config {
        float max_bank_angle_deg = 30.0f;
        float accel_limit = 0.5f;       // rad/s²
        float update_rate_hz = 10.0f;
        bool use_cog = false;           // true = COG, false = heading
    };
    
    struct State {
        float current_heading = 0.0f;
        float target_heading = 0.0f;
        float heading_error = 0.0f;
        uint32_t last_command_ms = 0;
        bool is_active = false;
    };
    
    bool setTargetHeading(float heading_deg);
    bool update(const UAVState& own_state);
    void reset();
    
private:
    Config config_;
    State state_;
    std::shared_ptr<FlightCommander> commander_;
    std::shared_ptr<SafetyGate> safety_gate_;
};
```

#### 7.2.2 SpeedController

```cpp
class SpeedController {
public:
    struct Config {
        float min_airspeed_ms = 12.0f;
        float max_airspeed_ms = 35.0f;
        float max_accel_ms2 = 5.0f;
        float update_rate_hz = 5.0f;
    };
    
    bool setTargetSpeed(float airspeed_ms);
    bool update(const UAVState& own_state);
    float getCurrentTarget() const;
    
private:
    Config config_;
    float target_airspeed_cm_ = -1;
    uint32_t last_command_ms_ = 0;
};
```

#### 7.2.3 AltitudeController

```cpp
class AltitudeController {
public:
    struct Config {
        float min_alt_m = 30.0f;
        float max_alt_m = 400.0f;
        float max_climb_rate_ms = 5.0f;
        float max_descent_rate_ms = 3.0f;
        float update_rate_hz = 5.0f;
    };
    
    bool setTargetAltitude(float altitude_m, MAV_FRAME frame);
    bool setClimbRate(float climb_rate_ms);
    bool update(const UAVState& own_state);
    
private:
    Config config_;
    Location target_location_;
    float target_climb_rate_ = 0.0f;
    uint32_t last_command_ms_ = 0;
};
```

### 7.3 FlightCommander Genişletme

```cpp
// flight_commander.hpp'a ekle
class FlightCommander {
public:
    // Mevcut metodlar...
    
    // YENİ: Guided Slew Commands
    bool changeHeading(float heading_deg, HeadingType type, float accel_limit = 0.5f);
    bool changeAirspeed(float airspeed_ms, float acceleration = 0.0f);
    bool changeAltitude(float altitude_m, float climb_rate = 0.0f, MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT);
    
    // YENİ: Query
    bool isSlewEnabled() const;
    float getGuidedTimeout() const;
    
private:
    // Slew command tracking
    struct {
        uint32_t heading_ms = 0;
        uint32_t airspeed_ms = 0;
        uint32_t altitude_ms = 0;
    } slew_command_times_;
};
```

---

## 8. IMPLEMENTASYON PLANI

### Faz 1: Temel Altyapı (1-2 hafta)

**Görevler:**
1. ✅ ArduPilot analizi tamamlandı
2. ⏳ FlightCommander'a slew command metodları ekle
3. ⏳ Parametre kontrolü (`AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`)
4. ⏳ Unit testler

**Çıktılar:**
- `FlightCommander::changeHeading()`
- `FlightCommander::changeAirspeed()`
- `FlightCommander::changeAltitude()`

### Faz 2: Controller Sınıfları (2-3 hafta)

**Görevler:**
1. ⏳ HeadingController implementasyonu
2. ⏳ SpeedController implementasyonu
3. ⏳ AltitudeController implementasyonu
4. ⏳ SafetyGate entegrasyonu

**Çıktılar:**
- `heading_controller.hpp/cpp`
- `speed_controller.hpp/cpp`
- `altitude_controller.hpp/cpp`

### Faz 3: FollowMode Entegrasyonu (1-2 hafta)

**Görevler:**
1. � FollowMode'da PN + heading control hibriti
2. � Pursuit phase'de fine control
3. � Rate limiting

**Çıktılar:**
- Güncellenmiş `FollowMode`
- Daha smooth takip

### Faz 4: Test ve Doğrulama (2-3 hafta)

**Görevler:**
1. ⏳ SITL testleri
2. ⏳ HIL testleri
3. ⏳ Field testleri
4. ⏳ Performance tuning

**Çıktılar:**
- Test raporları
- Tuned parametreler

---

## 9. TEST STRATEJİSİ

### 9.1 Unit Tests

```cpp
// HeadingController test
TEST(HeadingController, SetTarget) {
    auto controller = createHeadingController();
    EXPECT_TRUE(controller.setTargetHeading(90.0f));
    EXPECT_EQ(controller.getTarget(), 90.0f);
}

TEST(HeadingController, SafetyValidation) {
    auto controller = createHeadingController();
    // HSS içeren heading
    EXPECT_FALSE(controller.setTargetHeading(unsafe_heading));
}
```

### 9.2 SITL Test Senaryoları

**Senaryo 1: Basic Heading Control**
```
1. SITL başlat
2. GUIDED mode'a geç
3. Heading command gönder (45°)
4. DOĞRULA: Uçak 45°'ye döner
5. DOĞRULA: Timeout sonrası normal navigasyona döner
```

**Senaryo 2: Speed Change**
```
1. Cruise speed'de uç (18 m/s)
2. Speed command gönder (25 m/s)
3. DOĞRULA: Hız yavaşça artar
4. DOĞRULA: Acceleration limit'i aşmaz
```

**Senaryo 3: Altitude Slew**
```
1. 100m'de uç
2. Altitude command gönder (150m, 3 m/s climb)
3. DOĞRULA: 3 m/s ile tırmanır
4. DOĞRULA: 150m'de stabil olur
```

**Senaryo 4: Safety Violation**
```
1. Boundary'e yakın uç
2. Unsafe heading command gönder
3. DOĞRULA: Komut reddedilir
4. DOĞRULA: Safety warning loglanır
```

### 9.3 Field Test Checklist

- [ ] RC override çalışıyor
- [ ] Timeout mekanizması çalışıyor
- [ ] Boundary kontrolü çalışıyor
- [ ] HSS kontrolü çalışıyor
- [ ] Mode transition düzgün
- [ ] Battery drain kabul edilebilir
- [ ] Airspeed limitleri korunuyor

---

## 10. GERİYE DÖNÜŞ

### 10.1 Rollback Planı

Eğer slew commands çalışmazsa:

1. **Fallback mekanizması:**
```cpp
if (!slew_enabled || slew_failed) {
    // Global position fallback
    commander_->flyToPosition(global_target);
}
```

2. **Feature flag:**
```yaml
# config/default.yaml
guided:
  use_slew_commands: true  # false = fallback to DO_REPOSITION
```

### 10.2 Monitoring

**Metricler:**
- Slew command success rate
- Timeout frequency
- Heading error (rms)
- Speed tracking error
- Altitude tracking error

**Loglar:**
```
[INFO] HeadingController: Target=90.0° Current=85.2° Error=4.8°
[WARN] HeadingController: Command timeout, resetting
[ERROR] SafetyGate: Heading 270° violates HSS zone #1
```

---

## 11. PARAMETRELER

### 11.1 ArduPilot Parametreleri

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `GUIDED_TIMEOUT` | 3.0s | Slew command timeout |
| `GUIDED_HEADING_P` | 0.4 | Heading PID P |
| `GUIDED_HEADING_I` | 0.0 | Heading PID I |
| `GUIDED_HEADING_D` | 0.0 | Heading PID D |

### 11.2 Yeni Config Parametreleri

```yaml
# config/default.yaml'a ekle
guided_slew:
  enabled: true
  heading:
    max_bank_angle_deg: 30.0
    accel_limit: 0.5
    update_rate_hz: 10.0
    use_cog: false  # true = course over ground
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

## 12. ZAYIF NOKTALAR

### 12.1 Bilinen Kısıtlamalar

1. **X/Y Local Offset Yok**
   - Neden: ArduPilot fixed-wing tasarım kararı
   - Çözüm: Heading + speed kombinasyonu

2. **Rate Limit**
   - Neden: Komm gecikmesi + processing
   - Çözüm: Öngörücü algoritmalar

3. **Delay**
   - Neden: Guided → TECS → Servo pipeline
   - Çözüm: Lead compensation

### 12.2 Gelecek İyileştirmeler

1. **Predictive Control:** Model predictive control
2. **Vision Fusion:** Birden fazla vision source
3. **Learning:** Adaptive parametreler
4. **Multi-UAV:** Swarm control

---

## 13. KAYNAKLAR

### 13.1 ArduPilot Dokümantasyonu

- [Guided Mode](https://ardupilot.org/plane/docs/guided-mode.html)
- [MAVLink Commands](https://mavlink.io/en/messages/common.html#MAV_CMD)

### 13.2 Kod Referansları

- `ArduPlane/mode_guided.cpp` - Guided mode implementasyonu
- `ArduPlane/GCS_MAVLink_Plane.cpp:632-729` - Slew command handlers
- `ArduPlane/Plane.h:574-606` - GuidedState struct

### 13.3 Dış Kaynaklar

- PX4 Offboard Control: https://docs.px4.io/master/en/ros/offboard_control.html
- QGroundControl Follow Mode: https://github.com/mavlink/qgroundcontrol

---

## 14. KARAR MATRİSİ

| Kriter | Seçenek 1 | Seçenek 2 | Seçenek 3 |
|--------|-----------|-----------|-----------|
| **Geliştirme Süresi** | 2-4 hafta | 4-8 hafta | 3-6 hafta |
| **Risk** | Düşük | Yüksek | Orta |
| **Bakım Maliyeti** | Düşük | Yüksek | Orta |
| **Esneklik** | Orta | Yüksek | Yüksek |
| **ArduPilot Uyumu** | ✅ Native | ❌ Fork | ✅ Native |
| **Geriye Dönüş** | Kolay | Zor | Orta |

**ÖNERİ:** Seçenek 1 (Guided Slew Commands)

---

## 15. SONRAKİ ADIMLAR

1. ✅ Analiz ve planlama tamamlandı
2. ⏳ Ekibin onayı
3. ⏳ Faz 1 başlatma
4. ⏳ Weekly progress review

---

*Döküman v1.0 - 2026-04-14*
