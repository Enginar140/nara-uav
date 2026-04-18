# LOCAL CONTROL IMPLEMENTASYON PLANI - GÜVENLİ ADIM ADIMLARI

> **Amaç:** ArduPilot Guided Slew Commands'ı cpp_uav sistemine güvenli bir şekilde entegre etmek

---

## 📋 İÇİNDEKİLER

1. [Implementasyon Stratejisi](#1-implementasyon-stratejisi)
2. [Faz 1: Temel Altyapı](#2-faz-1-temel-altyapı)
3. [Faz 2: FlightCommander Genişletme](#3-faz-2-flightcommander-genişletme)
4. [Faz 3: Controller Sınıfları](#4-faz-3-controller-sınıfları)
5. [Faz 4: FollowMode Güncelleme](#5-faz-4-followmode-güncelleme)
6. [Faz 5: Test ve Doğrulama](#6-faz-5-test-ve-doğrulama)
7. [Risk Yönetimi](#7-risk-yönetimi)

---

## 1. IMPLEMENTASYON STRATEJİSİ

### 1.1 Prensipler

1. **Backward Compatibility:** Mevcut kodu bozma
2. **Feature Flags:** Yeni özellikler kapatılabilir olmalı
3. **Safety First:** Her adımda safety check
4. **Test Driven:** Unit test + SITL test
5. **Incremental:** Küçük adımlar, commit başına

### 1.2 Mimari Kararları

```
┌─────────────────────────────────────────────────────────────┐
│                  YENİ MİMARİ                                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. FlightCommander'a 3 yeni metod                          │
│     ├── changeHeading()                                    │
│     ├── changeAirspeed()                                   │
│     └── changeAltitude()                                   │
│                                                             │
│  2. Yeni config parametreleri                                 │
│     ├── guided_slew.enabled                                │
│     ├── guided_slew.heading.*                               │
│     ├── guided_slew.speed.*                                 │
│     └── guided_slew.altitude.*                              │
│                                                             │
│  3. FollowMode'da pursuit phase güncelleme                  │
│     - Slew commands kullan                                │
│     - PN + heading control hibriti                         │
│                                                             │
│  4. SafetyGate genişletme                                   │
│     - validateHeading()                                    │
│     - isHeadingSafe()                                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. FAZ 1: TEMEL ALTYAPI

### 2.1 Task Listesi

- [ ] Config parametrelerini ekle
- [ ] MAVLink message tiplerini tanımla
- [ ] Unit test framework kur
- [ ] Build sistemi güncelle

### 2.2 Config Dosyası

**Dosya:** [`config/default.yaml`](cpp_uav/config/default.yaml)

```yaml
# Mevcut config'in sonuna ekle:

guided_slew:
  enabled: true
  
  heading:
    max_bank_angle_deg: 30.0
    accel_limit: 0.5              # rad/s²
    update_rate_hz: 10.0
    use_cog: false                 # true=COG, false=HEADING
  
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
    
  timeout_s: 3.0                   # Komut timeout (ArduPilot GUIDED_TIMEOUT ile aynı olmalı)
```

### 2.3 MAVLink Constants

**Dosya:** `external/mavlink/mavlink_types.h` veya yeni bir dosya

```cpp
// MAVLink message constants (zaten tanımlı, referans için)
#define MAV_CMD_GUIDED_CHANGE_HEADING  40002
#define MAV_CMD_GUIDED_CHANGE_SPEED    40000
#define MAV_CMD_GUIDED_CHANGE_ALTITUDE 40001

// Heading types
enum class HeadingType : uint8_t {
    NONE = 0,
    COG = 1,        // Course Over Ground
    HEADING = 2       // Vehicle Heading
};

// Speed types
enum class SpeedType : uint8_t {
    UNSET = 0,
    AIRSPEED = 0,
    GROUNDSPEED = 1
};
```

---

## 3. FAZ 2: FLIGHTCOMMANDER GENİŞLETME

### 3.1 Header Dosyası

**Dosya:** [`include/uav/comm/flight_commander.hpp`](cpp_uav/include/uav/comm/flight_commander.hpp)

```cpp
// Mevcut enum'ların altına ekle:
enum class HeadingType : uint8_t {
    NONE = 0,
    COG = 1,
    HEADING = 2
};

class FlightCommander {
public:
    // ... mevcut metodlar ...
    
    // YENİ: Guided Slew Commands
    [[nodiscard]] bool changeHeading(
        float heading_deg,
        HeadingType type = HeadingType::HEADING,
        float accel_limit_rad_s2 = 0.5f
    );
    
    [[nodiscard]] bool changeAirspeed(
        float airspeed_ms,
        float acceleration_ms2 = 0.0f  // 0 = maksimum
    );
    
    [[nodiscard]] bool changeAltitude(
        float altitude_m,
        float climb_rate_ms = 0.0f,     // 0 = maksimum
        uint8_t mav_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    );
    
    // YENİ: Query metodları
    [[nodiscard]] bool isSlewEnabled() const;
    [[nodiscard]] float getGuidedTimeout() const;
    
private:
    // Slew command tracking
    struct {
        uint32_t heading_ms = 0;
        uint32_t airspeed_ms = 0;
        uint32_t altitude_ms = 0;
    } slew_times_;
};
```

### 3.2 Implementation Dosyası

**Dosya:** [`src/comm/flight_commander.cpp`](cpp_uav/src/comm/flight_commander.cpp)

```cpp
// Mevcut include'ların altına ekle:
#include "mavlink_types.h"  // veya tanımlanan constants

namespace uav::comm {

bool FlightCommander::changeHeading(
    float heading_deg,
    HeadingType type,
    float accel_limit_rad_s2
) {
    std::lock_guard lock(cmd_mutex_);
    
    // 1. Validation
    if (heading_deg < 0.0f || heading_deg >= 360.0f) {
        LOG_WARN("Invalid heading: {:.1f}° (range: 0-359)", heading_deg);
        return false;
    }
    
    if (accel_limit_rad_s2 < 0.05f) {
        accel_limit_rad_s2 = 0.05f;  // Minimum
    }
    
    // 2. MAVLink mesajı hazırla
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_INT,
        MAV_CMD_GUIDED_CHANGE_HEADING,
        0, 0,                                   // confirmation, reserved
        static_cast<float>(type),             // param1: Heading Type
        heading_deg,                              // param2: Heading (derece)
        accel_limit_rad_s2,                      // param3: Accel limit (rad/s²)
        0, 0, 0,                                  // param4-6: unused
        0, 0                                      // x, z: unused
    );
    
    // 3. Track timing
    slew_times_.heading_ms = nowMillis();
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    
    LOG_DEBUG("Heading slew: {:.1f}° type={} accel={:.2f} rad/s²",
              heading_deg, static_cast<int>(type), accel_limit_rad_s2);
    
    return sendMAVLinkMessage(msg);
}

bool FlightCommander::changeAirspeed(
    float airspeed_ms,
    float acceleration_ms2
) {
    std::lock_guard lock(cmd_mutex_);
    
    // 1. Validation
    const float MIN_AIRSPEED = 12.0f;
    const float MAX_AIRSPEED = 35.0f;
    
    if (airspeed_ms < MIN_AIRSPEED || airspeed_ms > MAX_AIRSPEED) {
        LOG_WARN("Airspeed out of range: {:.1f} m/s (range: {:.1f}-{:.1f})",
                 airspeed_ms, MIN_AIRSPEED, MAX_AIRSPEED);
        return false;
    }
    
    // 2. MAVLink mesajı
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_INT,
        MAV_CMD_GUIDED_CHANGE_SPEED,
        0, 0,
        SPEED_TYPE_AIRSPEED,                // param1
        airspeed_ms,                         // param2
        acceleration_ms2,                    // param3
        0, 0, 0, 0, 0
    );
    
    slew_times_.airspeed_ms = nowMillis();
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    
    LOG_DEBUG("Speed slew: {:.1f} m/s accel={:.1f} m/s²",
              airspeed_ms, acceleration_ms2);
    
    return sendMAVLinkMessage(msg);
}

bool FlightCommander::changeAltitude(
    float altitude_m,
    float climb_rate_ms,
    uint8_t mav_frame
) {
    std::lock_guard lock(cmd_mutex_);
    
    // 1. Validation
    const float MIN_ALT = 30.0f;
    const float MAX_ALT = 400.0f;
    
    if (altitude_m < MIN_ALT || altitude_m > MAX_ALT) {
        LOG_WARN("Altitude out of range: {:.1f} m (range: {:.1f}-{:.1f})",
                 altitude_m, MIN_ALT, MAX_ALT);
        return false;
    }
    
    if (climb_rate_ms < 0.0f) {
        climb_rate_ms = 0.0f;  // Maksimum
    }
    
    // 2. MAVLink mesajı
    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        mav_frame,                             // Frame önemli!
        MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0, 0, 0,
        altitude_m,                             // z: Altitude (m)
        climb_rate_ms,                         // param3: Climb rate (m/s, 0=maks)
        0, 0
    );
    
    slew_times_.altitude_ms = nowMillis();
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    
    LOG_DEBUG("Altitude slew: {:.1f} m climb={:.1f} m/s",
              altitude_m, climb_rate_ms);
    
    return sendMAVLinkMessage(msg);
}

bool FlightCommander::isSlewEnabled() const {
    // Config'dan oku veya varsayılan true
    // TODO: ConfigManager'dan oku
    return true;
}

float FlightCommander::getGuidedTimeout() const {
    // ArduPilot GUIDED_TIMEOUT parametresi (varsayılan 3.0s)
    // TODO: ParamReader'dan oku
    return 3.0f;
}

} // namespace uav::comm
```

---

## 4. FAZ 3: CONTROLLER SINIFLARI

### 4.1 Heading Controller

**Dosyalar:**
- `include/uav/control/heading_controller.hpp`
- `src/uav/control/heading_controller.cpp`

**Header:**

```cpp
#pragma once

#include "uav/core/types.hpp"
#include "uav/comm/flight_commander.hpp"
#include <memory>

namespace uav::control {

enum class HeadingType : uint8_t {
    NONE = 0,
    COG = 1,
    HEADING = 2
};

class HeadingController {
public:
    struct Config {
        float max_bank_angle_deg = 30.0f;
        float accel_limit_rad_s2 = 0.5f;
        float update_rate_hz = 10.0f;
        bool use_cog = false;
    };
    
    struct State {
        float current_heading = 0.0f;
        float target_heading = 0.0f;
        float heading_error_deg = 0.0f;
        uint32_t last_command_ms = 0;
        bool is_active = false;
    };
    
    HeadingController(std::shared_ptr<FlightCommander> commander,
                        const Config& config);
    
    // Control interface
    bool setTargetHeading(float heading_deg);
    bool update(const core::UAVState& own_state);
    void reset();
    
    // State query
    const State& getState() const { return state_; }
    bool isActive() const { return state_.is_active; }
    
private:
    Config config_;
    State state_;
    std::shared_ptr<FlightCommander> commander_;
    
    uint32_t last_update_ms_ = 0;
    uint32_t update_interval_ms_;
};

} // namespace uav::control
```

**Implementation:**

```cpp
#include "uav/control/heading_controller.hpp"
#include "uav/infra/log_manager.hpp"
#include <chrono>

namespace uav::control {

HeadingController::HeadingController(
    std::shared_ptr<FlightCommander> commander,
    const Config& config
) : commander_(std::move(commander))
    , config_(config)
    , update_interval_ms_(static_cast<uint32_t>(1000.0f / config.update_rate_hz))
{
    reset();
}

bool HeadingController::setTargetHeading(float heading_deg) {
    // 1. Validation
    if (heading_deg < 0.0f || heading_deg >= 360.0f) {
        LOG_WARN("Invalid heading: {:.1f}°", heading_deg);
        return false;
    }
    
    // 2. State güncelle
    state_.target_heading = heading_deg;
    state_.is_active = true;
    state_.last_command_ms = nowMillis();
    
    // 3. Komut gönder
    HeadingType type = config_.use_cog ? HeadingType::COG : HeadingType::HEADING;
    return commander_->changeHeading(
        heading_deg,
        type,
        config_.accel_limit_rad_s2
    );
}

bool HeadingController::update(const core::UAVState& own_state) {
    uint32_t now = nowMillis();
    
    // 1. Update current heading
    state_.current_heading = own_state.heading;
    state_.heading_error_deg = state_.target_heading - state_.current_heading;
    
    // Wrap to -180 to +180
    while (state_.heading_error_deg > 180.0f) state_.heading_error_deg -= 360.0f;
    while (state_.heading_error_deg < -180.0f) state_.heading_error_deg += 360.0f;
    
    // 2. Check timeout
    float timeout_s = commander_->getGuidedTimeout();
    if (now - state_.last_command_ms > timeout_s * 1000) {
        state_.is_active = false;
        return false;
    }
    
    // 3. Rate limiting
    if (now - last_update_ms_ < update_interval_ms_) {
        return true;  // Too soon
    }
    last_update_ms_ = now;
    
    // 4. Re-send if active
    if (state_.is_active && 
        fabs(state_.heading_error_deg) > 1.0f) {  // 1° tolerance
        HeadingType type = config_.use_cog ? HeadingType::COG : HeadingType::HEADING;
        commander_->changeHeading(
            state_.target_heading,
            type,
            config_.accel_limit_rad_s2
        );
    }
    
    return true;
}

void HeadingController::reset() {
    state_ = State{};
    last_update_ms_ = nowMillis();
}

} // namespace uav::control
```

### 4.2 Speed Controller

**Dosyalar:**
- `include/uav/control/speed_controller.hpp`
- `src/uav/control/speed_controller.cpp`

**Header:**

```cpp
#pragma once

#include "uav/core/types.hpp"
#include "uav/comm/flight_commander.hpp"
#include <memory>

namespace uav::control {

class SpeedController {
public:
    struct Config {
        float min_ms = 12.0f;
        float max_ms = 35.0f;
        float max_accel_ms2 = 5.0f;
        float update_rate_hz = 5.0f;
    };
    
    SpeedController(std::shared_ptr<FlightCommander> commander,
                      const Config& config);
    
    bool setTargetSpeed(float airspeed_ms);
    bool update(const core::UAVState& own_state);
    void reset();
    
    float getCurrentTarget() const { return target_airspeed_ms_; }
    
private:
    Config config_;
    float target_airspeed_ms_ = -1.0f;
    uint32_t last_command_ms_ = 0;
    uint32_t update_interval_ms_;
    std::shared_ptr<FlightCommander> commander_;
};

} // namespace uav::control
```

### 4.3 Altitude Controller

**Dosyalar:**
- `include/uav/control/altitude_controller.hpp`
- `src/uav/control/altitude_controller.cpp`

---

## 5. FAZ 4: FOLLOWMODE GÜNCELLEME

### 5.1 Mevcut FollowMode

**Dosya:** [`src/mission/follow_mode.cpp`](cpp_uav/src/mission/follow_mode.cpp)

**Mevcut pursuit phase:**

```cpp
// Mevcut (yaklaşık 200m'de Proportional Navigation)
void FollowMode::updatePursuit(const UAVState& own_state, const UAVState& target) {
    // 1. LOS bearing hesapla
    float los_bearing = geo::bearing(own_state.position, target.position);
    
    // 2. PN lead angle
    float t_go = calculateTimeToGo(own_state, target);
    float pn_offset = config_.pn_gain * los_rate * t_go;
    float intercept_bearing = los_bearing + pn_offset;
    
    // 3. Global waypoint gönder (MEVCUT YÖNTEM)
    auto intercept_pos = geo::destinationPoint(
        target.position,
        degrees(intercept_bearing),
        t_go * target.airspeed
    );
    nav_engine_->planRoute(intercept_pos, config_.approach_speed_ms);
}
```

**Yeni pursuit phase (Slew Commands):**

```cpp
// YENİ (heading + speed control)
void FollowMode::updatePursuit(const UAVState& own_state, const UAVState& target) {
    // 1. LOS bearing hesapla
    float los_bearing = geo::bearing(own_state.position, target.position);
    
    // 2. Calculate LOS rate
    float dt = 0.1f;  // 10Hz
    float los_rate = (los_bearing - last_los_bearing_) / dt;
    last_los_bearing_ = los_bearing;
    
    // 3. PN lead angle
    float t_go = calculateTimeToGo(own_state, target);
    float pn_offset = config_.pn_gain * los_rate * t_go;
    pn_offset = std::clamp(pn_offset, 
                       -config_.pn_max_lead_deg, 
                       config_.pn_max_lead_deg);
    float intercept_bearing = los_bearing + pn_offset;
    
    // 4. HEADING SLEW COMMAND (global waypoint yerine)
    if (nowMillis() - last_heading_update_ > 100ms) {  // 10Hz
        commander_->changeHeading(
            intercept_bearing,
            HeadingType::HEADING,
            0.5f
        );
        last_heading_update_ = nowMillis();
    }
    
    // 5. SPEED SLEW COMMAND
    if (nowMillis() - last_speed_update_ > 200ms) {  // 5Hz
        commander_->changeAirspeed(config_.pursuit_speed_ms, 3.0f);
        last_speed_update_ = nowMillis();
    }
    
    // 6. Altitude control (hedef irtifasında kal)
    float alt_error = target.position.altitude - own_state.position.altitude;
    if (fabs(alt_error) > 5.0f) {
        commander_->changeAltitude(target.position.altitude, 2.0f);
    }
}
```

---

## 6. FAZ 5: TEST VE DOĞRULAMA

### 6.1 Unit Tests

**Test Framework:** Catch2 veya Google Test

**Test Cases:**

```cpp
// test_flight_commander_slew.cpp
TEST_CASE("FlightCommander_HeadingSlew", "[flight_commander]") {
    auto conn = std::make_shared<MockConnectionManager>();
    FlightCommander fc(conn, 1, 1);
    
    // Test 1: Valid heading
    EXPECT_TRUE(fc.changeHeading(90.0f, HeadingType::HEADING, 0.5f));
    
    // Test 2: Invalid heading
    EXPECT_FALSE(fc.changeHeading(-10.0f, HeadingType::HEADING, 0.5f));
    EXPECT_FALSE(fc.changeHeading(370.0f, HeadingType::HEADING, 0.5f));
    
    // Test 3: Zero accel limit
    EXPECT_TRUE(fc.changeHeading(90.0f, HeadingType::HEADING, 0.0f));
}

TEST_CASE("FlightCommander_SpeedSlew", "[flight_commander]") {
    auto conn = std::make_shared<MockConnectionManager>();
    FlightCommander fc(conn, 1, 1);
    
    // Test 1: Valid speed
    EXPECT_TRUE(fc.changeAirspeed(25.0f, 3.0f));
    
    // Test 2: Out of range
    EXPECT_FALSE(fc.changeAirspeed(10.0f, 3.0f));  // Too low
    EXPECT_FALSE(fc.changeAirspeed(40.0f, 3.0f));  // Too high
}

TEST_CASE("FlightCommander_AltitudeSlew", "[flight_commander]") {
    auto conn = std::make_shared<MockConnectionManager>();
    FlightCommander fc(conn, 1, 1);
    
    // Test 1: Valid altitude
    EXPECT_TRUE(fc.changeAltitude(150.0f, 2.0f));
    
    // Test 2: Out of range
    EXPECT_FALSE(fc.changeAltitude(20.0f, 2.0f));   // Too low
    EXPECT_FALSE(fc.changeAltitude(500.0f, 2.0f));  // Too high
}
```

### 6.2 SITL Test Senaryoları

**Test Dosyası:** `tests/test_slew_commands.cpp`

```cpp
// Test 1: Basic heading control
TEST_CASE("SITL_HeadingControl", "[slew]") {
    // Setup
    auto& plane = sitl_setup_plane();
    auto fc = createFlightCommander();
    
    // Test
    EXPECT_TRUE(fc.changeHeading(90.0f, HEADING, 0.5f));
    
    // Verify
    hal.delay(2000);  // 2 saniye bekle
    float heading = plane.get_heading();
    EXPECT_NEAR(heading, 90.0f, 5.0f);  // ±5° tolerance
}

// Test 2: Speed transition
TEST_CASE("SITL_SpeedTransition", "[slew]") {
    auto& plane = sitl_setup_plane();
    auto fc = createFlightCommander();
    
    // Cruise → Pursuit
    EXPECT_TRUE(fc.changeAirspeed(18.0f, 2.0f));  // Cruise
    hal.delay(3000);  // 3 saniye bekle
    
    EXPECT_TRUE(fc.changeAirspeed(25.0f, 3.0f));  // Pursuit
    hal.delay(4000);  // 4 saniye bekle
    
    float airspeed = plane.get_airspeed();
    EXPECT_NEAR(airspeed, 25.0f, 1.0f);  // ±1 m/s tolerance
}

// Test 3: Timeout
TEST_CASE("SITL_Timeout", "[slew]") {
    auto& plane = sitl_setup_plane();
    auto fc = createFlightCommander();
    
    // Komut gönder
    EXPECT_TRUE(fc.changeHeading(90.0f, HEADING, 0.5f));
    
    // Timeout bekle (3 saniye)
    hal.delay(3500);
    
    // Normal navigasyona döndü mü kontrol et
    float roll = plane.get_nav_roll();
    EXPECT_NEAR(roll, 0.0f, 5.0f);  // ~0° = level flight
}
```

### 6.3 Field Test Checklist

- [ ] RC override çalışıyor
- [ ] Timeout mekanizması doğru
- [ ] Boundary kontrolü çalışıyor
- [ ] HSS kontrolü çalışıyor
- [ ] Mode transition düzgün
- [ ] Battery drain kabul edilebilir
- - [ ] Airspeed limitleri korunuyor

---

## 7. RISK YÖNETİMİ

### 7.1 Riskler ve Mitigasyon

| Risk | Olasılık | Etki | Mitigasyon |
|------|----------|------|------------|
| **ArduPilot parametresi yanlış** | Orta | Yüksek | Param validation + test |
| **Slew commands kapalı** | Düşük | Yüksek | Feature flag + fallback |
| **Timeout issues** | Düşük | Orta | Watchdog + retry logic |
| **Boundary violation** | Orta | Kritik | SafetyGate validation |
| **Performance hit** | Düşük | Düşük | Rate limiting |
| **Memory leak** | Düşük | Orta | Smart pointers |

### 7.2 Rollback Plan

```cpp
// Feature flag ile kontrol
if (config_->getBool("guided_slew.enabled", false)) {
    // Yöntem 1: Slew commands (yeni)
    return slewControlImplementation();
} else {
    // Yöntem 2: DO_REPOSITION (mevcut)
    return legacyImplementation();
}
```

### 7.3 Monitoring

```cpp
// Metricler
struct SlewCommandMetrics {
    uint32_t heading_commands_sent = 0;
    uint32_t heading_commands_acked = 0;
    uint32_t heading_timeouts = 0;
    float heading_error_rms = 0.0f;
    
    uint32_t speed_commands_sent = 0;
    uint32_t speed_commands_acked = 0;
    
    uint32_t altitude_commands_sent = 0;
    uint32_t altitude_commands_acked = 0;
};
```

---

## 8. IMPLEMENTASYON SIRASI

### 8.1 Sıralama

1. **Faz 1:**
   - ConfigManager'a guided_slew parametreleri ekle
   - MAVLink constants tanımla
   - CMakeLists.txt güncelle (yeni dosyalar için)

2. **Faz 2:**
   - FlightCommander genişletme
   - Unit testler yaz
   - Mock objects oluştur

3. **Faz 3:**
   - Controller sınıfları
   - SafetyGate genişletme
   - Integration testler

4. **Faz 4:**
   - FollowMode güncelleme
   - SITL testleri
   - Performance testleri

5. **Faz 5:**
   - Field testleri
   - Tuning
   - Documentation

### 8.2 Commit Stratejisi

```
commit 1: "Add guided_slew config parameters"
commit 2: "Add MAVLink constants for slew commands"
commit 3: "Extend FlightCommander with slew methods"
commit 4: "Add HeadingController class"
commit 5: "Add SpeedController class"
commit 6: "Add AltitudeController class"
commit 8: "Extend SafetyGate for heading validation"
commit 9: "Update FollowMode pursuit phase with slew"
commit 10: "Add unit tests for slew commands"
commit 11: "Add SITL test scenarios"
commit 12: "Update documentation"
```

---

## 9. SONRA KONTROL

### 9.1 Checklist

- [ ] Config parametreleri tanımlandı
- [ ] MAVLink mesajları tanımlandı
- [ ] FlightCommander genişletildi
- [ ] HeadingController implement edildi
- [ ] SpeedController implement edildi
- [ ] AltitudeController implement edildi
- [ ] SafetyGate genişletildi
- [ ] FollowMode güncellendi
- [ ] Unit testler yazıldı
- [ ] SITL testleri geçti
- [ ] Field testleri yapıldı
- [ ] Dokümantasyon güncellendi

### 9.2 Success Criteria

1. **Functional:** Tüm slew commands çalışıyor
2. **Safety:** Boundary/HSS kontrolü aktif
3. **Performance:** Overhead kabul edilebilir (<5%)
4. **Stability:** No oscillation veya drift
5. **Compatible:** Mevcut kod bozulmamış

---

*Döküman v1.0 - 2026-04-14*
