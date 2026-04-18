# TEKNOFEST UAV v10 - TAM SİSTEM ANALİZ DÖKÜMANTASYONU

> **Oluşturulma Tarihi:** 2026-04-14
> **Amaç:** Sistemin tamamının anlaşılması ve gelecekteki geliştirmelerde context kaybını önlemek için kapsamlı referans dokümanı

---

## 📋 İÇİNDEKİLER

1. [Sistem Genel Bakış](#1-sistem-genel-bakış)
2. [Mimari Yapı](#2-mimari-yapı)
3. [Modül Detaylı Analizi](#3-modül-detaylı-analizi)
4. [Veri Akışı](#4-veri-akışı)
5. [Thread Mimarisi](#5-thread-mimarisi)
6. [Event-Driven Sistem](#6-event-driven-sistem)
7. [Güvenlik Sistemi](#7-güvenlik-sistemi)
8. [Path Planning Algoritması](#8-path-planning-algoritması)
9. [Mission Modları](#9-mission-modları)
10. [Konfigürasyon Yönetimi](#10-konfigürasyon-yönetimi)
11. [Build Sistemi](#11-build-sistemi)
12. [Bilinen Sorunlar ve Çözümler](#12-bilinen-sorunlar-ve-çözümler)
13. [Gelecek Geliştirme Önerileri](#13-gelecek-geliştirme-önerileri)

---

## 1. SİSTEM GENEL BAKIŞ

### 1.1 Proje Tanımı

**TEKNOFEST UAV v10**, TEKNOFEST UAV İHA Yarışması için geliştirilmiş, C++17 tabanlı, otonom uçuş ve navigasyon sistemidir.

### 1.2 Teknik Özellikler

| Özellik | Değer |
|---------|-------|
| Dil | C++17 |
| Build Sistemi | CMake 3.16+ |
| Platform | Windows/Linux cross-platform |
| Bağımlılıklar | yaml-cpp, spdlog, mavlink, nlohmann/json |
| Thread Model | Multi-threaded (main + monitor + heartbeat + event + comm) |
| Komut Model | GUIDED mode only (RC override yok) |

### 1.3 Proje Yapısı

```
cpp_uav/
├── include/uav/          # Header dosyaları
│   ├── core/             # Temel veri yapıları
│   ├── config/           # Konfigürasyon yönetimi
│   ├── infra/            # Altyapı (log, event, thread)
│   ├── world_model/      # Dünya durumu
│   ├── comm/             # MAVLink iletişim
│   ├── safety/           # Güvenlik sistemleri
│   ├── nav/              # Navigasyon
│   ├── mission/          # Mission modları
│   ├── vision/           # Nesne takibi
│   └── app/              # Ana uygulama
├── src/                  # Source dosyaları
├── config/               # YAML konfigürasyonlar
├── external/             # External dependencies
└── build/                # Build output
```

---

## 2. MİMARİ YAPİ

### 2.1 Modül Hiyerarşisi

```
┌─────────────────────────────────────────────────────────────┐
│                        Application                          │
│  (main loop, initialization, failsafe coordination)        │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│   Mission    │   │   Safety     │   │ Navigation   │
│   Director   │   │   Monitor    │   │   Engine     │
└──────────────┘   └──────────────┘   └──────────────┘
        │                   │                   │
        └───────────────────┼───────────────────┘
                            ▼
                    ┌───────────────┐
                    │ World Model   │
                    │ (State Store) │
                    └───────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  MAVLink     │   │   Event      │   │   Thread     │
│  Bridge      │   │   Bus        │   │   Pool       │
└──────────────┘   └──────────────┘   └──────────────┘
```

### 2.2 Bağımlılık Grafiği

```
app → mission → nav → safety → world_model → core
     ↓           ↓        ↓        ↓         ↓
   event_bus  flight_commander zone_checker types
               ↓
          connection_manager
```

---

## 3. MODÜL DETAYLI ANALİZİ

### 3.1 Core Module (`core/`)

**Dosyalar:**
- [`types.hpp`](include/uav/core/types.hpp) - Temel veri yapıları
- [`geo.hpp`](include/uav/core/geo.hpp) - Coğrafi hesaplamalar
- [`time_utils.hpp`](include/uav/core/time_utils.hpp) - Zaman utility'leri

**Ana Structs:**

```cpp
struct Position {
    double latitude, longitude, altitude;
    bool operator==(const Position&) const;
};

struct UAVState {
    int id;
    Position position;
    Velocity velocity;      // north, east, down
    Attitude attitude;      // roll, pitch, yaw
    double heading, airspeed, groundspeed;
    uint64_t timestamp_us;
    
    // GPS bilgileri
    uint8_t gps_fix_type;
    uint8_t satellites_visible;
    uint16_t gps_hdop;
    
    // Battery
    float battery_voltage;
    int8_t battery_remaining;
    
    bool isValid() const;
    double ageSeconds() const;
};

struct HSSZone {
    int id;
    Position center;
    double radius;
    bool active;
    double buffer_distance = 40.0;
    
    double effectiveRadius() const;  // radius + buffer
};

struct Boundary {
    vector<Position> vertices;
    double min_altitude = 30.0;
    double max_altitude = 400.0;
    
    Position getCenter() const;
    bool isValid() const;
};
```

**Enum Types:**
- `MissionMode`: ARDUPILOT, PATROL, FOLLOW, QR_DIVE
- `GoalType`: NONE, PATROL, TRACK_TARGET, DIVE_TARGET, ESCAPE
- `RiskLevel`: NONE, LOW, MEDIUM, HIGH, CRITICAL
- `BoundaryStatus`: INSIDE, NEAR_BOUNDARY, OUTSIDE

**Geo Fonksiyonları:**
- `haversineDistance()` - İki nokta arası mesafe (3D)
- `bearing()` - İki nokta arası açı [0, 360)
- `destinationPoint()` - Belirli bearing ve mesafedeki nokta
- `isPointInPolygon()` - Polygon içinde mi kontrolü
- `segmentIntersectsCircle()` - Segment-daire kesişimi
- `segmentsIntersect()` - İki segment kesişimi
- `positionToNED()` / `nedToPosition()` - Koordinat dönüşümleri

---

### 3.2 Config Module (`config/`)

**Dosyalar:**
- [`config_manager.hpp`](include/uav/config/config_manager.hpp)
- [`config_manager.cpp`](src/config/config_manager.cpp)

**Özellikler:**
- YAML tabanlı konfigürasyon
- Dot-notation ile key erişimi: `"safety.hss_buffer_m"`
- Default değer desteği
- Tip güvenli getter'ler (getString, getInt, getDouble, getBool)

---

### 3.3 Infra Module (`infra/`)

#### 3.3.1 LogManager

**Dosyalar:**
- [`log_manager.hpp`](include/uav/infra/log_manager.hpp)
- [`log_manager.cpp`](src/infra/log_manager.cpp)

**Özellikler:**
- Singleton pattern
- spdlog wrapper
- Console + rotating file sink
- Log levels: debug, info, warn, error
- Otomatik flush (3 saniyede bir)

**Macro Kullanımı:**
```cpp
LOG_DEBUG("{}", msg);
LOG_INFO("{}", msg);
LOG_WARN("{}", msg);
LOG_ERROR("{}", msg);
```

#### 3.3.2 EventBus

**Dosyalar:**
- [`event_bus.hpp`](include/uav/infra/event_bus.hpp)
- [`event_bus.cpp`](src/infra/event_bus.cpp)

**Event Types:**
```cpp
enum class EventType {
    // Safety
    SAFETY_VIOLATION, BOUNDARY_VIOLATION, HSS_VIOLATION, COLLISION_WARNING,
    
    // Data
    OPPONENT_UPDATED, HSS_ZONE_UPDATED, BOUNDARY_UPDATED, OWN_STATE_UPDATED,
    
    // Mission
    MODE_CHANGED, MODE_COMMAND_RECEIVED,
    
    // Comm
    MAVLINK_CONNECTED, MAVLINK_DISCONNECTED,
    HEARTBEAT_TIMEOUT, HEARTBEAT_RECOVERED, HEARTBEAT_RECEIVED,
    
    // Failsafe
    GPS_QUALITY_LOW, GPS_QUALITY_CRITICAL,
    BATTERY_LOW, BATTERY_CRITICAL,
    
    // System
    PARAMS_READY, SYSTEM_STARTED, SYSTEM_STOPPING
};
```

**Özellikler:**
- Pub/Sub pattern
- Thread-safe queue
- Dispatch thread (separate thread)
- Event throttling (500ms)
- `std::any` payload desteği

#### 3.3.3 ThreadPool

**Dosyalar:**
- [`thread_pool.hpp`](include/uav/infra/thread_pool.hpp)
- [`thread_pool.cpp`](src/infra/thread_pool.cpp)

**Özellikler:**
- Task queue + worker threads
- Future return değerleri
- `waitAll()` ile tüm task'leri bekleme
- 4 worker thread (configurable)

---

### 3.4 World Model Module (`world_model/`)

**Dosyalar:**
- [`world_model.hpp`](include/uav/world_model/world_model.hpp)
- [`world_model.cpp`](src/world_model/world_model.cpp)

**Sınıf:**
```cpp
class WorldModel {
public:
    // Own state
    void setOwnState(const UAVState&);
    UAVState getOwnState() const;
    
    // Opponents (rakipler)
    void updateOpponent(const UAVState&);
    void removeOpponent(int);
    optional<UAVState> getOpponent(int) const;
    vector<UAVState> getAllOpponents() const;
    optional<UAVState> getNearestOpponent(const Position&) const;
    void removeStaleOpponents(double max_age_s);
    
    // HSS zones
    void updateHSSZone(const HSSZone&);
    vector<HSSZone> getActiveHSSZones() const;
    
    // Boundary
    void setBoundary(const Boundary&);
    optional<Boundary> getBoundary() const;
    
private:
    // Tüm data shared_mutex ile korunuyor
    UAVState own_state_;
    unordered_map<int, UAVState> opponents_;
    unordered_map<int, HSSZone> hss_zones_;
    optional<Boundary> boundary_;
};
```

**Thread Safety:**
- Tüm veri `shared_mutex` ile korunuyor
- Reader-writer lock pattern
- Multiple readers, single writer

---

### 3.5 Comm Module (`comm/`)

#### 3.5.1 ConnectionManager

**Dosyalar:**
- [`connection_manager.hpp`](include/uav/comm/connection_manager.hpp)
- [`connection_manager.cpp`](src/comm/connection_manager.cpp)

**Protocols:**
- UDP Server (udpin) - Pixhawk için varsayılan
- UDP Client (udpout)
- TCP Server (tcpin) - GCS için varsayılan
- TCP Client (tcpout)
- Serial (destekleniyor ama kullanılmıyor)

**Özellikler:**
- Cross-platform socket abstraction
- Non-blocking I/O
- Auto-reconnect with exponential backoff
- Message callback pattern
- Receive thread (separate)

#### 3.5.2 FlightCommander

**Dosyalar:**
- [`flight_commander.hpp`](include/uav/comm/flight_commander.hpp)
- [`flight_commander.cpp`](src/comm/flight_commander.cpp)

**Komutlar:**

```cpp
// GUIDED GLOBAL - DO_REPOSITION (uzun menzil)
bool flyToPosition(const Position& target, float airspeed = -1.0f);

// GUIDED LOCAL - SET_POSITION_TARGET_LOCAL_NED (hassas)
bool flyToLocalPosition(float north_m, float east_m, float down_m);
bool setLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);

// Mode control
bool setMode(ArduPilotMode mode);
bool arm();
bool disarm();
```

**ArduPilotMode Enum:**
- MANUAL=0, STABILIZE=2, FBWA=5, AUTO=10, RTL=11, LOITER=12, GUIDED=15, etc.

**Özellikler:**
- Duplicate command detection
- Command ack tracking
- Mode synchronization from heartbeat
- Arm status tracking

#### 3.5.3 MAVLinkBridge

**Dosyalar:**
- [`mavlink_bridge.hpp`](include/uav/comm/mavlink_bridge.hpp)
- [`mavlink_bridge.cpp`](src/comm/mavlink_bridge.cpp)

**Handled MAVLink Messages:**
- `HEARTBEAT` - Mode, arm status sync
- `GLOBAL_POSITION_INT` - Position, velocity
- `VFR_HUD` - Airspeed, heading
- `GPS_RAW_INT` - GPS quality
- `SYS_STATUS` - Battery
- `ATTITUDE` - Roll, pitch, yaw
- `STATUSTEXT` - RIV, HSS, BND, MODE commands
- `COMMAND_ACK` - Command acknowledgment
- `PARAM_VALUE` - ArduPilot parameters

**Heartbeat Loop (1Hz):**
- Send heartbeat to Pixhawk and GCS
- Check heartbeat timeout (5s)
- Auto-reconnect if connection lost
- Exponential backoff: 1s → 30s max

#### 3.5.4 StatusTextParser

**Dosyalar:**
- [`statustext_parser.hpp`](include/uav/comm/statustext_parser.hpp)
- [`statustext_parser.cpp`](src/comm/statustext_parser.cpp)

**Protocols:**
```
RIV:ID:LAT:LON:ALT:HDG:SPD:TS           → Opponent data
HSS:COUNT:ID1:LAT1:LON1:RAD1:...        → HSS zones
BND:COUNT:LAT1:LON1:LAT2:LON2:...       → Boundary vertices
LOCK:TARGET_ID                          → Follow/lock target
KAMIKAZE:LAT:LON                         → Dive to ground target
FREE                                     → Free mode (patrol)
ARDUPILOT                                → ArduPilot passthrough
```

**Multi-part Messages:**
- Format: `PREFIX[PART/TOTAL]:content`
- Örnek: `HSS[1/3]:HSS:5:1:39.82:32.75:100:...`
- Buffer timeout: 10 saniye

#### 3.5.5 ParamReader

**Dosyalar:**
- [`param_reader.hpp`](include/uav/comm/param_reader.hpp)
- [`param_reader.cpp`](src/comm/param_reader.cpp)

**Kritik Parametreler:**
- `ARSPD_FBW_MIN` - Stall speed
- `ARSPD_FBW_MAX` - Max airspeed
- `TRIM_ARSPD_CM` - Cruise speed
- `LIM_ROLL_CD` - Max bank angle
- `FENCE_ALT_MAX/MIN` - Altitude limits
- `WP_RADIUS` - Waypoint acceptance radius
- `WP_LOITER_RAD` - Loiter radius

**Request Timing:**
- Critical params: 2 saniye sonra
- All params: 7 saniye sonra (eğer critical hazır değilse)

---

### 3.6 Safety Module (`safety/`)

#### 3.6.1 ZoneChecker

**Dosyalar:**
- [`zone_checker.hpp`](include/uav/safety/zone_checker.hpp)
- [`zone_checker.cpp`](src/safety/zone_checker.cpp)

**Metodlar:**
```cpp
bool isInsideBoundary(const Position&, const Boundary&) const;
bool isInsideAnyHSS(const Position&, const vector<HSSZone>&, double buffer) const;
double distanceToBoundary(const Position&, const Boundary&) const;
double distanceToNearestHSS(const Position&, const vector<HSSZone>&) const;
bool segmentIntersectsAnyHSS(const Position&, const Position&, 
                             const vector<HSSZone>&, double buffer) const;
bool segmentExitsBoundary(const Position&, const Position&, const Boundary&) const;
bool isPositionSafe(const Position&, const Boundary&, 
                    const vector<HSSZone>&, double hss_buffer) const;
```

#### 3.6.2 SafetyGate

**Dosyalar:**
- [`safety_gate.hpp`](include/uav/safety/safety_gate.hpp)
- [`safety_gate.cpp`](src/safety/safety_gate.cpp)

**Config:**
```cpp
struct Config {
    double hss_buffer_m = 40.0;
    double boundary_buffer_m = 20.0;
    double min_altitude_m = 30.0;
    double max_altitude_m = 400.0;
    double max_velocity_ms = 40.0;
};
```

**Metodlar:**
```cpp
bool validatePosition(const Position& target) const;
bool validateVelocity(float vn, float ve, float vd) const;
Position clampToSafe(const Position& target) const;
void clampVelocity(float& vn, float& ve, float& vd) const;
```

**Özellikler:**
- Last line of defense - her komuttan önce kontrol
- Clamp to safe ile güvenli bölgeye zorlama
- 3 saniye öngörülü velocity kontrolü

#### 3.6.3 SafetyMonitor

**Dosyalar:**
- [`safety_monitor.hpp`](include/uav/safety/safety_monitor.hpp)
- [`safety_monitor.cpp`](src/safety/safety_monitor.cpp)

**Config:**
```cpp
struct Config {
    double collision_critical_m = 30.0;
    double collision_high_m = 80.0;
    double collision_medium_m = 200.0;
    double collision_low_m = 500.0;
    double ttc_critical_s = 5.0;
    double ttc_high_s = 10.0;
    double boundary_warn_m = 30.0;
    double hss_buffer_m = 40.0;
};
```

**Monitor Loop (20Hz):**
- Boundary check
- HSS check
- Collision detection (TTC hesaplama)
- GPS quality check
- Battery check
- Comm check

**Event Throttling:**
- Aynı event tipi için 500ms cooldown
- `publishIfChanged()` ile sadece değişimde publish

#### 3.6.4 EscapeManager

**Dosyalar:**
- [`escape_manager.hpp`](include/uav/safety/escape_manager.hpp)
- [`escape_manager.cpp`](src/safety/escape_manager.cpp)

**Stratejiler:**
- Boundary escape: Boundary merkezine doğru 200m
- HSS escape: HSS merkezinden uzaklaşma (radius + 80m)
- Collision escape: Threat bearing + 90° veya heading ±45°

**Alternatif Arama:**
- 30° adımlarla ±150° arama
- Maksimum 10 alternatif denemesi

---

### 3.7 Nav Module (`nav/`)

#### 3.7.1 PathPlanner

**Dosyalar:**
- [`path_planner.hpp`](include/uav/nav/path_planner.hpp)
- [`path_planner.cpp`](src/nav/path_planner.cpp)

**Algoritma: Visibility Graph + Dijkstra**

1. **Obstacle Expansion:**
   - HSS circles → convex polygons (12 segment)
   - Chord compensation: `r *= 1/cos(π/segments)`

2. **Visibility Graph:**
   - Nodes: start + goal + all polygon vertices
   - Edges: Line-of-sight ile bağlantı
   - Obstacle içinden geçen edge'leri atla

3. **Shortest Path:**
   - Dijkstra algoritması
   - Priority queue implementasyonu

4. **Path Simplification:**
   - Redundant waypoint silme
   - Line-of-sight kontrolü

5. **Turn Smoothing:**
   - Dubins arc ile dönüş yumuşatma
   - Minimum 15° dönüş açısı
   - Arc safety check (HSS + boundary)

**Config:**
```cpp
struct Config {
    double hss_buffer_m = 40.0;
    double boundary_buffer_m = 20.0;
    double min_turn_radius_m = 50.0;
    int polygon_segments = 12;
};
```

#### 3.7.2 WaypointChain

**Dosyalar:**
- [`waypoint_chain.hpp`](include/uav/nav/waypoint_chain.hpp)
- [`waypoint_chain.cpp`](src/nav/waypoint_chain.cpp)

**Status:**
```cpp
enum class ChainStatus { 
    EMPTY, IN_PROGRESS, WAYPOINT_REACHED, CHAIN_COMPLETE 
};
```

**Metodlar:**
```cpp
void load(vector<Waypoint> chain, bool circular = false);
ChainStatus update(const Position& own_pos);
optional<Waypoint> currentTarget() const;
void replace(vector<Waypoint> new_chain);  // Smart replace
bool validate(const ZoneChecker&, const Boundary&, 
             const vector<HSSZone>&, double hss_buffer) const;
```

#### 3.7.3 SpeedManager

**Dosyalar:**
- [`speed_manager.hpp`](include/uav/nav/speed_manager.hpp)
- Header-only (source dosyası yok)

**Speeds:**
- cruise: 20 m/s
- patrol: 18 m/s
- pursuit: 30 m/s
- dive: 25 m/s
- min/max: 12-35 m/s

#### 3.7.4 NavigationEngine

**Dosyalar:**
- [`navigation_engine.hpp`](include/uav/nav/navigation_engine.hpp)
- [`navigation_engine.cpp`](src/nav/navigation_engine.cpp)

**Facade Pattern** - PathPlanner + WaypointChain + FlightCommander + SafetyGate

**Metodlar:**
```cpp
bool planRoute(const Position& goal, float airspeed = -1.0f);
ChainStatus update(const Position& own_pos);
bool flyDirect(const Position& target, float airspeed = -1.0f);
bool flyLocalPosition(float north, float east, float down);
bool flyLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);
void clearRoute();
bool revalidateRoute();
```

**Rate Limiting:**
- Min command interval: 250ms
- Min plan interval: 2000ms

**Look-ahead:**
- 1.5x acceptance radius'da sonraki waypoint'i gönder
- Daha smooth uçuş için

---

### 3.8 Mission Module (`mission/`)

#### 3.8.1 IMissionMode Interface

**Dosyalar:**
- [`mission_mode.hpp`](include/uav/mission/mission_mode.hpp)

```cpp
class IMissionMode {
public:
    virtual void onEnter() = 0;
    virtual void onExit() = 0;
    virtual void tick(const UAVState& own_state) = 0;
    virtual MissionGoal getMissionGoal() = 0;
    virtual string getName() const = 0;
};
```

#### 3.8.2 PatrolMode

**Dosyalar:**
- [`patrol_mode.hpp`](include/uav/mission/patrol_mode.hpp)
- [`patrol_mode.cpp`](src/mission/patrol_mode.cpp)

**Strateji:**
1. Boundary vertices'ten waypoint oluştur
2. Her vertex için boundary inset (50m)
3. Angle'dan sort et
4. En yakın waypoint'ten başla
5. Waypoint'leri sırayla gez

**Config:**
```cpp
struct Config {
    double boundary_inset_m = 50.0;
    double cruise_altitude_m = 100.0;
    float patrol_speed_ms = 18.0f;
};
```

#### 3.8.3 FollowMode

**Dosyalar:**
- [`follow_mode.hpp`](include/uav/mission/follow_mode.hpp)
- [`follow_mode.cpp`](src/mission/follow_mode.cpp)

**Phases:**
- APPROACH: >200m mesafe - Path planning ile yaklaşma
- PURSUIT: ≤200m mesafe - Proportional Navigation ile intercept
- REACQUIRE: Target kayıp (max 30s)

**Proportional Navigation:**
```cpp
LOS_rate = (bearing_t - bearing_t-1) / dt
pn_offset = pn_gain * LOS_rate * t_go
intercept_bearing = LOS_bearing + pn_offset
```

**Config:**
```cpp
struct Config {
    double pursuit_range_m = 200.0;
    double reacquire_timeout_s = 5.0;
    double max_reacquire_s = 30.0;
    double prediction_horizon_s = 3.0;
    double target_stale_s = 3.0;
    double plan_update_interval_s = 3.0;
    float approach_speed_ms = 20.0f;
    float pursuit_speed_ms = 30.0f;
    double pn_gain = 3.0;
    double pn_max_lead_deg = 45.0;
};
```

#### 3.8.4 QRDiveMode

**Dosyalar:**
- [`qr_dive_mode.hpp`](include/uav/mission/qr_dive_mode.hpp)
- [`qr_dive_mode.cpp`](src/mission/qr_dive_mode.cpp)

**Phases:**
- APPROACH: Entry point'e (500m arka) git
- ALIGN: Heading ve altitude align et
- DIVE: -45° pitch ile dalış
- CLIMB: 100m'e tırman
- COMPLETED: Callback → PATROL

**Config:**
```cpp
struct Config {
    double approach_altitude_m = 120.0;
    double min_altitude_m = 20.0;
    double climb_altitude_m = 100.0;
    double entry_corridor_m = 500.0;
    double align_heading_tolerance_deg = 15.0;
    double align_altitude_tolerance_m = 10.0;
    double dive_pitch_deg = -45.0;
    double dive_speed_ms = 25.0;
    double target_pass_distance_m = 15.0;
    double climb_timeout_s = 30.0;
    bool use_position_dive = true;
    double dive_step_interval_s = 0.5;
};
```

**HSS Abort:**
- DIVE/ALIGN sırasında HSS'e çok yakınsa abort
- CLIMB phase'ine geç

#### 3.8.5 ArduPilotPassthrough

**Dosyalar:**
- [`ardupilot_mode.hpp`](include/uav/mission/ardupilot_mode.hpp)
- [`ardupilot_mode.cpp`](src/mission/ardupilot_mode.cpp)

**Amaç:**
- ArduPilot'a tam kontrol ver
- Manual flight için
- Failsafe fallback modu

#### 3.8.6 MissionDirector

**Dosyalar:**
- [`mission_director.hpp`](include/uav/mission/mission_director.hpp)
- [`mission_director.cpp`](src/mission/mission_director.cpp)

**Sorumluluklar:**
- Mode transition management
- Active mode tick çağrısı
- Mode command handling
- RTL trigger

**Transition Rules:**
```
ARDUPILOT → HERŞEY: İzin verili (operator override)
HERŞEY → ARDUPILOT: İzin verili (safety fallback)
QR_DIVE → SADECE PATROL: Tamamlanmadan çıkılamaz
FOLLOW → PATROL/QR_DIVE: İzin verili
PATROL → HERŞEY: İzin verili
```

**Tick Loop (5Hz):**
```cpp
void MissionDirector::tick() {
    // GUIDED mode kontrolü
    if (!commander_->isInGuidedMode()) {
        commander_->setMode(GUIDED);
    }
    
    // Active handler tick
    activeHandler()->tick(own_state);
}
```

---

### 3.9 Vision Module (`vision/`)

**Dosyalar:**
- [`tracking_controller.hpp`](include/uav/vision/tracking_controller.hpp)
- [`tracking_controller.cpp`](src/vision/tracking_controller.cpp)

**Sınıf:**
```cpp
class TrackingController {
public:
    struct Config {
        int max_age_frames = 30;
        int min_hits = 3;
        double dist_threshold = 100.0;
    };
    
    void update(const vector<Detection>& detections);
    vector<TrackedObject> getActiveTracks() const;
};
```

**Durum:**
- Implementasyon var ama kullanılmıyor
- Gelecek nesne takibi için hazır

---

### 3.10 App Module (`app/`)

**Dosyalar:**
- [`application.hpp`](include/uav/app/application.hpp)
- [`application.cpp`](src/app/application.cpp)
- [`main.cpp`](src/main.cpp)

**Initialization Order:**
```
1. initCore()     → config, event_bus, thread_pool, world_model
2. initComm()     → mavlink_bridge, flight_commander
3. initSafety()   → zone_checker, safety_gate, safety_monitor
4. initNav()      → path_planner, speed_manager, navigation_engine
5. initMission()  → mission_director + all modes
6. subscribeEvents() → event handler kayıtları
7. loadBoundaryFromConfig()
8. loadHSSFromConfig()
```

**Main Loop (5Hz):**
```cpp
void Application::mainLoop() {
    while (running_) {
        mission_->tick();
        
        // 2 saniyede bir stale opponent temizle
        world_model_->removeStaleOpponents(10.0s);
        
        // 2 saniyede bir dashboard yaz
        printDashboard();
        
        sleep_until(next_tick);
    }
}
```

**Event Subscriptions:**
```
GPS_QUALITY_CRITICAL → triggerRTL("gps_critical")
BATTERY_CRITICAL → triggerRTL("battery_critical")
HEARTBEAT_TIMEOUT → triggerRTL("heartbeat_timeout")
HEARTBEAT_RECOVERED → safety_monitor_->notifyHeartbeat()
PARAMS_READY → applyArduPilotParams()
HSS_ZONE_UPDATED → nav_engine_->revalidateRoute()
BOUNDARY_UPDATED → nav_engine_->revalidateRoute()
MODE_COMMAND_RECEIVED → mission_->handleModeCommand()
SAFETY_VIOLATION → handleEscapeNeeded()
COLLISION_WARNING → escape_manager_->generateCollisionEscape()
```

**Dashboard Output:**
```
=== UAV v10 Dashboard ===
Mode: PATROL | Transitions: 3
Pos: 37.xxx, -122.xxx | Alt: 100.0m | Spd: 18.0m/s | Hdg: 270.0
GPS: fix=3 sat=12 | Bat: 85%
Safety: OK | Collision: NONE | Boundary: INSIDE | CommLoss: NO
World: Opponents=2 | HSS=1 | Boundary=YES
MAVLink: PIX=OK GCS=OK | Mode=GUIDED | Armed=YES | RX=1523 TX=892
```

---

## 4. VERİ AKIŞI

### 4.1 MAVLink Message Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          MAVLINK MESSAGE FLOW                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Pixhawk ────────> ConnectionManager ────> MAVLinkBridge ────> WorldModel  │
│  (UDP:14550)                              │                    (setOwnState)│        │
│                                           │                        │        │
│                                           ├──> GLOBAL_POS_INT ─────┤        │
│                                           ├──> VFR_HUD ────────────┤        │
│                                           ├──> GPS_RAW_INT ────────┤        │
│                                           ├──> SYS_STATUS ─────────┤        │
│                                           ├──> ATTITUDE ───────────┤        │
│                                           ├──> HEARTBEAT ──────────┤        │
│                                           └──> STATUSTEXT ─────────┤        │
│                                                                    │        │
│                                          ┌─────────────────────────┘        │
│                                          │                                  │
│                                          ▼                                  │
│                                    EventBus ──> Application ──> MissionDir │
│                                                                    │        │
│  GCS ──────────> ConnectionManager ──> MAVLinkBridge              │        │
│  (TCP:14552)                            │                          │        │
│                                         └──> STATUSTEXT ──> parse │        │
│                                              (RIV/HSS/BND/MODE)   │        │
│                                                                     │        │
│  Pixhawk <──── ConnectionManager <─── FlightCommander <── NavEngine│        │
│               (DO_REPOSITION)         (flyToPosition)   (planRoute)│        │
│               (SET_POS_TARGET_NED)    (flyLocalPos/Vel)             │        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Command Flow

```
planRoute(goal)
    │
    ├──> WorldModel.getOwnState() ───> start position
    ├──> WorldModel.getBoundary() ──> boundary
    ├──> WorldModel.getActiveHSSZones() ──> obstacles
    │
    ├──> PathPlanner.plan(start, goal, boundary, zones)
    │       │
    │       ├──> isDirectPathClear() ──> HSS/boundary check
    │       │       YES → return {start, goal}
    │       │       NO  → continue
    │       │
    │       ├──> expandObstacles() ──> circles → polygons
    │       ├──> buildNodes() ──> start + goal + vertices
    │       ├──> buildAdjacency() ──> visibility edges
    │       ├──> dijkstra() ──> shortest path
    │       ├──> simplifyPath() ──> remove redundant
    │       └──> smoothTurns() ──> Dubins arc waypoints
    │
    ├──> WaypointChain.replace(waypoints)
    └──> sendCurrentWaypoint()
            │
            ├──> SafetyGate.validatePosition()
            │       UNSAFE → clampToSafe()
            └──> FlightCommander.flyToPosition()
                    └──> DO_REPOSITION (MAVLink)

flyDirect(target)
    ├──> SafetyGate.validatePosition() → clampToSafe()
    └──> FlightCommander.flyToPosition()

flyLocalVelocity(vn, ve, vd)
    ├──> SafetyGate.validateVelocity() → clampVelocity()
    └──> FlightCommander.setLocalVelocity()
            └──> SET_POSITION_TARGET_LOCAL_NED (velocity mask)
```

---

## 5. THREAD MİMARİSİ

```
┌──────────────────────────────────────────────────────────────┐
│                    THREAD ARCHITECTURE                        │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  Main Thread                                                 │
│  ├── Application.run() ──> mainLoop() [5Hz]                │
│  │   ├── MissionDirector.tick()                             │
│  │   ├── removeStaleOpponents() [2s interval]               │
│  │   └── printDashboard() [2s interval]                     │
│  │                                                           │
│  │  Signal: SIGINT/SIGTERM → requestShutdown()              │
│  │                                                           │
│  Safety Monitor Thread [20Hz]                               │
│  ├── checkBoundary()                                        │
│  ├──> checkHSS()                                             │
│  ├──> checkCollisions()                                     │
│  ├──> checkGPSQuality()                                     │
│  ├──> checkBattery()                                        │
│  ├──> checkCommLoss()                                       │
│  └──> publishEvents() [throttled 500ms]                     │
│                                                              │
│  Heartbeat Thread [1Hz]                                     │
│  ├──> sendHeartbeat() → Pixhawk                             │
│  ├──> sendHeartbeat() → GCS                                 │
│  ├──> checkHeartbeatTimeout() [5s]                          │
│  └──> autoReconnect() [exponential backoff]                │
│                                                              │
│  Event Dispatch Thread                                      │
│  ├──> dispatchQueuedEvents()                                │
│  └──> callSubscribedCallbacks()                             │
│                                                              │
│  MAVLink Receive Thread                                     │
│  ├──> receiveUDP() → Pixhawk                                │
│  ├──> receiveTCP() → GCS                                    │
│  └──> MAVLinkBridge.parseMessage()                          │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

**Thread Senkronizasyonu:**
- `WorldModel`: `shared_mutex` (reader-writer lock)
- `EventBus`: `mutex` + queue
- `WaypointChain`: `mutex`
- `SafetyMonitor`: Independent thread, event-based communication

---

## 6. EVENT-DRIVEN SİSTEM

### 6.1 EventBus Pub/Sub Pattern

```cpp
// Publish
eventBus_->publish(EventType::SAFETY_VIOLATION, safetyState);

// Subscribe
eventBus_->subscribe(EventType::SAFETY_VIOLATION, [this](const Event& e) {
    auto state = std::any_cast<SafetyState>(e.payload);
    this->handleSafetyViolation(state);
});
```

### 6.2 Event Listesi ve Yayınlayanlar

| Event | Yayınlayan | Aboneolan | Aksiyon |
|-------|-----------|-----------|---------|
| `SAFETY_VIOLATION` | SafetyMonitor | Application | handleEscapeNeeded() |
| `BOUNDARY_VIOLATION` | SafetyMonitor | Application | triggerRTL() |
| `HSS_VIOLATION` | SafetyMonitor | Application | triggerRTL() |
| `COLLISION_WARNING` | SafetyMonitor | Application | escape_manager_->generateCollisionEscape() |
| `OPPONENT_UPDATED` | MAVLinkBridge | WorldModel | updateOpponent() |
| `HSS_ZONE_UPDATED` | MAVLinkBridge | WorldModel | updateHSSZone() |
| `BOUNDARY_UPDATED` | MAVLinkBridge | WorldModel | setBoundary() |
| `OWN_STATE_UPDATED` | MAVLinkBridge | WorldModel | setOwnState() |
| `MODE_CHANGED` | MissionDirector | EventBus | notify listeners |
| `MODE_COMMAND_RECEIVED` | StatusTextParser | MissionDirector | handleModeCommand() |
| `HEARTBEAT_TIMEOUT` | MAVLinkBridge | Application | triggerRTL() |
| `HEARTBEAT_RECOVERED` | MAVLinkBridge | Application | safety_monitor_->notifyHeartbeat() |
| `GPS_QUALITY_CRITICAL` | SafetyMonitor | Application | triggerRTL() |
| `BATTERY_CRITICAL` | SafetyMonitor | Application | triggerRTL() |
| `PARAMS_READY` | ParamReader | Application | applyArduPilotParams() |

### 6.3 Event Throttling

Aynı event tipi için 500ms cooldown:
```cpp
void EventBus::publish(EventType type, std::any payload) {
    auto now = nowUs();
    if (last_publish_time_[type] + 500ms > now) {
        return;  // Skip duplicate
    }
    last_publish_time_[type] = now;
    queue_.push({type, payload, now});
}
```

---

## 7. GÜVENLİK SİSTEMİ

### 7.1 Çok Katmanlı Güvenlik

```
┌─────────────────────────────────────────────────────────────┐
│                    SAFETY LAYERS                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. PathPlanner                                            │
│     ├── HSS avoidance (buffer zone)                        │
│     └── Boundary constraint                                 │
│                                                             │
│  2. SafetyGate (Last Line of Defense)                      │
│     ├── validatePosition() → clampToSafe()                 │
│     └── validateVelocity() → clampVelocity()               │
│                                                             │
│  3. SafetyMonitor (Continuous Monitoring)                  │
│     ├── 20Hz check loop                                     │
│     ├── Boundary violation → RTL                           │
│     ├── HSS violation → RTL                                │
│     ├── Collision detection → Escape                       │
│     └── GPS/Battery critical → RTL                         │
│                                                             │
│  4. EscapeManager                                          │
│     ├── Boundary escape → center + 200m                    │
│     ├── HSS escape → away from HSS                         │
│     └── Collision escape → evasive maneuver                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Risk Level'ler

```cpp
enum class RiskLevel {
    NONE,       // Hiçbir tehdit yok
    LOW,        // Uzak tehdit (>500m)
    MEDIUM,     // Orta mesafe tehdit (200-500m)
    HIGH,       >>Yakın tehdit (80-200m)
    CRITICAL    // Kritik tehdit (<80m)
};
```

### 7.3 Safety State

```cpp
struct SafetyState {
    RiskLevel overall_risk = RiskLevel::NONE;
    BoundaryStatus boundary_status = BoundaryStatus::INSIDE;
    bool in_hss = false;
    bool collision_risk = false;
    bool gps_quality_good = true;
    bool battery_ok = true;
    bool comm_loss = false;
    uint64_t timestamp_us;
    
    bool isSafe() const;
};
```

---

## 8. PATH PLANNING ALGORİTMASI

### 8.1 Visibility Graph + Dijkstra

```
┌─────────────────────────────────────────────────────────────┐
│                  PATH PLANNING FLOW                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Input: start, goal, boundary, HSS zones                   │
│         │                                                   │
│         ▼                                                   │
│  ┌─────────────────┐                                       │
│  │ isDirectPathClear? │                                     │
│  └────┬────────┬───┘                                       │
│       YES      │ NO                                         │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 1. Expand Obstacles  │                        │
│       │    │    circle → polygon   │                        │
│       │    │    (12 segments)      │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 2. Build Nodes       │                        │
│       │    │    start + goal +    │                        │
│       │    │    all vertices      │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 3. Build Adjacency   │                        │
│       │    │    visibility graph  │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 4. Dijkstra          │                        │
│       │    │    shortest path     │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 5. Simplify Path     │                        │
│       │    │    remove redundant  │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       │    ┌───▼──────────────────┐                        │
│       │    │ 6. Smooth Turns      │                        │
│       │    │    Dubins arcs       │                        │
│       │    └───┬──────────────────┘                        │
│       │        │                                            │
│       └────────┼────────────────────┐                      │
│                │                    │                      │
│                ▼                    ▼                      │
│           Direct Path          Planned Path                 │
│           [start, goal]        [w1, w2, w3, ...]           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 Obstacle Expansion

HSS dairelerini convex polygon'a çevirme:
```cpp
// Chord compensation
double chord_comp = radius / std::cos(M_PI / segments);
double r_expanded = chord_comp + buffer;

// 12 segment (30° adım)
for (int i = 0; i < segments; ++i) {
    double angle = 2.0 * M_PI * i / segments;
    vertices.push_back({
        center.latitude + r_expanded * std::cos(angle) / EARTH_RADIUS,
        center.longitude + r_expanded * std::sin(angle) / (EARTH_RADIUS * std::cos(center.latitude))
    });
}
```

### 8.3 Visibility Graph

Line-of-sight kontrolü:
```cpp
bool isVisible(const Position& a, const Position& b, const vector<Polygon>& obstacles) {
    Segment ab = {a, b};
    for (const auto& obs : obstacles) {
        if (segmentIntersectsPolygon(ab, obs)) {
            return false;
        }
    }
    return true;
}
```

### 8.4 Dijkstra Algoritması

```cpp
vector<Position> dijkstra(const vector<VNode>& nodes, const vector<Edge>& edges, int start_idx, int goal_idx) {
    priority_queue<NodeDist> pq;
    vector<double> dist(nodes.size(), INF);
    vector<int> prev(nodes.size(), -1);
    
    dist[start_idx] = 0.0;
    pq.push({start_idx, 0.0});
    
    while (!pq.empty()) {
        auto [u, d] = pq.top(); pq.pop();
        
        if (u == goal_idx) break;
        if (d > dist[u]) continue;
        
        for (const auto& [v, w] : adj[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({v, dist[v]});
            }
        }
    }
    
    // Reconstruct path
    vector<Position> path;
    for (int v = goal_idx; v != -1; v = prev[v]) {
        path.push_back(nodes[v].pos);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
```

---

## 9. MISSION MODLARI

### 9.1 Mod Geçiş Diyagramı

```
┌─────────────────────────────────────────────────────────────┐
│                    MISSION MODES                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│    ┌──────────┐    ┌──────────┐    ┌──────────┐           │
│    │ARDUPILOT │◄──►│ PATROL   │◄──►│ FOLLOW   │           │
│    └────┬─────┘    └────┬─────┘    └────┬─────┘           │
│         │               │               │                  │
│         │               │               │                  │
│         └───────────────┴───┬───────────┘                  │
│                             ▼                              │
│                        ┌──────────┐                        │
│                        │ QR_DIVE  │                        │
│                        └─────┬────┘                        │
│                              │                             │
│                              ▼                             │
│                         PATROL (tamamlandıktan sonra)       │
│                                                             │
│  Transition Rules:                                         │
│  - ARDUPILOT ↔ HERŞEY: Her zaman izin verili              │
│  - QR_DIVE → PATROL: Sadece tamamlandıktan sonra          │
│  - QR_DIVE → HERŞEY: Tamamlanmadan çıkılamaz             │
│  - PATROL/FOLLOW ↔ HERŞEY: Serbest                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 9.2 PatrolMode

**Strateji:**
1. Boundary vertices'ten waypoint oluştur
2. Her vertex için boundary inset (50m)
3. Angle'dan sort et
4. En yakın waypoint'ten başla
5. Waypoint'leri sırayla gez

```cpp
void PatrolMode::tick(const UAVState& own_state) {
    if (waypoint_chain_.isEmpty()) {
        auto boundary = world_model_->getBoundary();
        auto waypoints = generatePatrolWaypoints(*boundary);
        waypoint_chain_.load(waypoints, true);  // circular
    }
    
    auto status = waypoint_chain_.update(own_state.position);
    if (status == ChainStatus::WAYPOINT_REACHED) {
        auto next = waypoint_chain_.currentTarget();
        nav_engine_->flyDirect(*next, config_.patrol_speed_ms);
    }
}
```

### 9.3 FollowMode

**Phases:**
- `APPROACH`: >200m mesafe - Path planning ile yaklaşma
- `PURSUIT`: ≤200m mesafe - Proportional Navigation ile intercept
- `REACQUIRE`: Target kayıp (max 30s)

```cpp
void FollowMode::tick(const UAVState& own_state) {
    auto target = world_model_->getOpponent(target_id_);
    if (!target || target->ageSeconds() > config_.target_stale_s) {
        phase_ = Phase::REACQUIRE;
        return;
    }
    
    double dist = haversineDistance(own_state.position, target->position);
    
    if (dist > config_.pursuit_range_m) {
        phase_ = Phase::APPROACH;
        nav_engine_->planRoute(target->position, config_.approach_speed_ms);
    } else {
        phase_ = Phase::PURSUIT;
        auto intercept = calculatePNIntercept(own_state, *target);
        nav_engine_->flyDirect(intercept, config_.pursuit_speed_ms);
    }
}
```

**Proportional Navigation:**
```cpp
Position FollowMode::calculatePNIntercept(const UAVState& own, const UAVState& target) {
    // LOS rate hesapla
    double los_bearing = bearing(own.position, target.position);
    double los_rate = (los_bearing - last_los_bearing_) / dt;
    
    // PN offset
    double t_go = dist / (own.airspeed + target.airspeed);
    double pn_offset = config_.pn_gain * los_rate * t_go;
    pn_offset = std::clamp(pn_offset, -config_.pn_max_lead_deg, config_.pn_max_lead_deg);
    
    // Intercept bearing
    double intercept_bearing = los_bearing + pn_offset;
    
    // Predicted target position
    double pred_dist = own.airspeed * config_.prediction_horizon_s;
    return destinationPoint(target.position, target.heading + 180.0, pred_dist);
}
```

### 9.4 QRDiveMode

**Phases:**
- `APPROACH`: Entry point'e (500m arka) git
- `ALIGN`: Heading ve altitude align et
- `DIVE`: -45° pitch ile dalış
- `CLIMB`: 100m'e tırman
- `COMPLETED`: Callback → PATROL

```cpp
void QRDiveMode::tick(const UAVState& own_state) {
    switch (phase_) {
        case Phase::APPROACH:
            if (reachedEntry(own_state)) {
                phase_ = Phase::ALIGN;
            }
            nav_engine_->flyDirect(entry_point_, config_.approach_speed_ms);
            break;
            
        case Phase::ALIGN:
            if (isAligned(own_state)) {
                phase_ = Phase::DIVE;
                dive_start_time_ = now();
            } else {
                // Adjust heading and altitude
                double hdg_error = target_heading_ - own_state.heading;
                double alt_error = config_.approach_altitude_m - own_state.position.altitude;
                nav_engine_->flyLocalVelocity(0, 0, -alt_error, hdg_error * 0.1);
            }
            break;
            
        case Phase::DIVE:
            if (passedTarget(own_state)) {
                phase_ = Phase::CLIMB;
                nav_engine_->flyDirect({own_state.position.latitude,
                                        own_state.position.longitude,
                                        config_.climb_altitude_m});
            } else {
                // -45° pitch dive
                double vn = own_state.airspeed * std::cos(std::.radians(-45.0));
                double vd = own_state.airspeed * std::sin(std::.radians(-45.0));
                nav_engine_->flyLocalVelocity(vn, 0, vd);
            }
            break;
            
        case Phase::CLIMB:
            if (own_state.position.altitude >= config_.climb_altitude_m) {
                phase_ = Phase::COMPLETED;
                on_completed_callback_();
            }
            break;
    }
}
```

---

## 10. KONFİGÜRASYON YÖNETİMİ

### 10.1 ConfigManager

**Dosyalar:**
- [`config_manager.hpp`](include/uav/config/config_manager.hpp)
- [`config_manager.cpp`](src/config/config_manager.cpp)

**Kullanım:**
```cpp
// Load YAML
config_->load("config/default.yaml");

// Get values
double hss_buffer = config_->getDouble("safety.hss_buffer_m");
int gps_min_fix = config_->getInt("safety.min_gps_fix");
bool use_sitl = config_->getBool("simulation.use_sitl");

// Default values
double timeout = config_->getDouble("comm.timeout_s", 5.0);
```

### 10.2 default.yaml Yapısı

```yaml
# Safety
safety:
  hss_buffer_m: 40.0
  boundary_buffer_m: 20.0
  min_altitude_m: 30.0
  max_altitude_m: 400.0
  min_gps_fix: 3
  min_satellites: 8
  max_gps_hdop: 200
  
  collision:
    critical_m: 30.0
    high_m: 80.0
    medium_m: 200.0
    low_m: 500.0
    ttc_critical_s: 5.0
    ttc_high_s: 10.0

# Navigation
navigation:
  cruise_speed_ms: 20.0
  patrol_speed_ms: 18.0
  pursuit_speed_ms: 30.0
  dive_speed_ms: 25.0
  min_speed_ms: 12.0
  max_speed_ms: 35.0
  
  path_planner:
    hss_buffer_m: 40.0
    boundary_buffer_m: 20.0
    min_turn_radius_m: 50.0
    polygon_segments: 12
    
  waypoint:
    acceptance_radius_m: 30.0
    lookahead_ratio: 1.5

# Communication
comm:
  pixhawk:
    protocol: udpin
    port: 14550
    host: "127.0.0.1"
  
  gcs:
    protocol: tcpin
    port: 14552
    host: "0.0.0.0"
  
  heartbeat:
    interval_s: 1.0
    timeout_s: 5.0
    reconnect_backoff_s: 1.0
    reconnect_max_s: 30.0

# Mission
mission:
  patrol:
    boundary_inset_m: 50.0
    cruise_altitude_m: 100.0
    patrol_speed_ms: 18.0
  
  follow:
    pursuit_range_m: 200.0
    reacquire_timeout_s: 5.0
    max_reacquire_s: 30.0
    prediction_horizon_s: 3.0
    target_stale_s: 3.0
    plan_update_interval_s: 3.0
    approach_speed_ms: 20.0
    pursuit_speed_ms: 30.0
    pn_gain: 3.0
    pn_max_lead_deg: 45.0
  
  qr_dive:
    approach_altitude_m: 120.0
    min_altitude_m: 20.0
    climb_altitude_m: 100.0
    entry_corridor_m: 500.0
    align_heading_tolerance_deg: 15.0
    align_altitude_tolerance_m: 10.0
    dive_pitch_deg: -45.0
    dive_speed_ms: 25.0
    target_pass_distance_m: 15.0
    climb_timeout_s: 30.0
    use_position_dive: true
    dive_step_interval_s: 0.5

# Logging
logging:
  level: info  # debug, info, warn, error
  console: true
  file: true
  file_path: "logs/uav.log"
  max_size_mb: 100
  max_files: 10
  flush_interval_s: 3.0
```

---

## 11. BUILD SİSTEMİ

### 11.1 CMakeLists.txt

**Build Type:**
- Debug: Debug symbols, optimization off
- Release: Optimization level 3, no debug symbols
- RelWithDebInfo: Optimization + debug symbols

**Unity Build:**
```cmake
# Unity build for faster compilation
set(UNITY_BUILD ON)
```

### 11.2 Build Komutları

**Windows (Visual Studio):**
```batch
build.bat          # Debug build
build.bat Release  # Release build
```

**Linux/macOS:**
```bash
./build.sh          # Debug build
./build.sh Release  # Release build
```

### 11.3 Bağımlılıklar

- **yaml-cpp 0.8.0**: YAML config parsing
- **spdlog**: Logging
- **mavlink**: MAVLink protocol
- **nlohmann/json**: JSON parsing (STATUSTEXT)

---

## 12. BİLİNEN SORUNLAR VE ÇÖZÜMLER

### 12.1 Path Planning Bug'ları (PATH_PLANNING_ANALYSIS.md)

**BUG-1: Direct Path Clear HSS Check**
- **Sorun:** `isDirectPathClear()` HSS buffer'ı kontrol etmiyor
- **Durum:** DÜZELTİLDİ
- **Çözüm:** `segmentIntersectsAnyHSS()` çağrısında buffer parametresi eklendi

**BUG-2: Goal Position Validation**
- **Sorun:** Goal position HSS içindeyse kontrol yok
- **Durum:** DÜZELTİLDİ
- **Çözüm:** PathPlanner başında goal validation eklendi

**BUG-3: Path Sampling Eksikliği**
- **Sorun:** Dubins arc'lar HSS içinden geçebilir
- **Durum:** BEKLEMEDE
- **Geçici çözüm:** Daha büyük HSS buffer kullanımı

### 12.2 ArduPilot Param Sync

**Sorun:** Bazı parametreler ArduPilot ile senkronize olmayabilir
**Çözüm:** ParamReader kritik parametreleri okur ve varsayılan değerleri uygular

### 12.3 GPS Quality Check

**Sorun:** GPS HDOP değeri kontrol edilmiyor
**Çözüm:** `SafetyMonitor` GPS quality check ekledi (fix type + HDOP + satellite count)

---

## 13. GELECEK GELİŞTİRME ÖNERİLERİ

### 13.1 Path Planning İyileştirmeleri

1. **Path Sampling:**
   - Dubins arc'ları sample et ve her noktayı HSS kontrolü
   - 5-10m aralıklarla kontrol

2. **Goal Validation:**
   - Goal position HSS içindeyse en yakın güvenli nokta bul
   - Alternatif goal önerileri

3. **Altitude-Aware HSS:**
   - HSS zones'a altitude range ekle
   - 3D path planning

### 13.2 Vision İyileştirmeleri

1. **QR Detection:**
   - OpenCV ile QR kod tespiti
   - QRDiveMode'a entegrasyon

2. **Object Tracking:**
   - TrackingController'ı aktif et
   - Opponent tracking için kullan

### 13.3 İletişim İyileştirmeleri

1. **MAVLink 2.0:**
   - MAVLink 2.0 desteği
   - Signature ve encryption

2. **QGroundControl Integration:**
   - QGC UI entegrasyonu
   - Mission upload/download

### 13.4 Performance İyileştirmeleri

1. **Path Planning Cache:**
   - Aynı start-goal için path cache'le
   - Route revalidation'da cache kullan

2. **Event Batching:**
   - Event'leri batch'le ve toplu gönder
   - Daha az lock contention

---

## 14. REFERANSLAR

### 14.1 Dokümanlar

- [`SYSTEM_DOCUMENTATION.md`](SYSTEM_DOCUMENTATION.md) - Ana sistem dökümantasyonu
- [`DESIGN_SPEC.txt`](DESIGN_SPEC.txt) - Tasarım spesifikasyonu
- [`PATH_PLANNING_ANALYSIS.md`](PATH_PLANNING_ANALYSIS.md) - Path planning analizi

### 14.2 Header Files (include/uav/)

**Core:**
- [`core/types.hpp`](include/uav/core/types.hpp) - Position, UAVState, HSSZone, Boundary, enums
- [`core/geo.hpp`](include/uav/core/geo.hpp) - Geographic calculations
- [`core/time_utils.hpp`](include/uav/core/time_utils.hpp) - Time utilities

**Config:**
- [`config/config_manager.hpp`](include/uav/config/config_manager.hpp) - YAML config manager

**Infra:**
- [`infra/log_manager.hpp`](include/uav/infra/log_manager.hpp) - Logging (spdlog wrapper)
- [`infra/event_bus.hpp`](include/uav/infra/event_bus.hpp) - Event pub/sub system
- [`infra/thread_pool.hpp`](include/uav/infra/thread_pool.hpp) - Thread pool

**World Model:**
- [`world_model/world_model.hpp`](include/uav/world_model/world_model.hpp) - State storage

**Comm:**
- [`comm/connection_manager.hpp`](include/uav/comm/connection_manager.hpp) - UDP/TCP connections
- [`comm/flight_commander.hpp`](include/uav/comm/flight_commander.hpp) - MAVLink commands
- [`comm/mavlink_bridge.hpp`](include/uav/comm/mavlink_bridge.hpp) - MAVLink message handler
- [`comm/statustext_parser.hpp`](include/uav/comm/statustext_parser.hpp) - STATUSTEXT protocol
- [`comm/param_reader.hpp`](include/uav/comm/param_reader.hpp) - ArduPilot params

**Safety:**
- [`safety/zone_checker.hpp`](include/uav/safety/zone_checker.hpp) - Boundary/HSS checks
- [`safety/safety_gate.hpp`](include/uav/safety/safety_gate.hpp) - Position/velocity validation
- [`safety/safety_monitor.hpp`](include/uav/safety/safety_monitor.hpp) - 20Hz safety loop
- [`safety/escape_manager.hpp`](include/uav/safety/escape_manager.hpp) - Escape maneuvers

**Nav:**
- [`nav/path_planner.hpp`](include/uav/nav/path_planner.hpp) - Visibility Graph + Dijkstra
- [`nav/waypoint_chain.hpp`](include/uav/nav/waypoint_chain.hpp) - Waypoint management
- [`nav/speed_manager.hpp`](include/uav/nav/speed_manager.hpp) - Speed profiles
- [`nav/navigation_engine.hpp`](include/uav/nav/navigation_engine.hpp) - Navigation facade

**Mission:**
- [`mission/mission_mode.hpp`](include/uav/mission/mission_mode.hpp) - IMissionMode interface
- [`mission/patrol_mode.hpp`](include/uav/mission/patrol_mode.hpp) - Patrol mission
- [`mission/follow_mode.hpp`](include/uav/mission/follow_mode.hpp) - Follow with PN
- [`mission/qr_dive_mode.hpp`](include/uav/mission/qr_dive_mode.hpp) - QR dive mission
- [`mission/ardupilot_mode.hpp`](include/uav/mission/ardupilot_mode.hpp) - ArduPilot passthrough
- [`mission/mission_director.hpp`](include/uav/mission/mission_director.hpp) - Mode manager

**Vision:**
- [`vision/tracking_controller.hpp`](include/uav/vision/tracking_controller.hpp) - Object tracking

**App:**
- [`app/application.hpp`](include/uav/app/application.hpp) - Main application

### 14.3 Source Files (src/)

**Core:**
- [`core/geo.cpp`](src/core/geo.cpp) - Geographic calculations (haversine, bearing, etc.)
- [`core/types.cpp`](src/core/types.cpp) - toString functions

**Config:**
- [`config/config_manager.cpp`](src/config/config_manager.cpp) - YAML config implementation

**Infra:**
- [`infra/log_manager.cpp`](src/infra/log_manager.cpp) - Logger initialization
- [`infra/event_bus.cpp`](src/infra/event_bus.cpp) - Event dispatch
- [`infra/thread_pool.cpp`](src/infra/thread_pool.cpp) - Thread pool workers

**World Model:**
- [`world_model/world_model.cpp`](src/world_model/world_model.cpp) - State storage implementation

**Comm:**
- [`comm/connection_manager.cpp`](src/comm/connection_manager.cpp) - Socket operations
- [`comm/flight_commander.cpp`](src/comm/flight_commander.cpp) - MAVLink commands
- [`comm/mavlink_bridge.cpp`](src/comm/mavlink_bridge.cpp) - Message parsing
- [`comm/statustext_parser.cpp`](src/comm/statustext_parser.cpp) - Protocol parsing
- [`comm/param_reader.cpp`](src/comm/param_reader.cpp) - Param reading

**Safety:**
- [`safety/zone_checker.cpp`](src/safety/zone_checker.cpp) - Geometric checks
- [`safety/safety_gate.cpp`](src/safety/safety_gate.cpp) - Validation & clamping
- [`safety/safety_monitor.cpp`](src/safety/safety_monitor.cpp) - 20Hz loop
- [`safety/escape_manager.cpp`](src/escape_manager.cpp) - Escape logic

**Nav:**
- [`nav/path_planner.cpp`](src/nav/path_planner.cpp) - Path planning (434 lines)
- [`nav/waypoint_chain.cpp`](src/nav/waypoint_chain.cpp) - Waypoint management
- [`nav/navigation_engine.cpp`](src/nav/navigation_engine.cpp) - Navigation facade

**Mission:**
- [`mission/patrol_mode.cpp`](src/mission/patrol_mode.cpp) - Patrol logic
- [`mission/follow_mode.cpp`](src/mission/follow_mode.cpp) - Follow + PN (197 lines)
- [`mission/qr_dive_mode.cpp`](src/mission/qr_dive_mode.cpp) - QR dive logic
- [`mission/ardupilot_mode.cpp`](src/mission/ardupilot_mode.cpp) - Passthrough
- [`mission/mission_director.cpp`](src/mission/mission_director.cpp) - Mode transitions

**Vision:**
- [`vision/tracking_controller.cpp`](src/vision/tracking_controller.cpp) - Object tracking

**App:**
- [`app/application.cpp`](src/app/application.cpp) - Main application (588 lines)
- [`main.cpp`](src/main.cpp) - Entry point

---

## 15. ÖZET

**TEKNOFEST UAV v10** 10 modülden oluşan, event-driven, multi-threaded bir otonom uçuş sistemidir:

1. **Core Module:** Temel veri yapıları ve coğrafi hesaplamalar
2. **Config Module:** YAML tabanlı konfigürasyon yönetimi
3. **Infra Module:** Log, event bus, thread pool
4. **World Model:** Tüm sistem durumunu tutan merkezi depo
5. **Comm Module:** MAVLink iletişim ve komut gönderme
6. **Safety Module:** Çok katmanlı güvenlik sistemi
7. **Nav Module:** Visibility Graph + Dijkstra path planning
8. **Mission Module:** 4 farklı mission modu (PATROL, FOLLOW, QR_DIVE, ARDUPILOT)
9. **Vision Module:** Nesne takibi (hazır ama kullanılmıyor)
10. **App Module:** Ana uygulama ve koordinasyon

**Key Features:**
- GUIDED mode only (RC override yok)
- Visibility Graph + Dijkstra path planning
- Proportional Navigation (Follow mode)
- Multi-layer safety system
- Event-driven architecture
- 5-thread yapısı (main, safety, heartbeat, event, comm)
- STATUSTEXT protocol parsing (RIV, HSS, BND, MODE commands)

**Context Kaybını Önlemek İçin:**
Bu döküman tüm sistemi kapsadığı için, gelecekteki geliştirmelerde context kaybı yaşanmadan sistemin her parçası hakkında referans bulunabilir.

---

*Döküman sonuna kadar tamamlandı.*