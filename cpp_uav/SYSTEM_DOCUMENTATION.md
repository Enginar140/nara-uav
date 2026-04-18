# TEKNOFEST UAV v10 - SİSTEM DOKÜMANTASYONU

===================================================================================================
1. SİSTEM GENEL BAKIŞ
===================================================================================================

Proje Adı      : TEKNOFEST UAV v10
Dosya Sayısı   : ~56 dosya (28 header + 28 source)
Dil            : C++17
Build          : CMake
Bağımlılıklar  : yaml-cpp, spdlog, mavlink, eigen, nlohmann/json

--------------------------------------------------------------------
1.1 MİMARİ YAPI (10 Modül)
--------------------------------------------------------------------

core → config → infra → world_model → comm → safety → nav → mission → vision → app

| Modül | Açıklama | Ana Sınıflar |
|-------|----------|--------------|
| core | Temel veri yapıları, geo utils | Position, UAVState, HSSZone, Boundary, MissionMode |
| config | YAML konfigürasyon yönetimi | ConfigManager |
| infra | Altyapı (log, event bus, thread pool) | LogManager, EventBus, ThreadPool |
| world_model | Dünya durumunu tutar (kendi UAV, rakipler, HSS, boundary) | WorldModel |
| comm | MAVLink iletişimi, uçuş komutları | MAVLinkBridge, FlightCommander, ConnectionManager |
| safety | Güvenlik kontrolleri, escape yönetimi | SafetyMonitor, SafetyGate, ZoneChecker, EscapeManager |
| nav | Path planning, waypoint yönetimi | PathPlanner, NavigationEngine, WaypointChain, SpeedManager |
| mission | Mission modları (PATROL, FOLLOW, QR_DIVE) | MissionDirector, PatrolMode, FollowMode, QRDiveMode |
| vision | (Gelecek) Nesne takibi | TrackingController |
| app | Ana uygulama, başlatma, main loop | Application |

--------------------------------------------------------------------
1.2 KOMUT MODELİ (KRİTİK!)
--------------------------------------------------------------------

TÜM KOMUTLAR GUIDED MODDA ÇALIŞIR - RC OVERRIDE YOK!

1. GUIDED GLOBAL: DO_REPOSITION (uzun menzil navigasyon)
   - Parametreler: lat (1e7), lon (1e7), alt (m), airspeed (m/s)
   - Kullanım: planRoute(), flyDirect()

2. GUIDED LOCAL: SET_POSITION_TARGET_LOCAL_NED (hassas kontrol)
   - Position mask: pos xyz kullan, vel xyz ignore
   - Velocity mask: vel xyz kullan, pos xyz ignore
   - Kullanım: flyLocalPosition(), flyLocalVelocity()

--------------------------------------------------------------------
1.3 PATH PLANNING STRATEJİSİ
--------------------------------------------------------------------

Algoritma: Visibility Graph + Dubins Smoothing

1. HSS engelleri → Polygon expansion (circle → 12 segment)
2. Start + Goal + tüm polygon vertices → Visibility Graph
3. Dijkstra ile en kısa yol
4. Path simplification (redundant waypoint silme)
5. Turn smoothing (gerçekçi uçuş yolu)

AVANTAJLAR:
- Minimal memory usage
- Optimal geometric path
- Hızlı planning

DEZAVANTAJLAR:
- Karmaşık geometric hesaplamalar
- Path sampling yok (v9.1'de var)
- Boundary case'ler için ekstra dikkat gerekli

===================================================================================================
2. MODÜL DETAYLARI
===================================================================================================

--------------------------------------------------------------------
2.1 CORE MODULE (core/)
--------------------------------------------------------------------

Dosyalar:
- types.hpp/cpp      : Temel veri yapıları
- geo.hpp/cpp        : Coğrafi hesaplamalar
- time_utils.hpp     : Zaman utility fonksiyonları

Key Structs:
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
    uint8_t gps_fix_type;
    uint8_t satellites_visible;
    uint16_t gps_hdop;
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

enum class MissionMode {
    ARDUPILOT, PATROL, FOLLOW, QR_DIVE
};

enum class RiskLevel {
    NONE, LOW, MEDIUM, HIGH, CRITICAL
};

enum class BoundaryStatus {
    INSIDE, NEAR_BOUNDARY, OUTSIDE
};
```

Geo Functions (geo.hpp):
```cpp
// Mesafe (3D - haversine + altitude)
double haversineDistance(const Position& a, const Position& b);

// Açı [0, 360)
double bearing(const Position& from, const Position& to);

// Belirli bir bearing ve mesafedeki nokta
Position destinationPoint(const Position& from, double bearing_deg, double distance_m);

// Polygon içinde mi?
bool isPointInPolygon(const Position& point, const vector<Position>& polygon);

// Segment-polygon mesafesi
double distanceToSegment(const Position& point, const Position& seg_a, const Position& seg_b);

// Segment-daire kesişimi
bool segmentIntersectsCircle(const Position& p1, const Position& p2, 
                             const Position& center, double radius);

// İki segment kesişir mi?
bool segmentsIntersect(const Position& a1, const Position& a2,
                       const Position& b1, const Position& b2);

// Position → NED (North-East-Down)
NED positionToNED(const Position& pos, const Position& reference);

// NED → Position
Position nedToPosition(const NED& ned, const Position& reference);

// Açı normalizasyonu
double normalizeAngle(double degrees);        // [0, 360)
double angleDifference(double a, double b);   // [-180, 180]
```

--------------------------------------------------------------------
2.2 CONFIG MODULE (config/)
--------------------------------------------------------------------

ConfigManager - YAML konfigürasyon okuma:
```cpp
class ConfigManager {
public:
    explicit ConfigManager(string config_path);
    
    string getString(string key, string default = "") const;
    int getInt(string key, int default = 0) const;
    double getDouble(string key, double default = 0.0) const;
    bool getBool(string key, bool default = false) const;
    bool hasKey(string key) const;
    YAML::Node getNode(string key) const;
    
    // Convenience methods
    string getPixhawkConnection() const;  // "mavlink.pixhawk.connection"
    string getGCSConnection() const;      // "mavlink.gcs.connection"
};
```

Config Structure (default.yaml):
```yaml
system:
  thread_pool_size: 4
  opponent_stale_timeout_s: 10.0

mavlink:
  pixhawk:
    connection: "udpin:127.0.0.1:14550"
    heartbeat_timeout: 5.0
  gcs:
    connection: "tcpin:127.0.0.1:14552"

safety:
  monitoring_rate_hz: 20.0
  hss_buffer_m: 40.0
  boundary_buffer_m: 20.0
  min_altitude_m: 30.0
  max_altitude_m: 400.0
  max_velocity_ms: 40.0
  collision_critical_m: 30.0
  collision_high_m: 80.0
  collision_medium_m: 200.0
  collision_low_m: 500.0
  ttc_critical_s: 5.0
  ttc_high_s: 10.0
  boundary_warn_m: 30.0

nav:
  planner:
    hss_buffer_m: 40.0
    min_turn_radius_m: 50.0
    polygon_segments: 12
  waypoint:
    acceptance_radius_m: 80.0
  speed:
    cruise: 20.0
    max: 35.0
    min: 12.0
    patrol: 18.0
    pursuit: 30.0
    dive: 25.0
  command:
    min_interval_ms: 250
    min_plan_interval_ms: 2000

mission:
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
  patrol:
    boundary_inset_m: 50.0
    cruise_altitude_m: 100.0
```

--------------------------------------------------------------------
2.3 INFRA MODULE (infra/)
--------------------------------------------------------------------

EventBus - Event-driven architecture:
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

struct Event {
    EventType type;
    any payload;
    uint64_t timestamp_us;
    
    template<T> T getPayload() const;
    template<T> bool hasPayload() const;
};

class EventBus {
    uint64_t subscribe(EventType type, EventHandler handler);
    void unsubscribe(uint64_t id);
    void publish(EventType type, any payload = {});
    void start();   // Dispatch thread başlatır
    void stop();
};
```

LogManager - Singleton logger:
```cpp
class LogManager {
    static LogManager& instance();
    void initialize(string dir, string level, int max_size_mb, int backup_count);
    void flush();
};

// Macros
LOG_DEBUG("{}", msg);
LOG_INFO("{}", msg);
LOG_WARN("{}", msg);
LOG_ERROR("{}", msg);
```

ThreadPool - Task paralelleştirme:
```cpp
class ThreadPool {
    explicit ThreadPool(int num_threads);
    template<F> future<result_of<F>> submit(F task);
    void waitAll();
    Stats getStats() const;
};
```

--------------------------------------------------------------------
2.4 WORLD MODEL MODULE (world_model/)
--------------------------------------------------------------------

WorldModel - Thread-safe dünya durumu:
```cpp
class WorldModel {
public:
    // Own UAV state
    void setOwnState(const UAVState& state);
    UAVState getOwnState() const;
    
    // Opponents (rakipler)
    void updateOpponent(const UAVState& state);
    void removeOpponent(int id);
    optional<UAVState> getOpponent(int id) const;
    vector<UAVState> getAllOpponents() const;
    optional<UAVState> getNearestOpponent(const Position& pos) const;
    void removeStaleOpponents(double max_age_s);
    
    // HSS zones (yasak bölgeler)
    void updateHSSZone(const HSSZone& zone);
    void removeHSSZone(int id);
    vector<HSSZone> getActiveHSSZones() const;
    
    // Boundary (yarışma alanı sınırları)
    void setBoundary(const Boundary& boundary);
    optional<Boundary> getBoundary() const;
    bool hasBoundary() const;

private:
    // Tüm data shared_mutex ile korunuyor (reader-writer locks)
    UAVState own_state_;
    mutable shared_mutex own_mutex_;
    
    unordered_map<int, UAVState> opponents_;
    mutable shared_mutex opp_mutex_;
    
    unordered_map<int, HSSZone> hss_zones_;
    mutable shared_mutex hss_mutex_;
    
    optional<Boundary> boundary_;
    mutable shared_mutex bnd_mutex_;
};
```

--------------------------------------------------------------------
2.5 COMM MODULE (comm/)
--------------------------------------------------------------------

ArduPilotMode Enum:
```cpp
enum class ArduPilotMode : uint32_t {
    MANUAL=0, CIRCLE=1, STABILIZE=2, TRAINING=3, ACRO=4, FBWA=5,
    FBWB=6, CRUISE=7, AUTOTUNE=8, AUTO=10, RTL=11, LOITER=12,
    TAKEOFF=13, AVOID_ADSB=14, GUIDED=15, INITIALISING=16
};
```

FlightCommander - Uçuş komutları:
```cpp
class FlightCommander {
public:
    // GUIDED GLOBAL: DO_REPOSITION
    bool flyToPosition(const Position& target, float airspeed = -1.0f);
    
    // GUIDED LOCAL: SET_POSITION_TARGET_LOCAL_NED
    bool flyToLocalPosition(float north_m, float east_m, float down_m);
    bool setLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);
    
    // Mode control
    bool setMode(ArduPilotMode mode);
    bool arm();
    bool disarm();
    
    ArduPilotMode getCurrentMode() const;
    bool isInGuidedMode() const;
    bool isArmed() const;
    
    void updateModeFromHeartbeat(uint32_t custom_mode);
    void updateArmedFromHeartbeat(uint8_t base_mode);
    void handleCommandAck(uint16_t cmd_id, uint8_t result);
    
private:
    shared_ptr<ConnectionManager> conn_;
    atomic<ArduPilotMode> current_mode_;
    atomic<bool> armed_;
    atomic<uint64_t> commands_sent_;
    atomic<uint64_t> commands_acked_;
    Position last_position_cmd_;  // Duplicate detection için
};
```

MAVLinkBridge - MAVLink iletişim yönetimi:
```cpp
class MAVLinkBridge {
public:
    MAVLinkBridge(string pixhawk_conn, string gcs_conn,
                  shared_ptr<ThreadPool>, shared_ptr<WorldModel>,
                  shared_ptr<EventBus>);
    
    bool start();
    void stop();
    bool isRunning() const;
    
    shared_ptr<FlightCommander> getFlightCommander();
    shared_ptr<ParamReader> getParamReader();
    Stats getStats() const;
    
private:
    shared_ptr<ConnectionManager> pixhawk_;
    shared_ptr<ConnectionManager> gcs_;
    shared_ptr<FlightCommander> flight_commander_;
    shared_ptr<ParamReader> param_reader_;
    StatusTextParser statustext_parser_;
    
    thread heartbeat_thread_;
    
    void handlePixhawkMessage(const MAVLinkMessage& msg);
    void handleGCSMessage(const MAVLinkMessage& msg);
    void handleHeartbeat(const MAVLinkMessage& msg);
    void handleStatusText(const MAVLinkMessage& msg);
    void handleCommandAck(const MAVLinkMessage& msg);
    void heartbeatLoop();
    
    void dispatchModeCommand(const ParsedStatusText& parsed);
};
```

STATUSTEXT Mesaj Formatları:
```
RIV: RIV,id,lat,lon,alt,vN,vE,vD,hdg,spd
HSS: HSS,id,lat,lon,alt,radius,active
BND: BND,lat1,lon1,lat2,lon2,...
MODE CMD: FREE, LOCK id, ARDUPILOT, KAMIKAZE lat,lon,alt
```

--------------------------------------------------------------------
2.6 SAFETY MODULE (safety/)
--------------------------------------------------------------------

ZoneChecker - Güvenlik bölgesi kontrolleri:
```cpp
class ZoneChecker {
public:
    bool isInsideBoundary(const Position& pos, const Boundary& boundary) const;
    bool isInsideAnyHSS(const Position& pos, const vector<HSSZone>& zones,
                        double extra_buffer = 0.0) const;
    double distanceToBoundary(const Position& pos, const Boundary& boundary) const;
    double distanceToNearestHSS(const Position& pos, const vector<HSSZone>& zones) const;
    bool segmentIntersectsAnyHSS(const Position& from, const Position& to,
                                 const vector<HSSZone>& zones, double buffer = 0.0) const;
    bool segmentExitsBoundary(const Position& from, const Position& to,
                              const Boundary& boundary) const;
    bool isPositionSafe(const Position& pos, const Boundary& boundary,
                        const vector<HSSZone>& zones, double hss_buffer = 40.0) const;
};
```

SafetyGate - Komut güvenlik kapısı (last line of defense):
```cpp
class SafetyGate {
public:
    struct Config {
        double hss_buffer_m = 40.0;
        double boundary_buffer_m = 20.0;
        double min_altitude_m = 30.0;
        double max_altitude_m = 400.0;
        double max_velocity_ms = 40.0;
    };
    
    bool validatePosition(const Position& target) const;
    bool validateVelocity(float vn, float ve, float vd) const;
    
    Position clampToSafe(const Position& target) const;
    void clampVelocity(float& vn, float& ve, float& vd) const;
};
```

SafetyMonitor - 20Hz sürekli güvenlik kontrolü:
```cpp
class SafetyMonitor {
public:
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
    
    bool start();   // Monitor thread başlatır
    void stop();
    SafetyState getSafetyState() const;
    void setMonitoringRate(double hz);
    
    void notifyHeartbeat();
    void notifyHeartbeatLost();
    
private:
    void monitorLoop();
    void checkBoundary(const UAVState& own, const Boundary& boundary);
    void checkHSS(const UAVState& own, const vector<HSSZone>& zones);
    void checkCollisions(const UAVState& own, const vector<UAVState>& opponents);
    void checkGPS(const UAVState& own);
    void checkBattery(const UAVState& own);
    void checkComm();
};
```

EscapeManager - Acil kaçış yönetimi:
```cpp
class EscapeManager {
public:
    optional<Position> generateEscapeTarget(const UAVState& own) const;
    optional<Position> generateBoundaryEscape(const UAVState& own,
                                              const Boundary& bnd) const;
    optional<Position> generateHSSEscape(const UAVState& own,
                                         const vector<HSSZone>& zones) const;
    optional<Position> generateCollisionEscape(const UAVState& own,
                                               const Threat& threat) const;
    
private:
    optional<Position> validateOrFindAlternative(
        const Position& candidate, const Position& own_pos,
        double base_bearing, double escape_dist) const;
};
```

Escape Stratejileri:
- Boundary escape: Boundary merkezine doğru
- HSS escape: HSS merkezinden uzakla
- Collision escape: Threat bearing + 90° (veya heading + 45°)

--------------------------------------------------------------------
2.7 NAV MODULE (nav/)
--------------------------------------------------------------------

PathPlanner - Visibility Graph + Dubins:
```cpp
class PathPlanner {
public:
    struct Config {
        double hss_buffer_m = 40.0;
        double boundary_buffer_m = 20.0;
        double min_turn_radius_m = 50.0;
        int polygon_segments = 12;
    };
    
    struct PathResult {
        bool success;
        vector<Position> waypoints;
        double total_distance_m;
        double planning_time_ms;
        string error;
    };
    
    PathResult plan(const Position& start, const Position& goal,
                    const Boundary& boundary, const vector<HSSZone>& hss_zones);
    
    bool isDirectPathClear(const Position& from, const Position& to,
                           const Boundary& boundary, const vector<HSSZone>& zones) const;
    
private:
    // Obstacle expansion: HSS circles → convex polygons
    vector<vector<Position>> expandObstacles(const vector<HSSZone>& zones) const;
    vector<Position> circleToPolygon(const Position& center, double radius) const;
    
    // Visibility graph
    struct VNode { Position pos; int obstacle_id; };
    vector<VNode> buildNodes(const Position& start, const Position& goal,
                             const vector<vector<Position>>& obstacles) const;
    AdjList buildAdjacency(const vector<VNode>& nodes,
                           const vector<vector<Position>>& obstacles,
                           const Boundary& boundary) const;
    vector<int> dijkstra(const AdjList& adj, int src, int dst) const;
    
    // Path optimization
    vector<Position> simplifyPath(const vector<Position>& path,
                                   const vector<vector<Position>>& obstacles,
                                   const Boundary& boundary) const;
    vector<Position> smoothTurns(const vector<Position>& path,
                                 const Boundary& boundary,
                                 const vector<HSSZone>& hss_zones) const;
    
    // Visibility check
    bool isEdgeVisible(const Position& a, const Position& b,
                       const vector<vector<Position>>& obstacles,
                       const Boundary& boundary,
                       int skip_obs_a = -1, int skip_obs_b = -1) const;
};
```

WaypointChain - Waypoint yönetimi:
```cpp
struct Waypoint {
    Position position;
    float airspeed = -1.0f;
    double acceptance_radius_m = 80.0;
    string label;
};

enum class ChainStatus { EMPTY, IN_PROGRESS, WAYPOINT_REACHED, CHAIN_COMPLETE };

class WaypointChain {
public:
    void load(vector<Waypoint> chain, bool circular = false);
    ChainStatus update(const Position& own_pos);
    optional<Waypoint> currentTarget() const;
    optional<Waypoint> peekNext() const;
    vector<Waypoint> remainingWaypoints() const;
    
    bool isEmpty() const;
    bool isComplete() const;
    size_t currentIndex() const;
    size_t size() const;
    void clear();
    
    void replace(vector<Waypoint> new_chain);  // Smart replace
    
    bool validate(const ZoneChecker& checker, const Boundary& boundary,
                  const vector<HSSZone>& zones, double hss_buffer = 40.0) const;
};
```

SpeedManager - Hız yönetimi:
```cpp
class SpeedManager {
public:
    struct Config {
        float cruise = 20.0f;
        float max_speed = 35.0f;
        float min_speed = 12.0f;
        float patrol = 18.0f;
        float pursuit = 30.0f;
        float dive = 25.0f;
    };
    
    float cruiseSpeed() const;
    float patrolSpeed() const;
    float pursuitSpeed() const;
    float diveSpeed() const;
    float clamp(float speed) const;
};
```

NavigationEngine - Navigation facade (PathPlanner + WaypointChain + FlightCommander):
```cpp
class NavigationEngine {
public:
    NavigationEngine(shared_ptr<PathPlanner> planner,
                     shared_ptr<FlightCommander> commander,
                     shared_ptr<SafetyGate> safety_gate,
                     shared_ptr<SpeedManager> speed_manager,
                     shared_ptr<WorldModel> world_model);
    
    // HSS-safe route planla ve waypoint chain yükle
    bool planRoute(const Position& goal, float airspeed = -1.0f);
    
    // Chain progress update, waypoint ulaşıldıysa sonraki komut gönder
    ChainStatus update(const Position& own_pos);
    
    // Direkt komutlar (chain bypass - pursuit/dive için)
    bool flyDirect(const Position& target, float airspeed = -1.0f);
    bool flyLocalPosition(float north, float east, float down);
    bool flyLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);
    
    // Route yönetimi
    void clearRoute();
    bool hasActiveRoute() const;
    bool revalidateRoute();
    
    // Rate limiting
    void setMinCommandInterval(chrono::milliseconds ms);
    void setMinPlanInterval(chrono::milliseconds ms);
    
private:
    shared_ptr<PathPlanner> planner_;
    shared_ptr<FlightCommander> commander_;
    shared_ptr<SafetyGate> safety_gate_;
    shared_ptr<SpeedManager> speed_manager_;
    shared_ptr<WorldModel> world_model_;
    
    WaypointChain chain_;
    
    chrono::steady_clock::time_point last_cmd_time_;
    chrono::steady_clock::time_point last_plan_time_;
    chrono::milliseconds min_cmd_interval_{250};
    chrono::milliseconds min_plan_interval_{2000};
    optional<Position> last_sent_target_;
    
    bool sendCurrentWaypoint();
    bool shouldThrottleCommand() const;
};
```

--------------------------------------------------------------------
2.8 MISSION MODULE (mission/)
--------------------------------------------------------------------

IMissionMode Interface:
```cpp
class IMissionMode {
public:
    virtual void onEnter() = 0;
    virtual void onExit() = 0;
    virtual void tick(const UAVState& own_state) = 0;
    virtual MissionGoal getMissionGoal() = 0;
    virtual string getName() const = 0;
    virtual ~IMissionMode() = default;
};
```

PatrolMode - Boundary devriyesi:
```cpp
class PatrolMode : public IMissionMode {
public:
    struct Config {
        double boundary_inset_m = 50.0;
        double cruise_altitude_m = 100.0;
        float patrol_speed_ms = 18.0f;
    };
    
    void onEnter() override;    // Patrol waypoints generate et ve chain yükle
    void onExit() override;     // Route temizle
    void tick(const UAVState&) override;  // Chain update, gerekirse regenerate
    
private:
    void generatePatrolWaypoints();  // Boundary inset
    void planToNextWaypoint();
    
    vector<Position> patrol_waypoints_;
    size_t current_patrol_idx_;
    bool waypoints_generated_;
};
```

FollowMode - Rakip takibi (Proportional Navigation):
```cpp
enum class FollowPhase { APPROACH, PURSUIT, REACQUIRE };

class FollowMode : public IMissionMode {
public:
    struct Config {
        double pursuit_range_m = 200.0;
        double reacquire_timeout_s = 5.0;
        double max_reacquire_s = 30.0;
        double prediction_horizon_s = 3.0;
        double target_stale_s = 3.0;
        double plan_update_interval_s = 3.0;
        float approach_speed_ms = 20.0f;
        float pursuit_speed_ms = 30.0f;
        double pn_gain = 3.0;           // Proportional Navigation gain
        double pn_max_lead_deg = 45.0;  // Maksimum lead açısı
    };
    
    void setTargetId(int id);
    FollowPhase getPhase() const;
    bool isTargetLost() const;
    void setTargetLostCallback(function<void()> cb);
    
private:
    void tickApproach(const UAVState& own, const UAVState& target);
    void tickPursuit(const UAVState& own, const UAVState& target);
    void tickReacquire(const UAVState& own);
    
    // Basit linear prediction
    Position predictTarget(const UAVState& target) const;
    
    // Proportional Navigation ile intercept
    Position predictWithPN(const UAVState& own, const UAVState& target);
    
    int target_id_;
    FollowPhase phase_;
    bool target_permanently_lost_;
    
    // PN için LOS rate tracking
    double prev_los_bearing_;
    steady_clock::time_point prev_los_time_;
    bool los_initialized_;
};
```

Proportional Navigation (PN) Formülü:
```
LOS_rate = (bearing_t - bearing_t-1) / dt
pn_offset = pn_gain * LOS_rate * t_go
intercept_bearing = LOS_bearing + pn_offset
intercept_pos = destinationPoint(own_pos, intercept_bearing, intercept_dist)
```

QRDiveMode - QR hedefe dalış:
```cpp
enum class DivePhase { APPROACH, ALIGN, DIVE, CLIMB, COMPLETED };

class QRDiveMode : public IMissionMode {
public:
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
    
    void setDiveTarget(const Position& target);
    void setQRCallback(QRCallback cb);
    void setCompletionCallback(function<void()> cb);
    
    DivePhase getPhase() const;
    bool isCompleted() const;
    Position getEntryPoint() const;
    
private:
    void calculateEntryPoint();
    void tickApproach(const UAVState& own);
    void tickAlign(const UAVState& own);
    void tickDive(const UAVState& own);
    void tickDivePosition(const UAVState& own);  // Position command ile dive
    void tickDiveVelocity(const UAVState& own);  // Velocity command ile dive
    void tickClimb(const UAVState& own);
    
    optional<Position> dive_target_;
    Position entry_point_;  // Hedefin 500m arkası
    bool entry_valid_;
    DivePhase phase_;
    steady_clock::time_point phase_start_;
    QRDetectionResult last_qr_;
};
```

Dive Stratejisi:
1. APPROACH: Entry point'e (500m arka) git
2. ALIGN: Heading ve altitude align et
3. DIVE: -45° pitch ile dalış (position veya velocity command)
4. CLIMB: 100m'e tırman
5. COMPLETED: Callback çağır → PATROL'a geç

ArduPilotPassthrough - Manuel kontrol modu:
```cpp
class ArduPilotPassthrough : public IMissionMode {
    void onEnter() override;   // ArduPilot'a kontrol ver
    void onExit() override;
    void tick(const UAVState&) override;  // No-op
    MissionGoal getMissionGoal() override;  // NONE
};
```

MissionDirector - Mission yöneticisi:
```cpp
class MissionDirector {
public:
    struct Config {
        PatrolMode::Config patrol;
        FollowMode::Config follow;
        QRDiveMode::Config qr_dive;
    };
    
    void initialize();
    
    bool requestMode(MissionMode mode);
    void forceMode(MissionMode mode);
    MissionMode getCurrentMode() const;
    
    void setFollowTarget(int target_id);
    void setDiveTarget(const Position& target);
    
    void tick();  // Main loop'ta 5Hz çağrılır
    
    void handleModeCommand(const ParsedModeCommand& cmd);
    void triggerRTL(const string& reason);
    
    struct Stats {
        MissionMode current_mode;
        uint64_t mode_transitions;
        string active_phase;  // Follow/Dive sub-phase
    };
    Stats getStats() const;
    
private:
    IMissionMode* activeHandler();
    bool validateTransition(MissionMode from, MissionMode to) const;
    void executeTransition(MissionMode from, MissionMode to);
    
    unique_ptr<PatrolMode> patrol_;
    unique_ptr<FollowMode> follow_;
    unique_ptr<QRDiveMode> qr_dive_;
    unique_ptr<ArduPilotPassthrough> ardupilot_;
    
    MissionMode current_mode_;
    uint64_t transition_count_;
};
```

Mode Transition Kuralları:
```
ARDUPILOT → HERŞEY: İzin verili (operator override)
HERŞEY → ARDUPILOT: İzin verili (safety fallback)
QR_DIVE → SADECE PATROL: Tamamlanmadan çıkılamaz
FOLLOW → PATROL/QR_DIVE: İzin verili
PATROL → HERŞEY: İzin verili
```

--------------------------------------------------------------------
2.9 APP MODULE (app/)
--------------------------------------------------------------------

Application - Ana uygulama sınıfı:
```cpp
class Application {
public:
    explicit Application(const string& config_path);
    ~Application();
    
    bool initialize();
    bool start();
    void run();      // Main loop - blocks until shutdown
    void stop();
    void requestShutdown();
    
    static void setupSignalHandlers();
    
private:
    // Initialization
    bool initCore();     // config, event_bus, thread_pool, world_model
    bool initComm();     // mavlink_bridge, flight_commander
    bool initSafety();   // zone_checker, safety_gate, safety_monitor
    bool initNav();      // path_planner, speed_manager, navigation_engine
    bool initMission();  // mission_director + all modes
    
    void subscribeEvents();
    void loadBoundaryFromConfig();
    void loadHSSFromConfig();
    
    // Main loop
    void mainLoop();     // 5Hz: mission.tick(), dashboard, safety check
    void printDashboard() const;
    
    // Failsafe
    void handleFailsafe(const Event& event);
    void handleEscapeNeeded(const SafetyState& safety);
    void triggerRTL(const string& reason);
    void applyArduPilotParams();  // Param sync
    
    // Components
    shared_ptr<ConfigManager> config_;
    shared_ptr<EventBus> event_bus_;
    shared_ptr<ThreadPool> thread_pool_;
    shared_ptr<WorldModel> world_model_;
    
    shared_ptr<MAVLinkBridge> mavlink_;
    shared_ptr<FlightCommander> commander_;
    
    shared_ptr<ZoneChecker> zone_checker_;
    shared_ptr<SafetyGate> safety_gate_;
    shared_ptr<SafetyMonitor> safety_monitor_;
    shared_ptr<EscapeManager> escape_manager_;
    
    shared_ptr<PathPlanner> planner_;
    shared_ptr<SpeedManager> speed_manager_;
    shared_ptr<NavigationEngine> nav_engine_;
    
    shared_ptr<MissionDirector> mission_;
};
```

Main Loop (5Hz):
```cpp
void Application::mainLoop() {
    constexpr int LOOP_HZ = 5;
    while (running_ && !shutdown_requested_) {
        mission_->tick();  // Aktif mission modunun tick()'i
        
        // 2 saniyede bir stale opponent temizle
        if (now - last_cleanup >= 2s) {
            world_model_->removeStaleOpponents(10.0s);
        }
        
        // 2 saniyede bir dashboard yaz
        if (now - last_dashboard >= 2s) {
            printDashboard();
        }
        
        sleep_until(next_tick);
    }
}
```

Event Subscriptions:
```cpp
GPS_QUALITY_CRITICAL  → triggerRTL("gps_critical")
BATTERY_CRITICAL      → triggerRTL("battery_critical")
HEARTBEAT_TIMEOUT     → triggerRTL("heartbeat_timeout")
HEARTBEAT_RECOVERED   → safety_monitor_->notifyHeartbeat()
PARAMS_READY          → applyArduPilotParams()
HSS_ZONE_UPDATED      → nav_engine_->revalidateRoute()
BOUNDARY_UPDATED      → nav_engine_->revalidateRoute()
MODE_COMMAND_RECEIVED → mission_->handleModeCommand()
SAFETY_VIOLATION      → handleEscapeNeeded()
COLLISION_WARNING     → escape_manager_->generateCollisionEscape()
```

===================================================================================================
3. DATA FLOW
===================================================================================================

--------------------------------------------------------------------
3.1 MAVLink DATA FLOW
--------------------------------------------------------------------

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          MAVLINK MESSAGE FLOW                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Pixhawk ────────> ConnectionManager ────> MAVLinkBridge ────> WorldModel  │
│  (UDP:14550)                              │                    (setOwnState)│
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

--------------------------------------------------------------------
3.2 COMMAND FLOW (NavigationEngine → ArduPilot)
--------------------------------------------------------------------

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
    │       ├──> expandObstacles() ──> circles → polygons (12 segment)
    │       ├──> buildNodes() ──> start + goal + polygon vertices
    │       ├──> buildAdjacency() ──> visibility edges
    │       ├──> dijkstra() ──> shortest path
    │       ├──> simplifyPath() ──> remove redundant waypoints
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

--------------------------------------------------------------------
3.3 MISSION FLOW (MissionDirector.tick())
--------------------------------------------------------------------

```
MissionDirector.tick()  [5Hz]
    │
    ├──> WorldModel.getOwnState()
    ├──> FlightCommander.isInGuidedMode() ──> retry if not
    │
    └──> activeHandler().tick(own_state)
            │
            ├── PATROL:
            │   ├──> generatePatrolWaypoints() ──> boundary inset
            │   ├──> planToNextWaypoint() ──> NavigationEngine.planRoute()
            │   └──> NavigationEngine.update() ──> waypoint chain advance
            │
            ├── FOLLOW:
            │   ├──> WorldModel.getOpponent(target_id)
            │   │       NOT FOUND → tickReacquire()
            │   │       FOUND     → continue
            │   │
            │   ├── APPROACH (dist > 200m):
            │   │   ├──> predictTarget() ──> linear prediction
            │   │   └──> NavigationEngine.planRoute(predicted)
            │   │
            │   ├── PURSUIT (dist <= 200m):
            │   │   ├──> predictWithPN() ──> Proportional Navigation
            │   │   └──> NavigationEngine.flyDirect(intercept)
            │   │
            │   └── REACQUIRE (target lost):
            │       ├──> wait max_reacquire_s (30s)
            │       └──> timeout → target_permanently_lost → PATROL
            │
            └── QR_DIVE:
                ├── APPROACH: nav_->planRoute(entry_point)
                ├── ALIGN:    nav_->flyDirect(align_target)
                ├── DIVE:     nav_->flyDirect(step_pos) or flyLocalVelocity()
                ├── CLIMB:    nav_->flyDirect(climb_pos)
                └── COMPLETED → callback → PATROL
```

4. THREAD MODELİ

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
│  ├── checkHSS()                                             │
│  ├── checkCollisions()                                      │
│  ├── checkGPS()                                             │
│  ├── checkBattery()                                         │
│  ├── checkComm()                                            │
│  └── publishIfChanged() ──> EventBus                        │
│                                                              │
│  MAVLink Heartbeat Thread [1Hz]                             │
│  ├── Send heartbeat to Pixhawk                              │
│  ├── Send heartbeat to GCS                                  │
│  ├── Check heartbeat timeout (5s)                           │
│  ├── Auto-reconnect Pixhawk (backoff: 1s → 30s)            │
│  └── Auto-reconnect GCS                                     │
│                                                              │
│  MAVLink Receive Thread (ConnectionManager internal)        │
│  ├── UDP/TCP/Serial polling                                 │
│  └── Message callback ──> MAVLinkBridge.handlePixhawkMsg()  │
│                                                              │
│  EventBus Dispatch Thread                                    │
│  ├── Queue-based event dispatch                             │
│  └── Subscriber callbacks (sync, in dispatch thread)        │
│                                                              │
│  ThreadPool (4 workers)                                      │
│  ├── ParamReader.requestCritical() [delayed 2s]            │
│  └── ParamReader.requestAll() [delayed 7s]                 │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

5. BAŞLATMA SIRASI (INITIALIZATION)

```
Application.initialize()
    │
    ├── 1. initCore()
    │   ├── ConfigManager(config_path) ──> YAML yükle
    │   ├── LogManager.initialize() ──> spdlog setup
    │   ├── EventBus() ──> event system
    │   ├── ThreadPool(4) ──> worker threads
    │   └── WorldModel() ──> dünya durumu (boş)
    │
    ├── 2. initComm()
    │   ├── MAVLinkBridge(pixhawk, gcs, ...)
    │   │   ├── ConnectionManager(UDP:14550) ──> Pixhawk
    │   │   ├── ConnectionManager(TCP:14552) ──> GCS
    │   │   ├── FlightCommander(pixhawk_conn)
    │   │   ├── ParamReader(pixhawk_conn)
    │   │   └── StatusTextParser()
    │   └── commander_ = bridge.getFlightCommander()
    │
    ├── 3. initSafety()
    │   ├── ZoneChecker() ──> bölge kontrol utility
    │   ├── SafetyGate(zone_checker, world_model, config)
    │   ├── SafetyMonitor(world_model, event_bus, config)
    │   └── EscapeManager(zone_checker, world_model, safety_gate)
    │
    ├── 4. initNav()
    │   ├── PathPlanner(zone_checker, config)
    │   ├── SpeedManager(config)
    │   └── NavigationEngine(planner, commander, safety_gate, speed_manager, world_model)
    │
    ├── 5. initMission()
    │   ├── MissionDirector(nav, commander, world_model, event_bus, config)
    │   │   ├── ArduPilotPassthrough(commander)
    │   │   ├── PatrolMode(nav, world_model, config)
    │   │   ├── FollowMode(nav, world_model, config)
    │   │   └── QRDiveMode(nav, world_model, commander, config)
    │   ├── qr_dive_->setCompletionCallback(→ PATROL)
    │   ├── follow_->setTargetLostCallback(→ PATROL)
    │   └── ardupilot_->onEnter() ──> başlangıç modu
    │
    ├── 6. subscribeEvents() ──> event handler kayıtları
    ├── 7. loadBoundaryFromConfig() ──> YAML'den boundary yükle
    └── 8. loadHSSFromConfig() ──> YAML'den HSS zones yükle

Application.start()
    ├── MAVLinkBridge.start()
    │   ├── Pixhawk connect + callback
    │   ├── GCS connect + callback (non-critical)
    │   ├── Heartbeat thread başlat
    │   └── ParamReader.requestCritical() (2s delay)
    ├── SafetyMonitor.start() ──> monitor thread başlat
    ├── EventBus.start() ──> dispatch thread başlat
    └── publish(SYSTEM_STARTED)
```

6. FAILSAFE SİSTEMİ

```
┌─────────────────────────────────────────────────────────────┐
│                    FAILSAFE HIERARCHY                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Level 1: SafetyGate (her komutta)                          │
│  ├── validatePosition() → clampToSafe()                     │
│  └── validateVelocity() → clampVelocity()                   │
│                                                              │
│  Level 2: SafetyMonitor (20Hz thread)                       │
│  ├── Boundary violation → SAFETY_VIOLATION event            │
│  ├── HSS violation → HSS_VIOLATION event                    │
│  ├── Collision risk → COLLISION_WARNING event               │
│  ├── GPS critical → GPS_QUALITY_CRITICAL event              │
│  ├── Battery critical → BATTERY_CRITICAL event              │
│  └── Comm loss → MAVLINK_DISCONNECTED event                 │
│                                                              │
│  Level 3: Application (event handler)                       │
│  ├── GPS_QUALITY_CRITICAL → triggerRTL("gps_critical")      │
│  ├── BATTERY_CRITICAL → triggerRTL("battery_critical")      │
│  ├── HEARTBEAT_TIMEOUT → triggerRTL("heartbeat_timeout")    │
│  ├── SAFETY_VIOLATION:                                      │
│  │   ├── boundary outside → boundary escape                 │
│  │   ├── HSS violation → HSS escape                         │
│  │   └── collision critical → collision escape              │
│  └── COLLISION_WARNING (CRITICAL) → collision escape        │
│                                                              │
│  Level 4: MissionDirector.triggerRTL()                      │
│  ├── Active handler.onExit()                                │
│  ├── current_mode_ = ARDUPILOT                              │
│  ├── ArduPilotPassthrough.onEnter()                         │
│  └── FlightCommander.setMode(RTL)                           │
│                                                              │
│  Level 5: EscapeManager (acil kaçış)                        │
│  ├── Boundary: merkez yönüne 200m                           │
│  ├── HSS: merkezden uzak +80m                               │
│  ├── Collision: bearing+90° (heading±45°), 150m +20m alt   │
│  └── Alternative search: 30° adımlarla ±150°               │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

7. BİLİNEN SORUNLAR VE NOTLAR

--------------------------------------------------------------------
7.1 PATH PLANNING BUG'LAR (PATH_PLANNING_ANALYSIS.md'den)
--------------------------------------------------------------------

BUG-1: Obstacle Skip Logic (DÜZELTİLDİ)
- Dosya: path_planner.cpp:191
- Sorun: `&&` yerine `||` olmalıydı
- Durum: Düzeltildi

BUG-2: Boundary Check (DÜZELTİLDİ)
- Dosya: path_planner.cpp:214-220
- Sorun: Sadece orta nokta kontrol ediliyordu
- Durum: Düzeltildi

BUG-3: isDirectPathClear Her Zaman True Dönebiliyor (KISMEN DÜZELTİLDİ)
- Sorun: HSS altitude range tanımsız, sampling yok
- Risk: Uzun segment'ler HSS içinden geçebilir

--------------------------------------------------------------------
7.2 EKSİK ÖZELLİKLER (v9.1 ile karşılaştırma)
--------------------------------------------------------------------

1. Path Sampling YOK
   - v9.1: 10 örnekle segment kontrolü
   - v10: Sadece segment intersection check
   - Öneri: isSegmentSafeWithSampling() ekle

2. Altitude-Aware HSS Kontrolü
   - types.hpp'de altitude range tanımlı ama kullanılmıyor
   - HSSZone.min_altitude/max_altitude default INFINITY

3. Goal HSS İçinde Kontrolü
   - Goal HSS içindeyse ne yapılacağı belirsiz
   - Öneri: Goal validation ekle

4. Vision Modülü Boş
   - TrackingController sadece header'da tanımlı
   - QR detection callback mekanizması var ama implement yok

--------------------------------------------------------------------
7.3 ARDUPILOT PARAM SYNC
--------------------------------------------------------------------

PARAMS_READY event'i tetiklendiğinde ArduPilot parametreleri subsystem'lara sync edilir:

1. SpeedManager: stall_speed, max_airspeed, cruise_speed
2. PathPlanner: turn_radius = estimated_r * 1.2
3. SafetyGate: FENCE_ALT_MAX, FENCE_ALT_MIN, max_velocity
4. WP_RADIUS ve LOITER_RADIUS loglanır

Param Reader kritik parametreleri 2s sonra request eder:
- AIRSPEED_CRUISE, AIRSPEED_MAX, STALL_SPEED
- FENCE_ALT_MAX, FENCE_ALT_MIN
- WP_RADIUS, LOITER_RADIUS
- TURN_RADIUS (hesaplanan)

8. DOSYA LİSTESİ

--------------------------------------------------------------------
Headers (include/uav/)
--------------------------------------------------------------------
 1. core/types.hpp           - Temel veri yapıları
 2. core/geo.hpp             - Coğrafi hesaplama fonksiyonları
 3. core/time_utils.hpp      - Zaman utility fonksiyonları
 4. config/config_manager.hpp - YAML konfigürasyon okuma
 5. infra/log_manager.hpp    - Singleton spdlog logger
 6. infra/event_bus.hpp      - Event system (pub/sub)
 7. infra/thread_pool.hpp    - Thread pool
 8. world_model/world_model.hpp - Thread-safe dünya durumu
 9. comm/mavlink_message.hpp - MAVLink mesaj wrapper
10. comm/connection_manager.hpp - UDP/TCP/Serial bağlantı
11. comm/flight_commander.hpp - Uçuş komut gönderimi
12. comm/statustext_parser.hpp - STATUSTEXT mesaj parse
13. comm/mavlink_bridge.hpp  - MAVLink iletişim merkezi
14. comm/param_reader.hpp    - ArduPilot param okuma
15. safety/zone_checker.hpp  - Bölge kontrol utility
16. safety/safety_gate.hpp   - Komut güvenlik kapısı
17. safety/safety_monitor.hpp - 20Hz güvenlik monitörü
18. safety/escape_manager.hpp - Acil kaçış yönetimi
19. nav/path_planner.hpp     - Visibility Graph path planner
20. nav/waypoint_chain.hpp   - Waypoint zincir yönetimi
21. nav/speed_manager.hpp    - Hız yönetimi
22. nav/navigation_engine.hpp - Navigation facade
23. mission/mission_mode.hpp - IMissionMode interface
24. mission/patrol_mode.hpp  - Devriye modu
25. mission/follow_mode.hpp  - Rakip takip modu
26. mission/qr_dive_mode.hpp - QR dalış modu
27. mission/ardupilot_mode.hpp - Manuel kontrol modu
28. mission/mission_director.hpp - Mission yöneticisi
29. vision/tracking_controller.hpp - Nesne takibi (boş)
30. app/application.hpp      - Ana uygulama sınıfı

--------------------------------------------------------------------
Sources (src/)
--------------------------------------------------------------------
 1. core/types.cpp           - toString() fonksiyonları
 2. core/geo.cpp             - Geo hesaplama implementasyonu
 3. config/config_manager.cpp - Config implementasyonu
 4. infra/log_manager.cpp    - Logger implementasyonu
 5. infra/event_bus.cpp      - EventBus implementasyonu
 6. infra/thread_pool.cpp    - ThreadPool implementasyonu
 7. world_model/world_model.cpp - WorldModel implementasyonu
 8. comm/mavlink_message.cpp - MAVLink mesaj
 9. comm/connection_manager.cpp - Bağlantı yönetimi
10. comm/connection_manager_socket.cpp - Platform socket code
11. comm/flight_commander.cpp - Uçuş komutları
12. comm/statustext_parser.cpp - STATUSTEXT parser
13. comm/mavlink_bridge.cpp  - MAVLink bridge
14. comm/param_reader.cpp    - Param reader
15. safety/zone_checker.cpp  - Zone checker
16. safety/safety_gate.cpp   - Safety gate
17. safety/safety_monitor.cpp - Safety monitor
18. safety/escape_manager.cpp - Escape manager
19. nav/path_planner.cpp     - Path planner (434 satır)
20. nav/waypoint_chain.cpp   - Waypoint chain
21. nav/speed_manager.cpp    - Speed manager
22. nav/navigation_engine.cpp - Navigation engine (204 satır)
23. mission/patrol_mode.cpp  - Patrol mode (104 satır)
24. mission/follow_mode.cpp  - Follow mode (197 satır)
25. mission/qr_dive_mode.cpp - QR dive mode (237 satır)
26. mission/ardupilot_mode.cpp - ArduPilot passthrough
27. mission/mission_director.cpp - Mission director (198 satır)
28. vision/tracking_controller.cpp - Tracking (boş)
29. app/application.cpp      - Application (588 satır)
30. main.cpp                 - Giriş noktası (49 satır)

9. ÖNEMLİ PARAMETRE TABLOSU

| Parametre                    | Varsayılan | Açıklama                         |
|------------------------------|------------|----------------------------------|
| safety.hss_buffer_m          | 40.0       | HSS etrafındaki güvenlik tamponu |
| safety.boundary_buffer_m     | 20.0       | Boundary güvenlik tamponu        |
| safety.min_altitude_m        | 30.0       | Minimum uçuş yüksekliği          |
| safety.max_altitude_m        | 400.0      | Maksimum uçuş yüksekliği         |
| safety.collision_critical_m  | 30.0       | Kritik çarpışma mesafesi         |
| safety.collision_high_m      | 80.0       | Yüksek çarpışma mesafesi         |
| safety.ttc_critical_s        | 5.0        | Kritik TTC süresi                |
| nav.planner.hss_buffer_m     | 40.0       | Planner HSS buffer               |
| nav.planner.min_turn_radius_m| 50.0       | Minimum dönüş yarıçapı           |
| nav.planner.polygon_segments | 12         | Circle→polygon segment sayısı    |
| nav.waypoint.acceptance_radius_m | 80.0  | Waypoint kabul yarıçapı         |
| nav.speed.cruise             | 20.0       | Seyir hızı (m/s)                 |
| nav.speed.pursuit            | 30.0       | Takip hızı (m/s)                 |
| nav.command.min_interval_ms  | 250        | Min komut aralığı (ms)           |
| nav.command.min_plan_interval_ms | 2000   | Min plan aralığı (ms)           |
| mission.follow.pursuit_range_m | 200.0   | Pursuit geçiş mesafesi          |
| mission.follow.pn_gain       | 3.0        | PN kazancı                       |
| mission.qr_dive.dive_pitch_deg | -45.0   | Dalış açısı                     |
| mission.qr_dive.approach_altitude_m | 120.0 | Yaklaşma yüksekliği         |
| mission.patrol.boundary_inset_m | 50.0    | Boundary inset mesafesi          |

SON
