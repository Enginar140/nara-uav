#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace uav::core {

// =============================================================================
// Geographic Types
// =============================================================================

struct Position {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    Position() = default;
    Position(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}

    bool operator==(const Position& o) const {
        constexpr double kLL = 1e-9;
        constexpr double kAlt = 0.01;
        return std::abs(latitude - o.latitude) < kLL &&
               std::abs(longitude - o.longitude) < kLL &&
               std::abs(altitude - o.altitude) < kAlt;
    }
    bool operator!=(const Position& o) const { return !(*this == o); }
};

struct Velocity {
    double north = 0.0;
    double east = 0.0;
    double down = 0.0;

    Velocity() = default;
    Velocity(double n, double e, double d) : north(n), east(e), down(d) {}

    double horizontalSpeed() const { return std::sqrt(north * north + east * east); }
    double speed3D() const { return std::sqrt(north * north + east * east + down * down); }
};

struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Attitude() = default;
    Attitude(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
};

// =============================================================================
// Aircraft State
// =============================================================================

struct UAVState {
    int id = 0;
    Position position;
    Velocity velocity;
    Attitude attitude;
    double heading = 0.0;
    double airspeed = 0.0;
    double groundspeed = 0.0;
    uint64_t timestamp_us = 0;

    uint8_t gps_fix_type = 0;
    uint8_t satellites_visible = 0;
    uint16_t gps_hdop = 9999;

    float battery_voltage = 0.0f;
    int8_t battery_remaining = -1;

    bool isValid() const { return timestamp_us > 0; }
    double ageSeconds() const;
};

// =============================================================================
// Competition Data
// =============================================================================

struct HSSZone {
    int id = 0;
    Position center;
    double radius = 0.0;
    bool active = false;
    double buffer_distance = 40.0;

    HSSZone() = default;
    HSSZone(int zone_id, const Position& pos, double r, bool act = true)
        : id(zone_id), center(pos), radius(r), active(act) {}

    double effectiveRadius() const { return radius + buffer_distance; }
};

struct Boundary {
    std::vector<Position> vertices;
    double min_altitude = 30.0;
    double max_altitude = 400.0;

    Boundary() = default;

    Position getCenter() const;
    bool isValid() const { return vertices.size() >= 3; }
};

// =============================================================================
// Command & Mission Types
// =============================================================================

enum class GoalType {
    NONE,
    PATROL,
    TRACK_TARGET,
    DIVE_TARGET,
    ESCAPE
};

struct MissionGoal {
    GoalType type = GoalType::NONE;
    std::optional<Position> target_position;
    std::optional<int> target_id;

    MissionGoal() = default;
    explicit MissionGoal(GoalType t) : type(t) {}
};

enum class MissionMode {
    ARDUPILOT,
    PATROL,
    FOLLOW,
    QR_DIVE
};

// =============================================================================
// Safety Types
// =============================================================================

enum class RiskLevel { NONE = 0, LOW = 1, MEDIUM = 2, HIGH = 3, CRITICAL = 4 };
enum class BoundaryStatus { INSIDE = 0, NEAR_BOUNDARY = 1, OUTSIDE = 2 };

struct Threat {
    int aircraft_id = 0;
    double distance = 0.0;
    double bearing = 0.0;
    double closing_speed = 0.0;
    double time_to_collision = INFINITY;
    RiskLevel risk_level = RiskLevel::NONE;
};

struct SafetyState {
    RiskLevel collision_risk = RiskLevel::NONE;
    BoundaryStatus boundary_status = BoundaryStatus::INSIDE;
    std::vector<int> hss_violations;
    std::vector<Threat> active_threats;
    bool comm_loss = false;

    bool isSafe() const {
        return collision_risk < RiskLevel::HIGH &&
               boundary_status == BoundaryStatus::INSIDE &&
               hss_violations.empty() &&
               !comm_loss;
    }
};

// =============================================================================
// String Conversions
// =============================================================================

std::string toString(MissionMode mode);
std::string toString(GoalType type);
std::string toString(RiskLevel level);
std::string toString(BoundaryStatus status);

} // namespace uav::core
