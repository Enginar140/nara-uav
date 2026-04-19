#pragma once

#include "uav/nav/path_planner.hpp"
#include "uav/nav/waypoint_chain.hpp"
#include "uav/nav/speed_manager.hpp"
#include "uav/perception/perception_types.hpp"
#include "uav/safety/safety_gate.hpp"
#include "uav/world_model/world_model.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <optional>

namespace uav::comm {
class FlightCommander;
}

namespace uav::nav {

class NavigationEngine {
public:
    NavigationEngine(std::shared_ptr<PathPlanner> planner,
                     std::shared_ptr<comm::FlightCommander> commander,
                     std::shared_ptr<safety::SafetyGate> safety_gate,
                     std::shared_ptr<SpeedManager> speed_manager,
                     std::shared_ptr<world_model::WorldModel> world_model);

    bool planRoute(const core::Position& goal, float airspeed = -1.0f);

    ChainStatus update(const core::Position& own_pos);

    bool flyDirect(const core::Position& target, float airspeed = -1.0f);

    /// Legacy local-position / local-velocity APIs.
    /// Stock ArduPlane silently ignores LOCAL_NED frame and velocity setpoints.
    /// Our feature/local-control fork only implements offset-position semantics.
    /// New code should use the four high-level wrappers below.
    bool flyLocalPosition(float north, float east, float down);
    bool flyLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);

    // ---------------------------------------------------------------------
    // Local control wrappers (feature/local-control fork).
    //
    // Each wrapper routes through the corresponding FlightCommander method
    // after applying command-rate throttling. yaw_deg is optional; pass NaN
    // (the default) to leave heading uncommanded.
    // ---------------------------------------------------------------------

    /// Body-frame offset: forward / right / down in metres.
    bool flyBodyOffset(float forward_m, float right_m, float down_m,
                       float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Local NED offset: north / east / down in metres relative to current loc.
    bool flyLocalOffset(float north_m, float east_m, float down_m,
                        float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Polar offset: bearing (deg from north) + distance + altitude offset.
    bool flyBearing(float bearing_deg, float distance_m, float altitude_offset_m,
                    float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Vision pursuit: yaw/pitch offset of the target in the camera frame
    /// plus an estimated range. Used by QR_DIVE and similar vision-driven
    /// modes.
    bool pursueVisionTarget(float yaw_offset_deg, float pitch_offset_deg, float range_m,
                            float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Frame-aware dispatch for perception modules. Routes the target to the
    /// correct local-control wrapper based on its frame:
    ///   BODY       -> flyBodyOffset
    ///   LOCAL_NED  -> flyLocalOffset
    ///   POLAR      -> flyBearing
    ///   VISION_LOS -> pursueVisionTarget
    /// Returns false if throttled or if the target frame is unsupported.
    bool dispatchPerceptionTarget(const perception::PerceptionTarget& target);

    /// Companion-side "stop holding state" command. Sends one zero-magnitude
    /// BODY_OFFSET_NED with YAW_IGNORE so the firmware clears any sticky
    /// guided_state.target_heading from previous commands. Use this when
    /// transitioning between mission modes that drive GUIDED, before the next
    /// mode's onEnter() issues fresh commands.
    bool releaseGuidedHold();

    void clearRoute();
    bool hasActiveRoute() const;
    bool revalidateRoute();

    void setMinCommandInterval(std::chrono::milliseconds ms) { min_cmd_interval_ = ms; }
    void setMinPlanInterval(std::chrono::milliseconds ms) { min_plan_interval_ = ms; }

    const WaypointChain& getChain() const { return chain_; }

private:
    std::shared_ptr<PathPlanner> planner_;
    std::shared_ptr<comm::FlightCommander> commander_;
    std::shared_ptr<safety::SafetyGate> safety_gate_;
    std::shared_ptr<SpeedManager> speed_manager_;
    std::shared_ptr<world_model::WorldModel> world_model_;

    WaypointChain chain_;

    std::chrono::steady_clock::time_point last_cmd_time_;
    std::chrono::steady_clock::time_point last_plan_time_;
    std::chrono::milliseconds min_cmd_interval_{250};
    std::chrono::milliseconds min_plan_interval_{2000};
    std::optional<core::Position> last_sent_target_;

    bool sendCurrentWaypoint();
    bool shouldThrottleCommand() const;
};

} // namespace uav::nav
