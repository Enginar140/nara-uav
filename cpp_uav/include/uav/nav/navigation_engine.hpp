#pragma once

#include "uav/nav/path_planner.hpp"
#include "uav/nav/waypoint_chain.hpp"
#include "uav/nav/speed_manager.hpp"
#include "uav/safety/safety_gate.hpp"
#include "uav/world_model/world_model.hpp"

#include <chrono>
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
    bool flyLocalPosition(float north, float east, float down);
    bool flyLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);

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
