#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/world_model/world_model.hpp"

#include <memory>
#include <vector>

namespace uav::mission {

class PatrolMode : public IMissionMode {
public:
    struct Config {
        double boundary_inset_m = 50.0;
        double cruise_altitude_m = 100.0;
        float patrol_speed_ms = 18.0f;
    };

    PatrolMode(std::shared_ptr<nav::NavigationEngine> nav,
               std::shared_ptr<world_model::WorldModel> world_model,
               Config config = {});

    void onEnter() override;
    void onExit() override;
    void tick(const core::UAVState& own_state) override;
    core::MissionGoal getMissionGoal() override;
    std::string getName() const override { return "PATROL"; }

private:
    std::shared_ptr<nav::NavigationEngine> nav_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;

    std::vector<core::Position> patrol_waypoints_;
    size_t current_patrol_idx_ = 0;
    bool waypoints_generated_ = false;

    void generatePatrolWaypoints();
    void planToNextWaypoint();
};

} // namespace uav::mission
