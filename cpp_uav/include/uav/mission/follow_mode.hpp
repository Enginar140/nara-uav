#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/world_model/world_model.hpp"

#include <chrono>
#include <functional>
#include <memory>

namespace uav::mission {

enum class FollowPhase { APPROACH, PURSUIT, REACQUIRE };

const char* toString(FollowPhase phase);

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
        double pn_gain = 3.0;
        double pn_max_lead_deg = 45.0;
    };

    FollowMode(std::shared_ptr<nav::NavigationEngine> nav,
               std::shared_ptr<world_model::WorldModel> world_model,
               Config config = {});

    void setTargetId(int id);
    int getTargetId() const { return target_id_; }
    FollowPhase getPhase() const { return phase_; }
    bool isTargetLost() const { return target_permanently_lost_; }
    void setTargetLostCallback(std::function<void()> cb) { lost_cb_ = std::move(cb); }

    void onEnter() override;
    void onExit() override;
    void tick(const core::UAVState& own_state) override;
    core::MissionGoal getMissionGoal() override;
    std::string getName() const override { return "FOLLOW"; }

private:
    std::shared_ptr<nav::NavigationEngine> nav_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;

    int target_id_ = -1;
    FollowPhase phase_ = FollowPhase::APPROACH;
    bool target_permanently_lost_ = false;
    std::chrono::steady_clock::time_point target_lost_time_;
    std::chrono::steady_clock::time_point last_plan_time_;
    std::function<void()> lost_cb_;

    double prev_los_bearing_ = 0.0;
    std::chrono::steady_clock::time_point prev_los_time_;
    std::chrono::steady_clock::time_point last_reacquire_log_;
    bool los_initialized_ = false;

    void tickApproach(const core::UAVState& own, const core::UAVState& target);
    void tickPursuit(const core::UAVState& own, const core::UAVState& target);
    void tickReacquire(const core::UAVState& own);

    core::Position predictTarget(const core::UAVState& target) const;
    core::Position predictWithPN(const core::UAVState& own, const core::UAVState& target);
    void transitionTo(FollowPhase phase);
};

} // namespace uav::mission
