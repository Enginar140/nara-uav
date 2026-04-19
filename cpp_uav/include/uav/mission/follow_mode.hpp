#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/perception/perception_engine.hpp"
#include "uav/perception/perception_types.hpp"
#include "uav/world_model/world_model.hpp"
#include "uav/infra/event_bus.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

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
        // Vision fallback: when world-model has no fresh target, drive the
        // plane from the active PerceptionEngine module instead of going to
        // REACQUIRE immediately. Set false to keep legacy behaviour.
        bool vision_fallback = false;
        std::string vision_module_id = "vision_tracker";
        double vision_lock_age_s = 0.5;
    };

    FollowMode(std::shared_ptr<nav::NavigationEngine> nav,
               std::shared_ptr<world_model::WorldModel> world_model,
               Config config = {});

    void setPerceptionEngine(std::shared_ptr<perception::PerceptionEngine> engine,
                             std::shared_ptr<infra::EventBus> bus);

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
    std::shared_ptr<perception::PerceptionEngine> perception_;
    std::shared_ptr<infra::EventBus> event_bus_;
    uint64_t perception_sub_id_ = 0;
    Config config_;

    int target_id_ = -1;
    FollowPhase phase_ = FollowPhase::APPROACH;
    bool target_permanently_lost_ = false;
    bool using_vision_fallback_ = false;
    std::chrono::steady_clock::time_point target_lost_time_;
    std::chrono::steady_clock::time_point last_plan_time_;
    std::function<void()> lost_cb_;

    mutable std::mutex perception_mutex_;
    std::optional<perception::PerceptionTarget> last_perception_;
    std::chrono::steady_clock::time_point last_perception_time_;
    std::atomic<bool> perception_active_{false};

    bool tickVisionFallback();
    void enablePerception();
    void disablePerception();

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
