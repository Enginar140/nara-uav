#include "uav/mission/follow_mode.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uav::mission {

const char* toString(FollowPhase phase) {
    switch (phase) {
        case FollowPhase::APPROACH:   return "APPROACH";
        case FollowPhase::PURSUIT:    return "PURSUIT";
        case FollowPhase::REACQUIRE:  return "REACQUIRE";
    }
    return "UNKNOWN";
}

FollowMode::FollowMode(std::shared_ptr<nav::NavigationEngine> nav,
                       std::shared_ptr<world_model::WorldModel> world_model,
                       Config config)
    : nav_(std::move(nav)), world_model_(std::move(world_model)), config_(config) {}

void FollowMode::setPerceptionEngine(std::shared_ptr<perception::PerceptionEngine> engine,
                                     std::shared_ptr<infra::EventBus> bus) {
    perception_ = std::move(engine);
    event_bus_ = std::move(bus);
}

void FollowMode::setTargetId(int id) { target_id_ = id; }

void FollowMode::enablePerception() {
    if (!config_.vision_fallback || !perception_ || !event_bus_) return;
    if (perception_active_) return;

    perception_sub_id_ = event_bus_->subscribe(
        infra::EventType::PERCEPTION_UPDATED,
        [this](const infra::Event& ev) {
            if (!ev.hasPayload<perception::PerceptionEvent>()) return;
            auto pe = ev.getPayload<perception::PerceptionEvent>();
            std::lock_guard lock(perception_mutex_);
            if (pe.has_lock) {
                last_perception_ = pe.target;
                last_perception_time_ = time::nowSteady();
            } else {
                last_perception_.reset();
            }
        });

    // Only steal the active module slot when world-model has nothing; we set
    // it lazily inside tickVisionFallback() so QRDiveMode is not interrupted.
    perception_active_ = true;
    LOG_INFO("FollowMode: vision fallback armed (module='{}')", config_.vision_module_id);
}

void FollowMode::disablePerception() {
    if (!perception_active_) return;
    if (event_bus_ && perception_sub_id_ != 0) {
        event_bus_->unsubscribe(perception_sub_id_);
        perception_sub_id_ = 0;
    }
    if (perception_ && perception_->activeId() == config_.vision_module_id) {
        (void)perception_->setActive("");
    }
    {
        std::lock_guard lock(perception_mutex_);
        last_perception_.reset();
    }
    perception_active_ = false;
    using_vision_fallback_ = false;
    LOG_INFO("FollowMode: vision fallback disarmed");
}

bool FollowMode::tickVisionFallback() {
    if (!perception_active_ || !nav_) return false;

    // Promote the module to active if not already (lazy activation).
    if (perception_ && perception_->activeId() != config_.vision_module_id) {
        (void)perception_->setActive(config_.vision_module_id);
    }

    perception::PerceptionTarget target;
    {
        std::lock_guard lock(perception_mutex_);
        if (!last_perception_.has_value()) return false;
        if (time::durationS(last_perception_time_, time::nowSteady()) >
            config_.vision_lock_age_s) {
            return false;
        }
        target = *last_perception_;
    }
    if (!nav_->dispatchPerceptionTarget(target)) return false;
    if (!using_vision_fallback_) {
        LOG_INFO("FollowMode: switched to vision fallback ({} frame, conf={:.2f})",
                 perception::toString(target.frame), target.confidence);
        using_vision_fallback_ = true;
    }
    return true;
}

void FollowMode::onEnter() {
    LOG_INFO("FollowMode: entering, target_id={}", target_id_);
    phase_ = FollowPhase::APPROACH;
    target_permanently_lost_ = false;
    los_initialized_ = false;
    using_vision_fallback_ = false;
    enablePerception();
}

void FollowMode::onExit() {
    LOG_INFO("FollowMode: exiting");
    disablePerception();
    nav_->clearRoute();
}

void FollowMode::tick(const core::UAVState& own_state) {
    if (target_id_ < 0) return;

    auto target_opt = world_model_->getOpponent(target_id_);
    bool world_target_fresh = target_opt.has_value() &&
                              target_opt->ageSeconds() <= config_.target_stale_s;

    if (!world_target_fresh) {
        // Before giving up, attempt vision fallback if armed. This keeps the
        // plane pointed at whatever the camera/perception module sees while
        // the operator/world-model recovers a fresh track.
        if (config_.vision_fallback && tickVisionFallback()) return;

        if (phase_ != FollowPhase::REACQUIRE) {
            transitionTo(FollowPhase::REACQUIRE);
        }
        tickReacquire(own_state);
        return;
    }

    // World-model recovered: drop vision fallback and reset companion-side
    // GUIDED state so the new pursuit command is not fighting a stale yaw.
    if (using_vision_fallback_) {
        LOG_INFO("FollowMode: world-model target recovered, releasing vision fallback");
        using_vision_fallback_ = false;
        if (nav_) (void)nav_->releaseGuidedHold();
    }

    auto& target = *target_opt;

    switch (phase_) {
        case FollowPhase::APPROACH:
            tickApproach(own_state, target);
            break;
        case FollowPhase::PURSUIT:
            tickPursuit(own_state, target);
            break;
        case FollowPhase::REACQUIRE:
            transitionTo(FollowPhase::APPROACH);
            tickApproach(own_state, target);
            break;
    }
}

core::MissionGoal FollowMode::getMissionGoal() {
    core::MissionGoal goal(core::GoalType::TRACK_TARGET);
    goal.target_id = target_id_;
    return goal;
}

void FollowMode::tickApproach(const core::UAVState& own,
                              const core::UAVState& target) {
    double dist = geo::haversineDistance(own.position, target.position);

    if (dist <= config_.pursuit_range_m) {
        transitionTo(FollowPhase::PURSUIT);
        tickPursuit(own, target);
        return;
    }

    auto now = time::nowSteady();
    double since_plan = time::elapsedS(last_plan_time_);
    if (since_plan >= config_.plan_update_interval_s || !nav_->hasActiveRoute()) {
        auto predicted = predictTarget(target);
        if (nav_->planRoute(predicted, config_.approach_speed_ms)) {
            last_plan_time_ = now;
        }
    } else {
        nav_->update(own.position);
    }
}

void FollowMode::tickPursuit(const core::UAVState& own,
                             const core::UAVState& target) {
    double dist = geo::haversineDistance(own.position, target.position);

    if (dist > config_.pursuit_range_m * 1.5) {
        transitionTo(FollowPhase::APPROACH);
        los_initialized_ = false;
        return;
    }

    auto intercept_pos = predictWithPN(own, target);
    nav_->flyDirect(intercept_pos, config_.pursuit_speed_ms);
}

void FollowMode::tickReacquire(const core::UAVState& own) {
    double since_lost = time::elapsedS(target_lost_time_);

    if (since_lost >= config_.max_reacquire_s) {
        LOG_WARN("FollowMode: no opponents for {:.1f}s, giving up", since_lost);
        target_permanently_lost_ = true;
        if (lost_cb_) lost_cb_();
        return;
    }

    auto now_log = time::nowSteady();
    if (time::durationS(last_reacquire_log_, now_log) > 5.0) {
        LOG_WARN("FollowMode: no opponents visible for {:.1f}s", since_lost);
        last_reacquire_log_ = now_log;
    }

    nav_->update(own.position);
}

core::Position FollowMode::predictWithPN(const core::UAVState& own,
                                         const core::UAVState& target) {
    double los_bearing = geo::bearing(own.position, target.position);
    auto now = time::nowSteady();

    if (!los_initialized_) {
        prev_los_bearing_ = los_bearing;
        prev_los_time_ = now;
        los_initialized_ = true;
        return predictTarget(target);
    }

    double dt = time::durationS(prev_los_time_, now);
    if (dt < 0.05) {
        return predictTarget(target);
    }

    double los_delta = geo::angleDifference(los_bearing, prev_los_bearing_);
    double los_rate = (los_delta * geo::kDegToRad) / dt;

    prev_los_bearing_ = los_bearing;
    prev_los_time_ = now;

    double dist = geo::haversineDistance(own.position, target.position);
    double closing_speed = target.velocity.horizontalSpeed() + own.airspeed;
    if (closing_speed < 5.0) closing_speed = 5.0;
    double t_go = dist / closing_speed;

    double pn_offset_rad = config_.pn_gain * los_rate * t_go;

    double max_lead_rad = config_.pn_max_lead_deg * geo::kDegToRad;
    pn_offset_rad = std::clamp(pn_offset_rad, -max_lead_rad, max_lead_rad);

    double intercept_bearing = los_bearing + pn_offset_rad * geo::kRadToDeg;
    double intercept_dist = config_.pursuit_speed_ms * t_go;
    intercept_dist = std::min(intercept_dist, dist * 1.5);

    auto intercept = geo::destinationPoint(own.position, intercept_bearing, intercept_dist);
    intercept.altitude = target.position.altitude;

    LOG_DEBUG("FollowMode PN: LOS={:.1f} rate={:.3f}rad/s offset={:.1f}deg t_go={:.1f}s",
              los_bearing, los_rate, pn_offset_rad * geo::kRadToDeg, t_go);

    return intercept;
}

core::Position FollowMode::predictTarget(const core::UAVState& target) const {
    double dt = config_.prediction_horizon_s;
    double bearing_deg = std::atan2(target.velocity.east, target.velocity.north) *
                         geo::kRadToDeg;
    double speed = target.velocity.horizontalSpeed();
    double distance = speed * dt;

    auto predicted = geo::destinationPoint(target.position, bearing_deg, distance);
    predicted.altitude = target.position.altitude;
    return predicted;
}

void FollowMode::transitionTo(FollowPhase phase) {
    LOG_INFO("FollowMode: {} -> {}", toString(phase_), toString(phase));
    if (phase_ == FollowPhase::REACQUIRE && phase != FollowPhase::REACQUIRE) {
        nav_->clearRoute();
    }
    phase_ = phase;
    if (phase == FollowPhase::REACQUIRE) {
        target_lost_time_ = time::nowSteady();
    }
}

} // namespace uav::mission
