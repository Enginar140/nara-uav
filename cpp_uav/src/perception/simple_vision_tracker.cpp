#include "uav/perception/simple_vision_tracker.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>

namespace uav::perception {

SimpleVisionTracker::SimpleVisionTracker(std::shared_ptr<world_model::WorldModel> world_model,
                                         Config config)
    : world_model_(std::move(world_model)), config_(config) {}

void SimpleVisionTracker::setSyntheticDetection(float yaw_offset_deg, float pitch_offset_deg,
                                                float range_m, float confidence) {
    PerceptionTarget t;
    t.frame = PerceptionFrame::VISION_LOS;
    t.a = yaw_offset_deg;
    t.b = pitch_offset_deg;
    t.c = range_m;
    t.confidence = confidence;
    t.stamp_us = time::nowUs();
    t.source_tag = id();

    std::lock_guard lock(injection_mutex_);
    injected_ = std::move(t);
    injected_at_ = time::nowSteady();
}

void SimpleVisionTracker::clearSyntheticDetection() {
    std::lock_guard lock(injection_mutex_);
    injected_.reset();
}

void SimpleVisionTracker::setTargetId(int id) { config_.target_id = id; }

void SimpleVisionTracker::start() {
    running_ = true;
    LOG_INFO("SimpleVisionTracker: start (target_id={}, world_fallback={})",
             config_.target_id, config_.use_world_fallback);
}

void SimpleVisionTracker::stop() {
    running_ = false;
    clearSyntheticDetection();
    LOG_INFO("SimpleVisionTracker: stop");
}

std::optional<PerceptionTarget> SimpleVisionTracker::poll(const core::UAVState& own_state) {
    if (!running_) return std::nullopt;

    {
        std::lock_guard lock(injection_mutex_);
        if (injected_.has_value() &&
            time::durationS(injected_at_, time::nowSteady()) < config_.detection_timeout_s) {
            return injected_;
        }
        injected_.reset();
    }

    if (!config_.use_world_fallback || !world_model_) return std::nullopt;
    return pollWorldModel(own_state);
}

std::optional<PerceptionTarget> SimpleVisionTracker::pollWorldModel(const core::UAVState& own_state) {
    if (!own_state.isValid()) return std::nullopt;

    std::optional<core::UAVState> target;
    if (config_.target_id >= 0) {
        target = world_model_->getOpponent(config_.target_id);
    } else {
        target = world_model_->getNearestOpponent(own_state.position);
    }
    if (!target.has_value() || !target->isValid()) return std::nullopt;

    double bearing_to = geo::bearing(own_state.position, target->position);
    double yaw_offset = geo::angleDifference(bearing_to, own_state.heading);

    double horiz_dist = geo::haversineDistance(own_state.position, target->position);
    double down = own_state.position.altitude - target->position.altitude;
    double pitch_offset = std::atan2(down, std::max(1.0, horiz_dist)) * geo::kRadToDeg;
    double slant_range = std::sqrt(horiz_dist * horiz_dist + down * down);
    if (slant_range < 1.0) slant_range = config_.default_range_m;

    PerceptionTarget t;
    t.frame = PerceptionFrame::VISION_LOS;
    t.a = static_cast<float>(yaw_offset);
    t.b = static_cast<float>(pitch_offset);
    t.c = static_cast<float>(slant_range);
    t.confidence = config_.min_confidence;
    t.stamp_us = time::nowUs();
    t.source_tag = id();
    t.label = "world_fallback";
    return t;
}

} // namespace uav::perception
