#include "uav/perception/simple_qr_detector.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

namespace uav::perception {

SimpleQRDetector::SimpleQRDetector(Config config) : config_(config) {}

void SimpleQRDetector::injectVisionLOS(float yaw_offset_deg, float pitch_offset_deg,
                                        float range_m, float confidence,
                                        const std::string& payload) {
    PerceptionTarget t;
    t.frame = PerceptionFrame::VISION_LOS;
    t.a = yaw_offset_deg;
    t.b = pitch_offset_deg;
    t.c = range_m;
    t.confidence = confidence;
    t.stamp_us = time::nowUs();
    t.source_tag = id();
    t.label = payload;

    std::lock_guard lock(mutex_);
    injected_ = std::move(t);
    injected_at_ = time::nowSteady();
    if (!payload.empty()) last_payload_ = payload;
}

void SimpleQRDetector::injectLocalOffset(float north_m, float east_m, float down_m,
                                          float confidence,
                                          const std::string& payload) {
    PerceptionTarget t;
    t.frame = PerceptionFrame::LOCAL_NED;
    t.a = north_m;
    t.b = east_m;
    t.c = down_m;
    t.confidence = confidence;
    t.stamp_us = time::nowUs();
    t.source_tag = id();
    t.label = payload;

    std::lock_guard lock(mutex_);
    injected_ = std::move(t);
    injected_at_ = time::nowSteady();
    if (!payload.empty()) last_payload_ = payload;
}

void SimpleQRDetector::clear() {
    std::lock_guard lock(mutex_);
    injected_.reset();
}

bool SimpleQRDetector::hasFreshDetection() const {
    std::lock_guard lock(mutex_);
    return injected_.has_value() &&
           time::durationS(injected_at_, time::nowSteady()) < config_.detection_timeout_s;
}

std::optional<std::string> SimpleQRDetector::lastPayload() const {
    std::lock_guard lock(mutex_);
    return last_payload_;
}

void SimpleQRDetector::start() {
    running_ = true;
    LOG_INFO("SimpleQRDetector: start (expected='{}')", config_.expected_payload);
}

void SimpleQRDetector::stop() {
    running_ = false;
    clear();
    LOG_INFO("SimpleQRDetector: stop");
}

std::optional<PerceptionTarget> SimpleQRDetector::poll(const core::UAVState& /*own_state*/) {
    if (!running_) return std::nullopt;

    std::lock_guard lock(mutex_);
    if (!injected_.has_value()) return std::nullopt;

    if (time::durationS(injected_at_, time::nowSteady()) >= config_.detection_timeout_s) {
        injected_.reset();
        return std::nullopt;
    }

    if (!config_.expected_payload.empty() &&
        injected_->label != config_.expected_payload) {
        // Wrong QR payload; ignore.
        return std::nullopt;
    }
    return injected_;
}

} // namespace uav::perception
