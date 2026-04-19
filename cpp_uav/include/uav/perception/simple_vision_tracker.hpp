#pragma once

#include "uav/perception/iperception_module.hpp"
#include "uav/world_model/world_model.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

namespace uav::perception {

/// Reference vision tracker. Stand-in for a real CV pipeline (YOLO/ByteTrack/etc).
///
/// Keeps a synthetic camera-frame line-of-sight target that callers can inject
/// via setSyntheticDetection(). When no fresh detection is available, falls
/// back to the WorldModel's tracked opponent (the same target FollowMode uses)
/// and projects it into the camera frame using current AHRS yaw.
///
/// Output frame: VISION_LOS (yaw_offset_deg, pitch_offset_deg, range_m) so
/// NavigationEngine routes through pursueVisionTarget() which body-rotates the
/// command into the active heading.
class SimpleVisionTracker : public IPerceptionModule {
public:
    struct Config {
        double detection_timeout_s = 1.0;  // injected detections expire after this
        float default_range_m = 100.0f;    // used when no range estimate is available
        float min_confidence = 0.5f;       // confidence floor for synthetic injections
        int target_id = -1;                // WorldModel opponent id; -1 = nearest
        bool use_world_fallback = true;    // synthesise from WorldModel if no injection
    };

    SimpleVisionTracker(std::shared_ptr<world_model::WorldModel> world_model,
                        Config config = {});

    /// Inject a synthetic detection (used by tests + while real pipeline is offline).
    /// Coordinates are camera-frame LOS angles in degrees.
    void setSyntheticDetection(float yaw_offset_deg, float pitch_offset_deg,
                                float range_m, float confidence = 0.9f);
    void clearSyntheticDetection();

    void setTargetId(int id);

    std::string id() const override { return "vision_tracker"; }
    void start() override;
    void stop() override;
    std::optional<PerceptionTarget> poll(const core::UAVState& own_state) override;

private:
    std::optional<PerceptionTarget> pollWorldModel(const core::UAVState& own_state);

    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;

    mutable std::mutex injection_mutex_;
    std::optional<PerceptionTarget> injected_;
    std::chrono::steady_clock::time_point injected_at_;

    std::atomic<bool> running_{false};
};

} // namespace uav::perception
