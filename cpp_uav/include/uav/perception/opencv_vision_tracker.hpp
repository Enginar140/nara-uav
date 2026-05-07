#pragma once

#ifdef UAV_HAS_OPENCV

#include "uav/perception/iperception_module.hpp"
#include "uav/perception/camera/icamera_source.hpp"
#include "uav/perception/detector/iframe_detector.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace uav::perception {

/// Real-camera vision tracker. Owns one camera source + one frame detector,
/// runs both on a dedicated worker thread, and exposes the latest detection
/// as a VISION_LOS PerceptionTarget through poll().
///
/// LOS angle math:
///   fx = (image_width  / 2) / tan(h_fov/2)
///   fy = (image_height / 2) / tan(v_fov/2)
///   yaw_off_deg   =  atan2(cx_px - cx0, fx) * rad2deg     (right of center -> +)
///   pitch_off_deg =  atan2(cy_px - cy0, fy) * rad2deg     (below center  -> +)
///
/// Range estimate is configurable:
///   FIXED      -> always cfg_.fixed_range_m
///   SIZE       -> real_diameter_m * fx / (2 * radius_px)
///   ALTITUDE   -> -own_alt / tan(pitch_off + own_pitch) when looking down
class OpenCvVisionTracker : public IPerceptionModule {
public:
    enum class RangeMode { FIXED, SIZE, ALTITUDE };

    struct Config {
        std::string id_tag = "vision_tracker_opencv";

        // Camera intrinsics (must roughly match camera_->config())
        double h_fov_deg = 60.0;
        double v_fov_deg = 45.0;

        // Tracking behaviour
        double detection_timeout_s = 1.0;   // poll() returns nullopt after this many s without a hit
        float min_confidence = 0.5f;
        int target_loss_max_frames = 5;     // logged loss before clearing

        // Range estimation
        RangeMode range_mode = RangeMode::FIXED;
        float fixed_range_m = 50.0f;
        float real_diameter_m = 0.30f;      // for SIZE mode
        float min_range_m = 1.0f;
        float max_range_m = 1000.0f;

        // Worker tick budget (independent from camera FPS - we always
        // process the freshest frame). 0 disables sleep.
        int worker_sleep_ms = 5;

        // Preview window (cv::imshow)
        bool preview_enabled = false;
        std::string preview_title = "uav vision (round)";
    };

    OpenCvVisionTracker(std::shared_ptr<ICameraSource> camera,
                        std::shared_ptr<IFrameDetector> detector,
                        Config cfg);
    ~OpenCvVisionTracker() override;

    std::string id() const override { return cfg_.id_tag; }
    void start() override;
    void stop() override;
    std::optional<PerceptionTarget> poll(const core::UAVState& own_state) override;

    /// MUST be called from the application's main thread when preview is
    /// enabled. OpenCV's HighGUI under Windows ignores cv::imshow / waitKey
    /// when invoked from a non-main worker, so we hand the latest annotated
    /// frame off here. Cheap no-op when preview is disabled or no frame is
    /// available yet.
    void tickPreviewMainThread();

    bool previewEnabled() const { return cfg_.preview_enabled; }

    // Diagnostics for tests / dashboards.
    uint64_t framesProcessed() const { return frames_processed_.load(); }
    uint64_t detections() const { return detections_.load(); }

private:
    void workerLoop();
    std::optional<PerceptionTarget> buildTarget(const FrameDetection& det,
                                                const core::UAVState& own_state) const;
    float estimateRange(const FrameDetection& det,
                        float yaw_off_deg, float pitch_off_deg,
                        const core::UAVState& own_state) const;

    std::shared_ptr<ICameraSource> camera_;
    std::shared_ptr<IFrameDetector> detector_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread worker_;

    mutable std::mutex result_mutex_;
    std::optional<PerceptionTarget> last_target_;
    std::chrono::steady_clock::time_point last_target_time_{};
    int consecutive_misses_ = 0;

    mutable std::mutex preview_mutex_;
    cv::Mat preview_frame_;          // populated by worker, drained by main thread
    bool preview_window_created_ = false;

    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> detections_{0};
};

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
