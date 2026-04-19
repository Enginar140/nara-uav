#ifdef UAV_HAS_OPENCV

#include "uav/perception/opencv_vision_tracker.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
}

namespace uav::perception {

OpenCvVisionTracker::OpenCvVisionTracker(std::shared_ptr<ICameraSource> camera,
                                         std::shared_ptr<IFrameDetector> detector,
                                         Config cfg)
    : camera_(std::move(camera)), detector_(std::move(detector)), cfg_(std::move(cfg)) {}

OpenCvVisionTracker::~OpenCvVisionTracker() { stop(); }

void OpenCvVisionTracker::start() {
    if (running_.exchange(true)) return;

    if (!camera_ || !detector_) {
        LOG_ERROR("OpenCvVisionTracker: missing camera or detector - refusing to start");
        running_ = false;
        return;
    }

    if (!camera_->open()) {
        LOG_ERROR("OpenCvVisionTracker: camera open failed ({})", camera_->description());
        running_ = false;
        return;
    }

    LOG_INFO("OpenCvVisionTracker: start (camera={}, detector={}, fov={:.1f}x{:.1f} deg, "
             "range_mode={}, fixed_range={:.1f} m)",
             camera_->description(), detector_->id(),
             cfg_.h_fov_deg, cfg_.v_fov_deg,
             static_cast<int>(cfg_.range_mode), cfg_.fixed_range_m);

    worker_ = std::thread([this] { workerLoop(); });
}

void OpenCvVisionTracker::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
    if (camera_) camera_->close();
    if (cfg_.preview_enabled) {
        try { cv::destroyWindow(cfg_.preview_title); } catch (...) {}
    }
    {
        std::lock_guard lock(result_mutex_);
        last_target_.reset();
    }
    LOG_INFO("OpenCvVisionTracker: stop (frames={}, detections={})",
             frames_processed_.load(), detections_.load());
}

void OpenCvVisionTracker::workerLoop() {
    cv::Mat frame;
    uint64_t stamp_us = 0;
    LOG_INFO("OpenCvVisionTracker: worker thread up");

    while (running_.load()) {
        if (!camera_->read(frame, stamp_us)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        frames_processed_.fetch_add(1, std::memory_order_relaxed);

        auto det = detector_->detect(frame);
        if (det && det->confidence >= cfg_.min_confidence) {
            // Image-frame center; OpenCV origin top-left.
            const float cx0 = camera_->width() * 0.5f;
            const float cy0 = camera_->height() * 0.5f;
            const float fx = static_cast<float>(
                cx0 / std::tan(cfg_.h_fov_deg * 0.5 * kDeg2Rad));
            const float fy = static_cast<float>(
                cy0 / std::tan(cfg_.v_fov_deg * 0.5 * kDeg2Rad));

            const float yaw_off_deg   = static_cast<float>(
                std::atan2(det->cx_px - cx0, fx) * kRad2Deg);
            const float pitch_off_deg = static_cast<float>(
                std::atan2(cy0 - det->cy_px, fy) * kRad2Deg);

            // own_state is fed via poll(); workerLoop builds a placeholder
            // target with a known-good range. poll() augments with own_state-
            // derived ALTITUDE range when configured.
            core::UAVState dummy_state{};
            const float range_m = estimateRange(*det, yaw_off_deg, pitch_off_deg, dummy_state);

            PerceptionTarget t;
            t.frame = PerceptionFrame::VISION_LOS;
            t.a = yaw_off_deg;
            t.b = pitch_off_deg;
            t.c = range_m;
            t.confidence = det->confidence;
            t.stamp_us = stamp_us;
            t.source_tag = id();
            t.label = det->label;

            {
                std::lock_guard lock(result_mutex_);
                last_target_ = std::move(t);
                last_target_time_ = time::nowSteady();
                consecutive_misses_ = 0;
            }
            detections_.fetch_add(1, std::memory_order_relaxed);
        } else {
            int misses;
            {
                std::lock_guard lock(result_mutex_);
                misses = ++consecutive_misses_;
            }
            if (misses == cfg_.target_loss_max_frames) {
                LOG_DEBUG("OpenCvVisionTracker: target lost ({} consecutive misses)", misses);
            }
        }

        if (cfg_.preview_enabled) {
            // Annotate with last detection so the operator sees the lock,
            // then hand the frame to the main thread (HighGUI on Windows
            // refuses imshow/waitKey from non-main threads).
            std::optional<PerceptionTarget> snap;
            {
                std::lock_guard lock(result_mutex_);
                snap = last_target_;
            }
            const int cx0 = camera_->width() / 2;
            const int cy0 = camera_->height() / 2;
            cv::line(frame, {cx0, 0}, {cx0, camera_->height()}, {60, 60, 60}, 1);
            cv::line(frame, {0, cy0}, {camera_->width(), cy0}, {60, 60, 60}, 1);
            if (snap.has_value()) {
                const int cx = static_cast<int>(camera_->width() * 0.5f
                    + std::tan(snap->a * kDeg2Rad)
                      * (camera_->width() * 0.5f / std::tan(cfg_.h_fov_deg * 0.5 * kDeg2Rad)));
                const int cy = static_cast<int>(camera_->height() * 0.5f
                    - std::tan(snap->b * kDeg2Rad)
                      * (camera_->height() * 0.5f / std::tan(cfg_.v_fov_deg * 0.5 * kDeg2Rad)));
                cv::circle(frame, {cx, cy}, 18, {0, 255, 0}, 2);
                cv::circle(frame, {cx, cy}, 3, {0, 0, 255}, -1);
                cv::line(frame, {cx0, cy0}, {cx, cy}, {0, 200, 255}, 1);
                cv::putText(frame,
                            cv::format("yaw=%+5.1f pitch=%+5.1f r=%.1fm conf=%.2f",
                                       snap->a, snap->b, snap->c, snap->confidence),
                            {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            {0, 255, 0}, 1, cv::LINE_AA);
            } else {
                cv::putText(frame, "no lock", {10, 24}, cv::FONT_HERSHEY_SIMPLEX,
                            0.6, {0, 0, 255}, 1, cv::LINE_AA);
            }
            {
                std::lock_guard lock(preview_mutex_);
                frame.copyTo(preview_frame_);
            }
        }

        if (cfg_.worker_sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.worker_sleep_ms));
        }
    }
    LOG_INFO("OpenCvVisionTracker: worker thread exit");
}

float OpenCvVisionTracker::estimateRange(const FrameDetection& det,
                                         float /*yaw_off_deg*/,
                                         float pitch_off_deg,
                                         const core::UAVState& own_state) const {
    float r = cfg_.fixed_range_m;
    switch (cfg_.range_mode) {
        case RangeMode::FIXED:
            r = cfg_.fixed_range_m;
            break;
        case RangeMode::SIZE: {
            if (det.radius_px > 1.0f) {
                const double cx0 = camera_->width() * 0.5;
                const double fx = cx0 / std::tan(cfg_.h_fov_deg * 0.5 * kDeg2Rad);
                r = static_cast<float>(cfg_.real_diameter_m * fx / (2.0 * det.radius_px));
            }
            break;
        }
        case RangeMode::ALTITUDE: {
            // Geometric ground-target estimate. Only meaningful when the camera
            // is roughly forward-pitched and the target is on the ground.
            const double total_pitch_deg = pitch_off_deg + own_state.attitude.pitch;
            const double t = std::tan(-total_pitch_deg * kDeg2Rad);
            if (t > 0.05 && own_state.position.altitude > 0.0) {
                r = static_cast<float>(own_state.position.altitude / t);
            } else {
                r = cfg_.fixed_range_m;
            }
            break;
        }
    }
    if (r < cfg_.min_range_m) r = cfg_.min_range_m;
    if (r > cfg_.max_range_m) r = cfg_.max_range_m;
    return r;
}

std::optional<PerceptionTarget> OpenCvVisionTracker::poll(const core::UAVState& own_state) {
    if (!running_.load()) return std::nullopt;

    std::optional<PerceptionTarget> snap;
    std::chrono::steady_clock::time_point stamp;
    {
        std::lock_guard lock(result_mutex_);
        snap = last_target_;
        stamp = last_target_time_;
    }

    if (!snap.has_value()) return std::nullopt;
    if (time::durationS(stamp, time::nowSteady()) > cfg_.detection_timeout_s) {
        return std::nullopt;
    }

    // For ALTITUDE range mode, recompute now that we have a real own_state.
    if (cfg_.range_mode == RangeMode::ALTITUDE && own_state.isValid()) {
        FrameDetection synth_det;
        synth_det.radius_px = 1.0f; // unused for ALTITUDE
        snap->c = estimateRange(synth_det, snap->a, snap->b, own_state);
    }

    return snap;
}

std::optional<PerceptionTarget> OpenCvVisionTracker::buildTarget(
    const FrameDetection& /*det*/, const core::UAVState& /*own_state*/) const {
    return std::nullopt; // unused (logic inlined in workerLoop for hot-path clarity)
}

void OpenCvVisionTracker::tickPreviewMainThread() {
    if (!cfg_.preview_enabled) return;

    cv::Mat frame;
    {
        std::lock_guard lock(preview_mutex_);
        if (preview_frame_.empty()) {
            // Pump waitKey anyway so any existing window stays responsive.
            if (preview_window_created_) cv::waitKey(1);
            return;
        }
        frame = preview_frame_;
        preview_frame_.release();
    }

    if (!preview_window_created_) {
        cv::namedWindow(cfg_.preview_title, cv::WINDOW_AUTOSIZE);
        // Force a real window event before the first imshow so Windows
        // actually realises the HWND (some drivers/themes need this).
        cv::setWindowProperty(cfg_.preview_title, cv::WND_PROP_TOPMOST, 1.0);
        preview_window_created_ = true;
    }

    cv::imshow(cfg_.preview_title, frame);
    cv::waitKey(1);
}

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
