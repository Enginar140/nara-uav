#ifdef UAV_HAS_OPENCV

#include "uav/perception/camera/opencv_camera.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <opencv2/imgproc.hpp>

#include <thread>

namespace uav::perception {

OpenCvCamera::OpenCvCamera(Config cfg) : cfg_(std::move(cfg)) {}

OpenCvCamera::~OpenCvCamera() { close(); }

bool OpenCvCamera::open() {
    if (cap_.isOpened()) return true;

    if (cfg_.type == "auto") {
        if (tryOpenAs("device")) return true;
        if (!cfg_.file_path.empty() && tryOpenAs("file")) return true;
        if (!cfg_.url.empty() && tryOpenAs("rtsp")) return true;
        LOG_ERROR("OpenCvCamera: 'auto' failed - no source available");
        return false;
    }

    if (!tryOpenAs(cfg_.type)) {
        LOG_ERROR("OpenCvCamera: failed to open type='{}' ({})", cfg_.type, description());
        return false;
    }
    return true;
}

bool OpenCvCamera::tryOpenAs(const std::string& kind) {
    bool ok = false;
    if (kind == "device") {
        ok = cap_.open(cfg_.device_index, cfg_.api_preference);
    } else if (kind == "rtsp" || kind == "http") {
        if (cfg_.url.empty()) {
            LOG_WARN("OpenCvCamera: type='{}' but url is empty", kind);
            return false;
        }
        ok = cap_.open(cfg_.url, cfg_.api_preference);
    } else if (kind == "file") {
        if (cfg_.file_path.empty()) {
            LOG_WARN("OpenCvCamera: type='file' but file_path is empty");
            return false;
        }
        ok = cap_.open(cfg_.file_path, cfg_.api_preference);
    } else {
        LOG_ERROR("OpenCvCamera: unknown type '{}'", kind);
        return false;
    }

    if (!ok) return false;

    opened_kind_ = kind;
    applyDesiredFormat();

    actual_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    actual_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (actual_width_ <= 0) actual_width_ = cfg_.width;
    if (actual_height_ <= 0) actual_height_ = cfg_.height;

    LOG_INFO("OpenCvCamera: opened {} ({}x{} @ requested {:.1f} fps)",
             description(), actual_width_, actual_height_, cfg_.fps);
    return true;
}

void OpenCvCamera::applyDesiredFormat() {
    if (cfg_.width > 0)  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
    if (cfg_.height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    if (cfg_.fps > 0.0)  cap_.set(cv::CAP_PROP_FPS, cfg_.fps);
    // Reduce internal buffering so we always read the freshest frame
    // (driver may ignore - silently best-effort).
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
}

void OpenCvCamera::close() {
    if (cap_.isOpened()) {
        cap_.release();
        LOG_INFO("OpenCvCamera: closed ({})", description());
    }
    opened_kind_.clear();
}

bool OpenCvCamera::isOpen() const { return cap_.isOpened(); }

bool OpenCvCamera::read(cv::Mat& out_frame, uint64_t& out_stamp_us) {
    if (!cap_.isOpened()) {
        // throttle reconnect attempts
        auto now = std::chrono::steady_clock::now();
        if (cfg_.reconnect_delay_ms > 0 &&
            now - last_failure_ < std::chrono::milliseconds(cfg_.reconnect_delay_ms)) {
            return false;
        }
        last_failure_ = now;
        if (!open()) return false;
    }

    if (!cap_.read(out_frame) || out_frame.empty()) {
        if (opened_kind_ == "file" && cfg_.loop_file) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (cap_.read(out_frame) && !out_frame.empty()) {
                out_stamp_us = time::nowUs();
                return true;
            }
        }
        LOG_DEBUG("OpenCvCamera: read failure - reconnecting");
        cap_.release();
        last_failure_ = std::chrono::steady_clock::now();
        return false;
    }

    out_stamp_us = time::nowUs();
    return true;
}

std::string OpenCvCamera::description() const {
    if (opened_kind_ == "device") {
        return "device:" + std::to_string(cfg_.device_index);
    }
    if (opened_kind_ == "file") {
        return "file:" + cfg_.file_path;
    }
    if (opened_kind_ == "rtsp" || opened_kind_ == "http") {
        return opened_kind_ + ":" + cfg_.url;
    }
    return cfg_.type + "(unopened)";
}

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
