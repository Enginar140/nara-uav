#pragma once

#ifdef UAV_HAS_OPENCV

#include "uav/perception/camera/icamera_source.hpp"

#include <opencv2/videoio.hpp>

#include <chrono>
#include <string>

namespace uav::perception {

/// Single OpenCV-backed camera that handles every cv::VideoCapture-supported
/// source. Picks behaviour by Config::type:
///   "device" -> integer index (USB/built-in webcam)
///   "rtsp"   -> rtsp://user:pass@host:port/stream URL
///   "http"   -> HTTP(S)/MJPEG URL (e.g. phone IP-Webcam apps, Axis cams)
///   "file"   -> path to a video file (loops if loop=true)
///   "auto"   -> tries device(0) then file_path then url, in that order
class OpenCvCamera : public ICameraSource {
public:
    struct Config {
        std::string type = "device";   // device|rtsp|http|file|auto
        int device_index = 0;
        std::string url;
        std::string file_path;
        int width = 640;
        int height = 480;
        double fps = 30.0;
        bool loop_file = true;
        int api_preference = cv::CAP_ANY;   // override (e.g. CAP_DSHOW, CAP_FFMPEG)
        int reconnect_delay_ms = 1000;       // wait before retrying a dropped stream
    };

    explicit OpenCvCamera(Config cfg);
    ~OpenCvCamera() override;

    bool open() override;
    void close() override;
    bool isOpen() const override;
    bool read(cv::Mat& out_frame, uint64_t& out_stamp_us) override;

    int width() const override { return actual_width_; }
    int height() const override { return actual_height_; }
    std::string description() const override;

private:
    bool tryOpenAs(const std::string& kind);
    void applyDesiredFormat();

    Config cfg_;
    cv::VideoCapture cap_;
    int actual_width_ = 0;
    int actual_height_ = 0;
    std::string opened_kind_;
    std::chrono::steady_clock::time_point last_failure_{};
};

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
