#pragma once

#include <cstdint>
#include <string>

#ifdef UAV_HAS_OPENCV
#include <opencv2/core.hpp>
#endif

namespace uav::perception {

/// Abstract camera source. Implementations adapt different backends
/// (cv::VideoCapture for USB/RTSP/HTTP/file, GStreamer pipelines, ROS topics)
/// behind a single grabFrame() call so detectors stay backend-agnostic.
///
/// Lifecycle: open() once (typically at module start()), read() per tick,
/// close() at stop(). Implementations must be safe to close() even when not
/// open and to read() repeatedly after a transient failure.
class ICameraSource {
public:
    virtual ~ICameraSource() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

#ifdef UAV_HAS_OPENCV
    /// Grab the next frame. Returns false on transient failure (caller may
    /// retry). out_stamp_us is filled with a steady-clock-derived monotonic
    /// timestamp so detector age can be computed even when the source has
    /// no native timestamp.
    virtual bool read(cv::Mat& out_frame, uint64_t& out_stamp_us) = 0;
#endif

    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual std::string description() const = 0;
};

} // namespace uav::perception
