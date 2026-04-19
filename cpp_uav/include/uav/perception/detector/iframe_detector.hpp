#pragma once

#include <optional>
#include <string>

#ifdef UAV_HAS_OPENCV
#include <opencv2/core.hpp>
#endif

namespace uav::perception {

/// Single detection result expressed in image-pixel coordinates.
/// Origin is top-left, x grows right, y grows down (OpenCV convention).
struct FrameDetection {
    float cx_px = 0.0f;
    float cy_px = 0.0f;
    float radius_px = 0.0f;     // for circular/blob targets; bbox half-diagonal otherwise
    float confidence = 0.0f;
    std::string label;          // detector-specific (e.g. "circle", "qr:HELLO")
};

/// Pure-CV detector: takes a frame, returns at most one best detection.
/// Implementations MUST be re-entrant safe (perception engine may call
/// detect() from a single worker thread but lifecycle calls can come from
/// the main thread).
class IFrameDetector {
public:
    virtual ~IFrameDetector() = default;

    virtual std::string id() const = 0;

#ifdef UAV_HAS_OPENCV
    virtual std::optional<FrameDetection> detect(const cv::Mat& bgr) = 0;
#endif
};

} // namespace uav::perception
