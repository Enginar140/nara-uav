#pragma once

#ifdef UAV_HAS_OPENCV

#include "uav/perception/detector/iframe_detector.hpp"

#include <opencv2/core.hpp>

namespace uav::perception {

/// HoughCircles-based round-object detector. Designed as a quick first-pass
/// vision target so the rest of the perception pipeline (LOS computation,
/// NavigationEngine dispatch) can be exercised end-to-end without bringing
/// in a heavy ML model.
///
/// Optional HSV pre-filter masks the frame to a colour band before the
/// transform - useful when a known coloured ball is the target. Disable
/// (use_hsv_mask=false) to detect any sufficiently round bright shape.
class RoundDetector : public IFrameDetector {
public:
    struct Config {
        // Hough transform parameters
        double dp = 1.2;
        double min_dist_factor = 0.25;  // min distance between centers, fraction of min(w,h)
        double param1 = 100.0;          // Canny upper threshold
        double param2 = 30.0;           // accumulator threshold (lower = more circles)
        int min_radius_px = 15;
        int max_radius_px = 200;

        // Pre-processing
        int gaussian_kernel = 9;        // must be odd; 0 disables
        bool use_hsv_mask = false;
        // HSV thresholds (OpenCV: H 0-179, S 0-255, V 0-255).
        // Default = bright red (two ranges merged inside detect()).
        int hsv_lo_h1 = 0,   hsv_hi_h1 = 10;
        int hsv_lo_h2 = 170, hsv_hi_h2 = 179;
        int hsv_lo_s = 100,  hsv_hi_s = 255;
        int hsv_lo_v = 80,   hsv_hi_v = 255;

        // Output
        float base_confidence = 0.7f;   // assigned when a circle is found
        bool draw_overlay = false;      // mutates the input frame for preview
    };

    explicit RoundDetector(Config cfg = {});

    std::string id() const override { return "round"; }
    std::optional<FrameDetection> detect(const cv::Mat& bgr) override;

private:
    Config cfg_;
};

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
