#ifdef UAV_HAS_OPENCV

#include "uav/perception/detector/round_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <vector>

namespace uav::perception {

RoundDetector::RoundDetector(Config cfg) : cfg_(std::move(cfg)) {
    // Hough requires odd kernel size and >= 3 to be meaningful.
    if (cfg_.gaussian_kernel > 0 && cfg_.gaussian_kernel % 2 == 0) {
        cfg_.gaussian_kernel += 1;
    }
}

std::optional<FrameDetection> RoundDetector::detect(const cv::Mat& bgr) {
    if (bgr.empty()) return std::nullopt;

    cv::Mat gray;

    if (cfg_.use_hsv_mask) {
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
                    cv::Scalar(cfg_.hsv_lo_h1, cfg_.hsv_lo_s, cfg_.hsv_lo_v),
                    cv::Scalar(cfg_.hsv_hi_h1, cfg_.hsv_hi_s, cfg_.hsv_hi_v), mask1);
        cv::inRange(hsv,
                    cv::Scalar(cfg_.hsv_lo_h2, cfg_.hsv_lo_s, cfg_.hsv_lo_v),
                    cv::Scalar(cfg_.hsv_hi_h2, cfg_.hsv_hi_s, cfg_.hsv_hi_v), mask2);
        cv::bitwise_or(mask1, mask2, mask);
        // Morph clean-up so isolated specks don't trick HoughCircles.
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7}));
        gray = mask; // Hough works on single-channel; mask is fine
    } else {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    }

    if (cfg_.gaussian_kernel >= 3) {
        cv::GaussianBlur(gray, gray, cv::Size(cfg_.gaussian_kernel, cfg_.gaussian_kernel), 0);
    }

    const int min_dim = std::min(bgr.cols, bgr.rows);
    const double min_dist = std::max(10.0, cfg_.min_dist_factor * min_dim);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
                     cfg_.dp, min_dist,
                     cfg_.param1, cfg_.param2,
                     cfg_.min_radius_px, cfg_.max_radius_px);

    if (circles.empty()) return std::nullopt;

    // Pick largest radius - usually the closest / most reliable target.
    auto best = std::max_element(circles.begin(), circles.end(),
        [](const cv::Vec3f& a, const cv::Vec3f& b) { return a[2] < b[2]; });

    FrameDetection det;
    det.cx_px = (*best)[0];
    det.cy_px = (*best)[1];
    det.radius_px = (*best)[2];

    // Confidence heuristic: scale base by relative radius (bigger = more
    // confident, capped). This keeps a single threshold meaningful across
    // resolutions without tuning per-camera.
    const float rel = std::min(1.0f, det.radius_px / static_cast<float>(min_dim) * 4.0f);
    det.confidence = std::min(1.0f, cfg_.base_confidence + 0.3f * rel);
    det.label = "circle";

    if (cfg_.draw_overlay) {
        cv::circle(const_cast<cv::Mat&>(bgr),
                   cv::Point(static_cast<int>(det.cx_px), static_cast<int>(det.cy_px)),
                   static_cast<int>(det.radius_px), cv::Scalar(0, 255, 0), 2);
        cv::circle(const_cast<cv::Mat&>(bgr),
                   cv::Point(static_cast<int>(det.cx_px), static_cast<int>(det.cy_px)),
                   3, cv::Scalar(0, 0, 255), -1);
    }

    return det;
}

} // namespace uav::perception

#endif // UAV_HAS_OPENCV
