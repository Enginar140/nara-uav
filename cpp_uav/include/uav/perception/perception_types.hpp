#pragma once

#include "uav/core/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace uav::perception {

/// Frame in which a perception result expresses its target offset.
/// The companion's NavigationEngine dispatches to a different local-control
/// MAVLink wrapper based on this value, so each perception module must pick
/// the frame that matches the data it actually has.
enum class PerceptionFrame {
    BODY,       // a=forward_m, b=right_m, c=down_m (relative to current AHRS yaw)
    LOCAL_NED,  // a=north_m,   b=east_m,  c=down_m (relative to current location)
    POLAR,      // a=bearing_deg (from north), b=distance_m, c=altitude_offset_m (positive = climb)
    VISION_LOS  // a=yaw_offset_deg, b=pitch_offset_deg, c=range_m (camera-frame line-of-sight)
};

const char* toString(PerceptionFrame frame);

/// Output of a single perception tick. Modules return at most one of these;
/// downstream consumers (mission modes) translate it to a navigation command.
struct PerceptionTarget {
    PerceptionFrame frame = PerceptionFrame::BODY;
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;

    /// Optional commanded yaw (deg, true). Pass empty to leave heading
    /// uncommanded (firmware will steer toward the synthetic waypoint).
    std::optional<float> yaw_deg;

    /// 0..1 detection confidence. Mission modes filter on this.
    float confidence = 0.0f;

    uint64_t stamp_us = 0;
    std::string source_tag;     // e.g. "vision_tracker", "qr_detector_simple"
    std::string label;          // payload-specific (e.g. decoded QR text), optional
};

/// Wrapper bus event. has_lock=false means "no actionable target this tick";
/// consumers use it to decide between perception-driven and fallback paths.
struct PerceptionEvent {
    bool has_lock = false;
    PerceptionTarget target;    // valid only when has_lock=true
};

} // namespace uav::perception
