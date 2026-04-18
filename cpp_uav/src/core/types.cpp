#include "uav/core/types.hpp"

#include <chrono>

namespace uav::core {

double UAVState::ageSeconds() const {
    if (timestamp_us == 0) return INFINITY;
    auto now = std::chrono::system_clock::now();
    auto now_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
    return static_cast<double>(now_us - timestamp_us) / 1'000'000.0;
}

Position Boundary::getCenter() const {
    if (vertices.empty()) return {};
    double lat_sum = 0, lon_sum = 0, alt_sum = 0;
    for (const auto& v : vertices) {
        lat_sum += v.latitude;
        lon_sum += v.longitude;
        alt_sum += v.altitude;
    }
    double n = static_cast<double>(vertices.size());
    return {lat_sum / n, lon_sum / n, alt_sum / n};
}

std::string toString(MissionMode mode) {
    switch (mode) {
        case MissionMode::ARDUPILOT: return "ARDUPILOT";
        case MissionMode::PATROL:    return "PATROL";
        case MissionMode::FOLLOW:    return "FOLLOW";
        case MissionMode::QR_DIVE:   return "QR_DIVE";
    }
    return "UNKNOWN";
}

std::string toString(GoalType type) {
    switch (type) {
        case GoalType::NONE:         return "NONE";
        case GoalType::PATROL:       return "PATROL";
        case GoalType::TRACK_TARGET: return "TRACK_TARGET";
        case GoalType::DIVE_TARGET:  return "DIVE_TARGET";
        case GoalType::ESCAPE:       return "ESCAPE";
    }
    return "UNKNOWN";
}

std::string toString(RiskLevel level) {
    switch (level) {
        case RiskLevel::NONE:     return "NONE";
        case RiskLevel::LOW:      return "LOW";
        case RiskLevel::MEDIUM:   return "MEDIUM";
        case RiskLevel::HIGH:     return "HIGH";
        case RiskLevel::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}

std::string toString(BoundaryStatus status) {
    switch (status) {
        case BoundaryStatus::INSIDE:        return "INSIDE";
        case BoundaryStatus::NEAR_BOUNDARY: return "NEAR_BOUNDARY";
        case BoundaryStatus::OUTSIDE:       return "OUTSIDE";
    }
    return "UNKNOWN";
}

} // namespace uav::core
