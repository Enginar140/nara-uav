#pragma once

#include <chrono>
#include <cstdint>

namespace uav::time {

inline uint64_t nowUs() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
}

inline std::chrono::steady_clock::time_point nowSteady() {
    return std::chrono::steady_clock::now();
}

inline double elapsedMs(std::chrono::steady_clock::time_point start) {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - start).count();
}

inline double elapsedS(std::chrono::steady_clock::time_point start) {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start).count();
}

inline double durationS(std::chrono::steady_clock::time_point from,
                         std::chrono::steady_clock::time_point to) {
    return std::chrono::duration<double>(to - from).count();
}

} // namespace uav::time
