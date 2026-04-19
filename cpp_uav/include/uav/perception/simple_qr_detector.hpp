#pragma once

#include "uav/perception/iperception_module.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace uav::perception {

/// Reference QR detector. Stand-in for a real ZBar/zxing-cpp pipeline.
///
/// Two output modes:
///  - VISION_LOS: kameradan gelen yaw/pitch açısı; range tahmini varsa eklenir.
///    QR_DIVE terminal fazında pursueVisionTarget'a dispatch edilir.
///  - LOCAL_NED: QR'ın bilinen world-pose'u kullanılarak öz konuma göre
///    kuzey/doğu/aşağı offset üretilir. flyLocalOffset'a dispatch edilir.
///
/// Hangi mode'un kullanılacağı injected detection'ın frame alanıyla belirlenir.
class SimpleQRDetector : public IPerceptionModule {
public:
    struct Config {
        double detection_timeout_s = 0.7;
        float default_range_m = 60.0f;
        float min_confidence = 0.6f;
        std::string expected_payload; // boş = herhangi bir QR kabul
    };

    explicit SimpleQRDetector(Config config = {});

    /// Inject a camera-frame detection (yaw_offset / pitch_offset derece, range metre).
    void injectVisionLOS(float yaw_offset_deg, float pitch_offset_deg,
                          float range_m, float confidence = 0.9f,
                          const std::string& payload = "");

    /// Inject a world-pose detection (north/east/down ofset metre cinsinden).
    void injectLocalOffset(float north_m, float east_m, float down_m,
                            float confidence = 0.9f,
                            const std::string& payload = "");

    void clear();

    bool hasFreshDetection() const;
    std::optional<std::string> lastPayload() const;

    std::string id() const override { return "qr_simple"; }
    void start() override;
    void stop() override;
    std::optional<PerceptionTarget> poll(const core::UAVState& own_state) override;

private:
    Config config_;

    mutable std::mutex mutex_;
    std::optional<PerceptionTarget> injected_;
    std::chrono::steady_clock::time_point injected_at_;
    std::optional<std::string> last_payload_;

    std::atomic<bool> running_{false};
};

} // namespace uav::perception
