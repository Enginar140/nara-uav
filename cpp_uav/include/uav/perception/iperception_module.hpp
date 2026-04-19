#pragma once

#include "uav/perception/perception_types.hpp"
#include "uav/core/types.hpp"

#include <optional>
#include <string>

namespace uav::perception {

/// Drop-in interface for vision/QR/AprilTag/etc detection algorithms.
/// Each tick the PerceptionEngine calls poll() with the latest UAV state and
/// expects either an actionable PerceptionTarget or std::nullopt.
///
/// Modules MUST be cheap to construct, idempotent across start()/stop(), and
/// thread-safe with respect to concurrent enable/disable from the engine.
class IPerceptionModule {
public:
    virtual ~IPerceptionModule() = default;

    /// Stable identifier used to look up the module from config (yaml).
    /// Convention: lowercase, snake_case (e.g. "vision_tracker", "qr_simple").
    virtual std::string id() const = 0;

    /// Called once when the engine activates this module. Do heavy setup here
    /// (camera open, model load) so swap is fast.
    virtual void start() = 0;

    /// Called when the engine deactivates this module (mode change, shutdown).
    virtual void stop() = 0;

    /// Run one detection cycle. Implementations should be non-blocking and
    /// return quickly (<= one PerceptionEngine tick budget).
    virtual std::optional<PerceptionTarget> poll(const core::UAVState& own_state) = 0;
};

} // namespace uav::perception
