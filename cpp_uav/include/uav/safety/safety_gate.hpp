#pragma once

#include "uav/safety/zone_checker.hpp"
#include "uav/world_model/world_model.hpp"

#include <memory>

namespace uav::safety {

class SafetyGate {
public:
    struct Config {
        double hss_buffer_m = 40.0;
        double boundary_buffer_m = 20.0;
        double min_altitude_m = 30.0;
        double max_altitude_m = 400.0;
        double max_velocity_ms = 40.0;
    };

    SafetyGate(std::shared_ptr<ZoneChecker> checker,
               std::shared_ptr<world_model::WorldModel> world_model,
               Config config = {});

    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

    bool validatePosition(const core::Position& target) const;
    bool validateSegment(const core::Position& from, const core::Position& target) const;
    bool validateEscapeSegment(const core::Position& from, const core::Position& target) const;
    bool validateVelocity(float vn, float ve, float vd) const;
    bool isInsideHSS(const core::Position& pos, bool include_buffer = false) const;
    bool isPositionSafeNow(const core::Position& pos) const;

    core::Position clampToSafe(const core::Position& target) const;
    void clampVelocity(float& vn, float& ve, float& vd) const;

private:
    std::shared_ptr<ZoneChecker> checker_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;
};

} // namespace uav::safety
