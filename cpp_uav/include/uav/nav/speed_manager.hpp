#pragma once

#include <algorithm>

namespace uav::nav {

class SpeedManager {
public:
    struct Config {
        float cruise = 20.0f;
        float max_speed = 35.0f;
        float min_speed = 12.0f;
        float patrol = 18.0f;
        float pursuit = 30.0f;
        float dive = 25.0f;
    };

    explicit SpeedManager(Config config = {}) : config_(config) {}

    float cruiseSpeed() const { return config_.cruise; }
    float patrolSpeed() const { return config_.patrol; }
    float pursuitSpeed() const { return config_.pursuit; }
    float diveSpeed() const { return config_.dive; }

    float clamp(float speed) const {
        return std::clamp(speed, config_.min_speed, config_.max_speed);
    }

    void setConfig(const Config& c) { config_ = c; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
};

} // namespace uav::nav
