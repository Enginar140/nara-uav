#pragma once

#include "uav/safety/zone_checker.hpp"
#include "uav/world_model/world_model.hpp"
#include "uav/infra/event_bus.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace uav::safety {

class SafetyMonitor {
public:
    struct Config {
        double collision_critical_m = 30.0;
        double collision_high_m = 80.0;
        double collision_medium_m = 200.0;
        double collision_low_m = 500.0;
        double ttc_critical_s = 5.0;
        double ttc_high_s = 10.0;
        double boundary_warn_m = 30.0;
        double hss_buffer_m = 40.0;
        int heartbeat_timeout_ms = 5000;
    };

    SafetyMonitor(std::shared_ptr<world_model::WorldModel> world_model,
                  std::shared_ptr<infra::EventBus> event_bus,
                  Config config = {});
    ~SafetyMonitor();

    bool start();
    void stop();
    bool isRunning() const { return running_; }
    core::SafetyState getSafetyState() const;
    void setMonitoringRate(double hz) { rate_hz_ = hz; }
    void setConfig(const Config& cfg) { config_ = cfg; }

    void notifyHeartbeat();
    void notifyHeartbeatLost();

private:
    std::shared_ptr<world_model::WorldModel> world_model_;
    std::shared_ptr<infra::EventBus> event_bus_;
    ZoneChecker zone_checker_;
    Config config_;

    std::atomic<bool> running_{false};
    std::thread monitor_thread_;
    double rate_hz_ = 20.0;

    mutable std::mutex state_mutex_;
    core::SafetyState state_;

    std::chrono::steady_clock::time_point last_event_time_;
    std::chrono::milliseconds event_throttle_{500};
    std::atomic<bool> heartbeat_ok_{false};

    bool gps_low_warned_ = false;
    bool gps_critical_warned_ = false;
    bool battery_low_warned_ = false;
    bool battery_critical_warned_ = false;

    void monitorLoop();
    void checkBoundary(const core::UAVState& own, const core::Boundary& boundary);
    void checkHSS(const core::UAVState& own, const std::vector<core::HSSZone>& zones);
    void checkCollisions(const core::UAVState& own, const std::vector<core::UAVState>& opponents);
    void checkGPS(const core::UAVState& own);
    void checkBattery(const core::UAVState& own);
    void checkComm();
    void publishIfChanged();
};

} // namespace uav::safety
