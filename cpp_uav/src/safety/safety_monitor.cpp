#include "uav/safety/safety_monitor.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>

namespace uav::safety {

SafetyMonitor::SafetyMonitor(std::shared_ptr<world_model::WorldModel> world_model,
                             std::shared_ptr<infra::EventBus> event_bus,
                             Config config)
    : world_model_(std::move(world_model)),
      event_bus_(std::move(event_bus)),
      config_(config) {}

SafetyMonitor::~SafetyMonitor() { stop(); }

bool SafetyMonitor::start() {
    if (running_) return true;
    running_ = true;
    monitor_thread_ = std::thread(&SafetyMonitor::monitorLoop, this);
    return true;
}

void SafetyMonitor::stop() {
    running_ = false;
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

core::SafetyState SafetyMonitor::getSafetyState() const {
    std::lock_guard lock(state_mutex_);
    return state_;
}

void SafetyMonitor::notifyHeartbeat() { heartbeat_ok_ = true; }
void SafetyMonitor::notifyHeartbeatLost() { heartbeat_ok_ = false; }

void SafetyMonitor::monitorLoop() {
    while (running_) {
        auto own = world_model_->getOwnState();

        if (own.isValid()) {
            std::lock_guard lock(state_mutex_);

            state_.collision_risk = core::RiskLevel::NONE;
            state_.active_threats.clear();
            state_.hss_violations.clear();

            auto boundary = world_model_->getBoundary();
            auto hss_zones = world_model_->getActiveHSSZones();
            auto opponents = world_model_->getAllOpponents();

            if (boundary.has_value()) {
                checkBoundary(own, *boundary);
            }
            checkHSS(own, hss_zones);
            checkCollisions(own, opponents);
            checkGPS(own);
            checkBattery(own);
            checkComm();
        }

        publishIfChanged();

        auto sleep_ms = static_cast<int>(1000.0 / rate_hz_);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

void SafetyMonitor::checkBoundary(const core::UAVState& own,
                                  const core::Boundary& boundary) {
    if (!zone_checker_.isInsideBoundary(own.position, boundary)) {
        state_.boundary_status = core::BoundaryStatus::OUTSIDE;
        event_bus_->publish(infra::EventType::BOUNDARY_VIOLATION);
        return;
    }

    double dist = zone_checker_.distanceToBoundary(own.position, boundary);
    if (dist < config_.boundary_warn_m) {
        state_.boundary_status = core::BoundaryStatus::NEAR_BOUNDARY;
    } else {
        state_.boundary_status = core::BoundaryStatus::INSIDE;
    }
}

void SafetyMonitor::checkHSS(const core::UAVState& own,
                              const std::vector<core::HSSZone>& zones) {
    for (const auto& zone : zones) {
        double dist = geo::haversineDistance(own.position, zone.center);
        if (dist < zone.radius) {
            state_.hss_violations.push_back(zone.id);
            event_bus_->publish(infra::EventType::HSS_VIOLATION, zone.id);
        }
    }
}

void SafetyMonitor::checkCollisions(const core::UAVState& own,
                                    const std::vector<core::UAVState>& opponents) {
    for (const auto& opp : opponents) {
        double dist = geo::haversineDistance(own.position, opp.position);

        double rel_vn = opp.velocity.north - own.velocity.north;
        double rel_ve = opp.velocity.east - own.velocity.east;
        double bearing_rad = geo::bearing(own.position, opp.position) * geo::kDegToRad;
        double closing_speed = -(rel_vn * std::cos(bearing_rad) + rel_ve * std::sin(bearing_rad));
        double ttc = (closing_speed > 0.5) ? dist / closing_speed : INFINITY;

        core::RiskLevel risk = core::RiskLevel::NONE;
        if (dist < config_.collision_critical_m || ttc < config_.ttc_critical_s) {
            risk = core::RiskLevel::CRITICAL;
        } else if (dist < config_.collision_high_m || ttc < config_.ttc_high_s) {
            risk = core::RiskLevel::HIGH;
        } else if (dist < config_.collision_medium_m) {
            risk = core::RiskLevel::MEDIUM;
        } else if (dist < config_.collision_low_m) {
            risk = core::RiskLevel::LOW;
        }

        if (risk > state_.collision_risk) {
            state_.collision_risk = risk;
        }

        if (risk >= core::RiskLevel::MEDIUM) {
            core::Threat threat;
            threat.aircraft_id = opp.id;
            threat.distance = dist;
            threat.bearing = geo::bearing(own.position, opp.position);
            threat.closing_speed = closing_speed;
            threat.time_to_collision = ttc;
            threat.risk_level = risk;
            state_.active_threats.push_back(threat);
        }
    }

    if (state_.collision_risk >= core::RiskLevel::HIGH && !state_.active_threats.empty()) {
        event_bus_->publish(infra::EventType::COLLISION_WARNING, state_.active_threats.front());
    }
}

void SafetyMonitor::checkGPS(const core::UAVState& own) {
    bool has_gps_data = own.gps_fix_type > 0 || own.gps_hdop < 9000;
    if (!has_gps_data) return;

    if (own.gps_fix_type <= 1 && own.gps_hdop > 500) {
        if (!gps_critical_warned_) {
            LOG_WARN("SafetyMonitor: GPS CRITICAL fix={} hdop={}", own.gps_fix_type, own.gps_hdop);
            event_bus_->publish(infra::EventType::GPS_QUALITY_CRITICAL);
            gps_critical_warned_ = true;
        }
    } else if (own.gps_fix_type <= 2 && own.gps_hdop > 300) {
        if (!gps_low_warned_) {
            LOG_WARN("SafetyMonitor: GPS LOW fix={} hdop={}", own.gps_fix_type, own.gps_hdop);
            event_bus_->publish(infra::EventType::GPS_QUALITY_LOW);
            gps_low_warned_ = true;
        }
        gps_critical_warned_ = false;
    } else {
        gps_low_warned_ = false;
        gps_critical_warned_ = false;
    }
}

void SafetyMonitor::checkBattery(const core::UAVState& own) {
    if (own.battery_remaining < 0) return;

    if (own.battery_remaining < 10) {
        if (!battery_critical_warned_) {
            event_bus_->publish(infra::EventType::BATTERY_CRITICAL);
            battery_critical_warned_ = true;
        }
    } else if (own.battery_remaining < 25) {
        if (!battery_low_warned_) {
            event_bus_->publish(infra::EventType::BATTERY_LOW);
            battery_low_warned_ = true;
        }
        battery_critical_warned_ = false;
    } else {
        battery_low_warned_ = false;
        battery_critical_warned_ = false;
    }
}

void SafetyMonitor::checkComm() {
    bool was_lost = state_.comm_loss;
    state_.comm_loss = !heartbeat_ok_.load();

    if (state_.comm_loss && !was_lost) {
        // Failsafe layer subscribes to HEARTBEAT_TIMEOUT; emit both so any
        // listener gets a single canonical "we lost the link" notification.
        event_bus_->publish(infra::EventType::HEARTBEAT_TIMEOUT);
        event_bus_->publish(infra::EventType::MAVLINK_DISCONNECTED);
        LOG_WARN("SafetyMonitor: comm loss detected");
    } else if (!state_.comm_loss && was_lost) {
        event_bus_->publish(infra::EventType::HEARTBEAT_RECOVERED);
        event_bus_->publish(infra::EventType::MAVLINK_CONNECTED);
        LOG_INFO("SafetyMonitor: comm recovered");
    }
}

void SafetyMonitor::publishIfChanged() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_event_time_ < event_throttle_) return;

    std::lock_guard lock(state_mutex_);
    if (!state_.isSafe()) {
        event_bus_->publish(infra::EventType::SAFETY_VIOLATION, state_);
        last_event_time_ = now;
    }
}

} // namespace uav::safety
