#include "uav/nav/navigation_engine.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>

namespace uav::nav {

NavigationEngine::NavigationEngine(
    std::shared_ptr<PathPlanner> planner,
    std::shared_ptr<comm::FlightCommander> commander,
    std::shared_ptr<safety::SafetyGate> safety_gate,
    std::shared_ptr<SpeedManager> speed_manager,
    std::shared_ptr<world_model::WorldModel> world_model)
    : planner_(std::move(planner)),
      commander_(std::move(commander)),
      safety_gate_(std::move(safety_gate)),
      speed_manager_(std::move(speed_manager)),
      world_model_(std::move(world_model)) {}

bool NavigationEngine::planRoute(const core::Position& goal, float airspeed) {
    std::lock_guard lock(nav_mutex_);
    return planRouteLocked(goal, airspeed);
}

bool NavigationEngine::planRouteLocked(const core::Position& goal, float airspeed) {
    if (shouldBlockNormalCommandLocked()) return false;

    auto now = time::nowSteady();
    if (now - last_plan_time_ < min_plan_interval_ &&
        !chain_.isEmpty() && !chain_.isComplete()) {
        return true;
    }

    auto boundary = world_model_->getBoundary();
    if (!boundary.has_value() || !boundary->isValid()) {
        static auto last_warn = std::chrono::steady_clock::time_point{};
        auto now_w = time::nowSteady();
        if (time::durationS(last_warn, now_w) > 10.0) {
            LOG_WARN("NavigationEngine: no boundary, cannot plan");
            last_warn = now_w;
        }
        return false;
    }

    auto own = world_model_->getOwnState();
    if (!own.isValid()) {
        LOG_WARN("NavigationEngine: no own-state telemetry, cannot plan");
        return false;
    }

    if (safety_gate_->isInsideHSS(own.position, true)) {
        LOG_WARN("NavigationEngine: inside HSS buffer, refusing normal route plan until escape completes");
        return false;
    }
    auto zones = world_model_->getActiveHSSZones();

    core::Position safe_goal = goal;
    if (!safety_gate_->validatePosition(safe_goal)) {
        safe_goal = safety_gate_->clampToSafe(safe_goal);
        LOG_WARN("NavigationEngine: route goal unsafe, clamped before planning");
    }

    auto result = planner_->plan(own.position, safe_goal, *boundary, zones);
    if (!result.success) {
        static auto last_plan_warn = std::chrono::steady_clock::time_point{};
        auto now_w = time::nowSteady();
        if (time::durationS(last_plan_warn, now_w) > 5.0) {
            LOG_WARN("NavigationEngine: planning failed - {}", result.error);
            last_plan_warn = now_w;
        }
        return false;
    }

    float speed = (airspeed > 0) ? airspeed : speed_manager_->cruiseSpeed();

    std::vector<Waypoint> wp_chain;
    wp_chain.reserve(result.waypoints.size());

    for (size_t i = 0; i < result.waypoints.size(); ++i) {
        if (i == 0 && result.waypoints.size() > 1) continue;

        Waypoint wp;
        wp.position = result.waypoints[i];
        wp.airspeed = speed;
        wp.label = "WP" + std::to_string(i);
        wp_chain.push_back(wp);
    }

    if (wp_chain.empty()) {
        Waypoint wp;
        wp.position = safe_goal;
        wp.airspeed = speed;
        wp.label = "GOAL";
        wp_chain.push_back(wp);
    }

    chain_.replace(std::move(wp_chain));

    LOG_INFO("NavigationEngine: route planned, {} waypoints, {:.0f}m",
             chain_.size(), result.total_distance_m);

    bool sent = sendCurrentWaypointLocked();
    if (sent) last_plan_time_ = now;
    return sent;
}

ChainStatus NavigationEngine::update(const core::Position& own_pos) {
    std::lock_guard lock(nav_mutex_);
    return updateLocked(own_pos);
}

ChainStatus NavigationEngine::updateLocked(const core::Position& own_pos) {
    if (shouldBlockNormalCommandLocked()) return ChainStatus::IN_PROGRESS;
    if (chain_.isEmpty()) return ChainStatus::EMPTY;

    auto status = chain_.update(own_pos);

    if (status == ChainStatus::WAYPOINT_REACHED) {
        if (!shouldThrottleCommand()) {
            sendCurrentWaypointLocked();
        }
    } else if (status == ChainStatus::IN_PROGRESS) {
        auto wp = chain_.currentTarget();
        if (wp.has_value()) {
            double dist = geo::haversineDistance(own_pos, wp->position);
            double lookahead_radius = wp->acceptance_radius_m * 1.5;
            if (dist < lookahead_radius && chain_.peekNext().has_value()) {
                if (!shouldThrottleCommand()) {
                    auto next = chain_.peekNext();
                    core::Position target = next->position;
                    if (!safety_gate_->validatePosition(target)) {
                        target = safety_gate_->clampToSafe(target);
                    }
                    if (!validateProjectedCommandLocked(target, false)) {
                        LOG_WARN("NavigationEngine: look-ahead segment unsafe, forcing route revalidation");
                        (void)revalidateRouteLocked();
                        return status;
                    }
                    bool sent = commander_->flyToPosition(target, next->airspeed);
                    last_cmd_time_ = time::nowSteady();
                    if (sent) {
                        last_sent_target_ = target;
                        LOG_DEBUG("NavigationEngine: look-ahead sent next WP at {:.0f}m from current", dist);
                    }
                }
            } else if (!shouldThrottleCommand()) {
                sendCurrentWaypointLocked();
            }
        }
    }

    return status;
}

bool NavigationEngine::flyDirect(const core::Position& target, float airspeed) {
    std::lock_guard lock(nav_mutex_);
    return flyDirectLocked(target, airspeed, false);
}

bool NavigationEngine::flyEscapeDirect(const core::Position& target, float airspeed) {
    std::lock_guard lock(nav_mutex_);
    chain_.clear();
    last_sent_target_.reset();
    bool sent = flyDirectLocked(target, airspeed, true);
    if (sent) {
        escape_active_ = true;
        escape_target_ = target;
    }
    return sent;
}

bool NavigationEngine::flyDirectLocked(const core::Position& target, float airspeed, bool escape_command) {
    if (!escape_command && shouldBlockNormalCommandLocked()) return false;
    if (!escape_command && shouldThrottleCommand()) return false;

    float speed = (airspeed > 0) ? airspeed : speed_manager_->cruiseSpeed();
    core::Position safe_target = target;

    if (!safety_gate_->validatePosition(target)) {
        safe_target = safety_gate_->clampToSafe(target);
        LOG_DEBUG("NavigationEngine: flyDirect clamped to safe position");
    }

    if (!validateProjectedCommandLocked(safe_target, escape_command)) {
        if (!escape_command) {
            auto own = currentOwnPositionLocked();
            if (own.has_value() &&
                !safety_gate_->isInsideHSS(*own, true) &&
                safety_gate_->validatePosition(safe_target)) {
                LOG_WARN("NavigationEngine: direct segment unsafe, replanning around HSS/boundary");
                chain_.clear();
                last_plan_time_ = {};
                return planRouteLocked(safe_target, speed);
            }
        }
        LOG_WARN("NavigationEngine: refusing unsafe direct command");
        return false;
    }

    bool sent = commander_->flyToPosition(safe_target, speed);
    last_cmd_time_ = time::nowSteady();
    if (sent) last_sent_target_ = safe_target;
    return sent;
}

bool NavigationEngine::flyLocalPosition(float north, float east, float down) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return false;
    if (!std::isfinite(north) || !std::isfinite(east) || !std::isfinite(down)) return false;

    auto target = localOffsetToPosition(own, north, east, down);
    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe legacy local-position command");
        return false;
    }

    bool sent = commander_->flyToLocalPosition(north, east, down);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

bool NavigationEngine::flyLocalVelocity(float vn, float ve, float vd, float yaw_rate) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;

    if (!safety_gate_->validateVelocity(vn, ve, vd)) {
        safety_gate_->clampVelocity(vn, ve, vd);
        LOG_DEBUG("NavigationEngine: velocity clamped by safety gate");
    }

    bool sent = commander_->setLocalVelocity(vn, ve, vd, yaw_rate);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

// =====================================================================
// Local control wrappers (feature/local-control fork).
// All four delegate to FlightCommander after command-rate throttling.
// =====================================================================

bool NavigationEngine::flyBodyOffset(float forward_m, float right_m, float down_m, float yaw_deg) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return false;
    if (!std::isfinite(forward_m) || !std::isfinite(right_m) || !std::isfinite(down_m)) return false;

    auto target = bodyOffsetToPosition(own, forward_m, right_m, down_m);
    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe body-offset command");
        return false;
    }

    bool sent = commander_->flyToBodyOffset(forward_m, right_m, down_m, yaw_deg);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

bool NavigationEngine::flyLocalOffset(float north_m, float east_m, float down_m, float yaw_deg) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return false;
    if (!std::isfinite(north_m) || !std::isfinite(east_m) || !std::isfinite(down_m)) return false;

    auto target = localOffsetToPosition(own, north_m, east_m, down_m);
    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe local-offset command");
        return false;
    }

    bool sent = commander_->flyToLocalOffset(north_m, east_m, down_m, yaw_deg);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

bool NavigationEngine::flyBearing(float bearing_deg, float distance_m, float altitude_offset_m, float yaw_deg) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return false;
    if (!std::isfinite(bearing_deg) || !std::isfinite(distance_m) ||
        !std::isfinite(altitude_offset_m) || distance_m < 0.0f) {
        return false;
    }

    const float br = bearing_deg * static_cast<float>(geo::kDegToRad);
    const float north = std::cos(br) * distance_m;
    const float east = std::sin(br) * distance_m;
    const float down = -altitude_offset_m;
    auto target = localOffsetToPosition(own, north, east, down);
    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe bearing-offset command");
        return false;
    }

    bool sent = commander_->flyToBearingOffset(bearing_deg, distance_m, altitude_offset_m, yaw_deg);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

bool NavigationEngine::pursueVisionTarget(float yaw_offset_deg, float pitch_offset_deg, float range_m, float yaw_deg) {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    if (shouldThrottleCommand()) return false;
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return false;
    if (!std::isfinite(yaw_offset_deg) || !std::isfinite(pitch_offset_deg) ||
        !std::isfinite(range_m) || range_m <= 0.0f) {
        return false;
    }

    const float yo = yaw_offset_deg * static_cast<float>(geo::kDegToRad);
    const float po = pitch_offset_deg * static_cast<float>(geo::kDegToRad);
    const float fwd = std::cos(po) * std::cos(yo) * range_m;
    const float right = std::cos(po) * std::sin(yo) * range_m;
    const float down = std::sin(po) * range_m;
    auto target = bodyOffsetToPosition(own, fwd, right, down);
    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe vision target command");
        return false;
    }

    bool sent = commander_->flyToVisionTarget(yaw_offset_deg, pitch_offset_deg, range_m, yaw_deg);
    last_cmd_time_ = time::nowSteady();
    return sent;
}

bool NavigationEngine::dispatchPerceptionTarget(const perception::PerceptionTarget& t) {
    const float yaw = t.yaw_deg.value_or(std::numeric_limits<float>::quiet_NaN());
    switch (t.frame) {
        case perception::PerceptionFrame::BODY:
            return flyBodyOffset(t.a, t.b, t.c, yaw);
        case perception::PerceptionFrame::LOCAL_NED:
            return flyLocalOffset(t.a, t.b, t.c, yaw);
        case perception::PerceptionFrame::POLAR:
            return flyBearing(t.a, t.b, t.c, yaw);
        case perception::PerceptionFrame::VISION_LOS:
            return pursueVisionTarget(t.a, t.b, t.c, yaw);
    }
    LOG_WARN("NavigationEngine: dispatchPerceptionTarget unknown frame");
    return false;
}

bool NavigationEngine::releaseGuidedHold() {
    std::lock_guard lock(nav_mutex_);
    if (shouldBlockNormalCommandLocked()) return false;
    // Bypass throttle: this is a one-shot reset that must reach the firmware
    // even if a fast burst of commands just went out.
    last_cmd_time_ = time::nowSteady();
    last_sent_target_.reset();
    chain_.clear();
    return commander_->flyToBodyOffset(0.0f, 0.0f, 0.0f,
                                        std::numeric_limits<float>::quiet_NaN());
}

void NavigationEngine::clearRoute() {
    std::lock_guard lock(nav_mutex_);
    chain_.clear();
    last_sent_target_.reset();
}

bool NavigationEngine::hasActiveRoute() const {
    std::lock_guard lock(nav_mutex_);
    return !chain_.isEmpty() && !chain_.isComplete();
}

bool NavigationEngine::revalidateRoute() {
    std::lock_guard lock(nav_mutex_);
    return revalidateRouteLocked();
}

bool NavigationEngine::revalidateRouteLocked() {
    if (chain_.isEmpty()) return true;

    auto boundary = world_model_->getBoundary();
    if (!boundary.has_value()) return true;

    auto zones = world_model_->getActiveHSSZones();
    auto own = world_model_->getOwnState();
    auto current = chain_.currentTarget();
    if (own.isValid() && current.has_value() &&
        !safety_gate_->validateSegment(own.position, current->position)) {
        LOG_WARN("NavigationEngine: current route leg invalid after HSS/boundary change, replanning");
        auto remaining = chain_.remainingWaypoints();
        float speed = current->airspeed;
        if (!remaining.empty()) {
            auto goal = remaining.back().position;
            chain_.clear();
            last_plan_time_ = {};
            return planRouteLocked(goal, speed);
        }
        chain_.clear();
        last_sent_target_.reset();
        return false;
    }

    if (!chain_.validate(*planner_->getZoneChecker(), *boundary, zones,
                         planner_->getConfig().hss_buffer_m)) {
        LOG_WARN("NavigationEngine: route invalid after HSS change, replanning");
        auto remaining = chain_.remainingWaypoints();
        if (!remaining.empty()) {
            auto target = chain_.currentTarget();
            float speed = target.has_value() ? target->airspeed : -1.0f;
            auto goal = remaining.back().position;
            chain_.clear();
            last_plan_time_ = {};
            return planRouteLocked(goal, speed);
        }
        chain_.clear();
        last_sent_target_.reset();
        return false;
    }
    return true;
}

bool NavigationEngine::sendCurrentWaypointLocked() {
    auto wp = chain_.currentTarget();
    if (!wp.has_value()) return false;

    core::Position target = wp->position;
    if (!safety_gate_->validatePosition(target)) {
        LOG_WARN("NavigationEngine: waypoint {} unsafe, clamping", wp->label);
        target = safety_gate_->clampToSafe(target);
    }

    if (!validateProjectedCommandLocked(target, false)) {
        LOG_WARN("NavigationEngine: refusing unsafe waypoint segment");
        return false;
    }

    bool sent = commander_->flyToPosition(target, wp->airspeed);
    last_cmd_time_ = time::nowSteady();
    if (sent) last_sent_target_ = target;
    return sent;
}

bool NavigationEngine::shouldBlockNormalCommandLocked() {
    if (!escape_active_) return false;

    auto own = world_model_->getOwnState();
    if (own.isValid() && safety_gate_->isPositionSafeNow(own.position)) {
        LOG_INFO("NavigationEngine: escape complete, normal commands re-enabled");
        escape_active_ = false;
        escape_target_.reset();
        return false;
    }

    return true;
}

bool NavigationEngine::validateProjectedCommandLocked(const core::Position& target,
                                                      bool escape_command) {
    auto own = currentOwnPositionLocked();
    if (!own.has_value()) return false;
    return escape_command
        ? safety_gate_->validateEscapeSegment(*own, target)
        : safety_gate_->validateSegment(*own, target);
}

std::optional<core::Position> NavigationEngine::currentOwnPositionLocked() const {
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return std::nullopt;
    return own.position;
}

core::Position NavigationEngine::bodyOffsetToPosition(const core::UAVState& own,
                                                      float forward_m, float right_m,
                                                      float down_m) const {
    const double yaw = own.heading * geo::kDegToRad;
    const double north = std::cos(yaw) * forward_m - std::sin(yaw) * right_m;
    const double east = std::sin(yaw) * forward_m + std::cos(yaw) * right_m;
    return localOffsetToPosition(own, static_cast<float>(north),
                                 static_cast<float>(east), down_m);
}

core::Position NavigationEngine::localOffsetToPosition(const core::UAVState& own,
                                                       float north_m, float east_m,
                                                       float down_m) const {
    return geo::nedToPosition(
        geo::NED{static_cast<double>(north_m), static_cast<double>(east_m),
                 static_cast<double>(down_m)},
        own.position);
}

bool NavigationEngine::shouldThrottleCommand() const {
    auto now = time::nowSteady();
    return (now - last_cmd_time_) < min_cmd_interval_;
}

} // namespace uav::nav
