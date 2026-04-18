#include "uav/nav/navigation_engine.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

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
    auto now = time::nowSteady();
    if (now - last_plan_time_ < min_plan_interval_ && hasActiveRoute()) {
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
    auto zones = world_model_->getActiveHSSZones();

    auto result = planner_->plan(own.position, goal, *boundary, zones);
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
        wp.position = goal;
        wp.airspeed = speed;
        wp.label = "GOAL";
        wp_chain.push_back(wp);
    }

    chain_.replace(std::move(wp_chain));
    last_plan_time_ = now;

    LOG_INFO("NavigationEngine: route planned, {} waypoints, {:.0f}m",
             chain_.size(), result.total_distance_m);

    sendCurrentWaypoint();
    return true;
}

ChainStatus NavigationEngine::update(const core::Position& own_pos) {
    if (chain_.isEmpty()) return ChainStatus::EMPTY;

    auto status = chain_.update(own_pos);

    if (status == ChainStatus::WAYPOINT_REACHED) {
        if (!shouldThrottleCommand()) {
            sendCurrentWaypoint();
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
                    last_sent_target_ = target;
                    last_cmd_time_ = time::nowSteady();
                    (void)commander_->flyToPosition(target, next->airspeed);
                    LOG_DEBUG("NavigationEngine: look-ahead sent next WP at {:.0f}m from current", dist);
                }
            } else if (!shouldThrottleCommand()) {
                sendCurrentWaypoint();
            }
        }
    }

    return status;
}

bool NavigationEngine::flyDirect(const core::Position& target, float airspeed) {
    if (shouldThrottleCommand()) return true;

    float speed = (airspeed > 0) ? airspeed : speed_manager_->cruiseSpeed();
    core::Position safe_target = target;

    if (!safety_gate_->validatePosition(target)) {
        safe_target = safety_gate_->clampToSafe(target);
        LOG_DEBUG("NavigationEngine: flyDirect clamped to safe position");
    }

    last_sent_target_ = safe_target;
    last_cmd_time_ = time::nowSteady();
    return commander_->flyToPosition(safe_target, speed);
}

bool NavigationEngine::flyLocalPosition(float north, float east, float down) {
    if (shouldThrottleCommand()) return true;

    last_cmd_time_ = time::nowSteady();
    return commander_->flyToLocalPosition(north, east, down);
}

bool NavigationEngine::flyLocalVelocity(float vn, float ve, float vd, float yaw_rate) {
    if (!safety_gate_->validateVelocity(vn, ve, vd)) {
        safety_gate_->clampVelocity(vn, ve, vd);
        LOG_DEBUG("NavigationEngine: velocity clamped by safety gate");
    }

    last_cmd_time_ = time::nowSteady();
    return commander_->setLocalVelocity(vn, ve, vd, yaw_rate);
}

void NavigationEngine::clearRoute() {
    chain_.clear();
    last_sent_target_.reset();
}

bool NavigationEngine::hasActiveRoute() const {
    return !chain_.isEmpty() && !chain_.isComplete();
}

bool NavigationEngine::revalidateRoute() {
    if (chain_.isEmpty()) return true;

    auto boundary = world_model_->getBoundary();
    if (!boundary.has_value()) return true;

    auto zones = world_model_->getActiveHSSZones();
    if (!chain_.validate(*planner_->getZoneChecker(), *boundary, zones,
                         planner_->getConfig().hss_buffer_m)) {
        LOG_WARN("NavigationEngine: route invalid after HSS change, replanning");
        auto remaining = chain_.remainingWaypoints();
        if (!remaining.empty()) {
            auto target = chain_.currentTarget();
            float speed = target.has_value() ? target->airspeed : -1.0f;
            return planRoute(remaining.back().position, speed);
        }
        clearRoute();
        return false;
    }
    return true;
}

bool NavigationEngine::sendCurrentWaypoint() {
    auto wp = chain_.currentTarget();
    if (!wp.has_value()) return false;

    core::Position target = wp->position;
    if (!safety_gate_->validatePosition(target)) {
        LOG_WARN("NavigationEngine: waypoint {} unsafe, clamping", wp->label);
        target = safety_gate_->clampToSafe(target);
    }

    last_sent_target_ = target;
    last_cmd_time_ = time::nowSteady();
    return commander_->flyToPosition(target, wp->airspeed);
}

bool NavigationEngine::shouldThrottleCommand() const {
    auto now = time::nowSteady();
    return (now - last_cmd_time_) < min_cmd_interval_;
}

} // namespace uav::nav
