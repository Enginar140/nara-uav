#include "uav/mission/patrol_mode.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cmath>

namespace uav::mission {

PatrolMode::PatrolMode(std::shared_ptr<nav::NavigationEngine> nav,
                       std::shared_ptr<world_model::WorldModel> world_model,
                       Config config)
    : nav_(std::move(nav)), world_model_(std::move(world_model)), config_(config) {}

void PatrolMode::onEnter() {
    LOG_INFO("PatrolMode: entering");
    waypoints_generated_ = false;
    current_patrol_idx_ = 0;
    generatePatrolWaypoints();
    if (!patrol_waypoints_.empty()) {
        planToNextWaypoint();
    }
}

void PatrolMode::onExit() {
    LOG_INFO("PatrolMode: exiting");
    nav_->clearRoute();
}

void PatrolMode::tick(const core::UAVState& own_state) {
    if (patrol_waypoints_.empty()) {
        generatePatrolWaypoints();
        if (patrol_waypoints_.empty()) return;
    }

    auto status = nav_->update(own_state.position);

    if (status == nav::ChainStatus::CHAIN_COMPLETE ||
        status == nav::ChainStatus::EMPTY) {
        current_patrol_idx_ = (current_patrol_idx_ + 1) % patrol_waypoints_.size();
        planToNextWaypoint();
    }
}

core::MissionGoal PatrolMode::getMissionGoal() {
    return core::MissionGoal(core::GoalType::PATROL);
}

void PatrolMode::generatePatrolWaypoints() {
    auto boundary = world_model_->getBoundary();
    if (!boundary.has_value() || !boundary->isValid()) {
        LOG_WARN("PatrolMode: no valid boundary, cannot generate waypoints");
        return;
    }

    patrol_waypoints_.clear();
    const auto& verts = boundary->vertices;
    auto center = boundary->getCenter();
    size_t n = verts.size();

    for (size_t i = 0; i < n; ++i) {
        double bearing_to_center = geo::bearing(verts[i], center);
        double dist_to_center = geo::haversineDistance(verts[i], center);

        double inset = std::min(config_.boundary_inset_m, dist_to_center * 0.3);
        auto wp = geo::destinationPoint(verts[i], bearing_to_center, inset);
        wp.altitude = config_.cruise_altitude_m;
        patrol_waypoints_.push_back(wp);
    }

    // Sort by angle from center for proper traversal order
    std::sort(patrol_waypoints_.begin(), patrol_waypoints_.end(),
        [&center](const core::Position& a, const core::Position& b) {
            double angle_a = geo::bearing(center, a);
            double angle_b = geo::bearing(center, b);
            return angle_a < angle_b;
        });

    // Find nearest waypoint to start from
    auto own = world_model_->getOwnState();
    if (own.isValid() && !patrol_waypoints_.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;
        for (size_t i = 0; i < patrol_waypoints_.size(); ++i) {
            double d = geo::haversineDistance(own.position, patrol_waypoints_[i]);
            if (d < min_dist) {
                min_dist = d;
                nearest_idx = i;
            }
        }
        current_patrol_idx_ = nearest_idx;
    }

    waypoints_generated_ = !patrol_waypoints_.empty();
    LOG_INFO("PatrolMode: generated {} patrol waypoints", patrol_waypoints_.size());
}

void PatrolMode::planToNextWaypoint() {
    if (patrol_waypoints_.empty()) return;
    auto& target = patrol_waypoints_[current_patrol_idx_];
    nav_->planRoute(target, config_.patrol_speed_ms);
}

} // namespace uav::mission
