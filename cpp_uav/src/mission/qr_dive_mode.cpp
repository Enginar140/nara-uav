#include "uav/mission/qr_dive_mode.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>

namespace uav::mission {

const char* toString(DivePhase phase) {
    switch (phase) {
        case DivePhase::APPROACH:   return "APPROACH";
        case DivePhase::ALIGN:      return "ALIGN";
        case DivePhase::DIVE:       return "DIVE";
        case DivePhase::CLIMB:      return "CLIMB";
        case DivePhase::COMPLETED:  return "COMPLETED";
    }
    return "UNKNOWN";
}

QRDiveMode::QRDiveMode(std::shared_ptr<nav::NavigationEngine> nav,
                       std::shared_ptr<world_model::WorldModel> world_model,
                       std::shared_ptr<comm::FlightCommander> commander,
                       Config config)
    : nav_(std::move(nav)),
      world_model_(std::move(world_model)),
      commander_(std::move(commander)),
      config_(config) {}

void QRDiveMode::setDiveTarget(const core::Position& target) {
    dive_target_ = target;
    entry_valid_ = false;
}

void QRDiveMode::onEnter() {
    LOG_INFO("QRDiveMode: entering");
    phase_ = DivePhase::APPROACH;
    phase_start_ = time::nowSteady();
    last_qr_ = {};

    if (!dive_target_.has_value()) {
        LOG_ERROR("QRDiveMode: no dive target set");
        return;
    }

    calculateEntryPoint();
    if (entry_valid_) {
        nav_->planRoute(entry_point_);
    }
}

void QRDiveMode::onExit() {
    LOG_INFO("QRDiveMode: exiting, phase was {}", toString(phase_));
    nav_->clearRoute();
}

void QRDiveMode::tick(const core::UAVState& own_state) {
    if (!dive_target_.has_value()) return;

    // HSS abort check during dive
    if (phase_ == DivePhase::DIVE || phase_ == DivePhase::ALIGN) {
        auto zones = world_model_->getActiveHSSZones();
        for (const auto& zone : zones) {
            double dist = geo::haversineDistance(own_state.position, zone.center);
            if (dist < zone.radius + 20.0) {
                LOG_WARN("QRDiveMode: HSS zone {} too close ({:.0f}m), aborting dive",
                         zone.id, dist);
                phase_ = DivePhase::CLIMB;
                phase_start_ = time::nowSteady();
                return;
            }
        }
    }

    switch (phase_) {
        case DivePhase::APPROACH:  tickApproach(own_state); break;
        case DivePhase::ALIGN:     tickAlign(own_state); break;
        case DivePhase::DIVE:      tickDive(own_state); break;
        case DivePhase::CLIMB:     tickClimb(own_state); break;
        case DivePhase::COMPLETED: break;
    }
}

core::MissionGoal QRDiveMode::getMissionGoal() {
    core::MissionGoal goal(core::GoalType::DIVE_TARGET);
    goal.target_position = dive_target_;
    return goal;
}

void QRDiveMode::tickApproach(const core::UAVState& own) {
    auto status = nav_->update(own.position);

    if (status == nav::ChainStatus::CHAIN_COMPLETE || status == nav::ChainStatus::EMPTY) {
        double dist_to_entry = geo::haversineDistance(own.position, entry_point_);
        if (dist_to_entry < 150.0) {
            LOG_INFO("QRDiveMode: arrived at entry point, switching to ALIGN");
            phase_ = DivePhase::ALIGN;
            phase_start_ = time::nowSteady();
        } else {
            nav_->planRoute(entry_point_);
        }
    }
}

void QRDiveMode::tickAlign(const core::UAVState& own) {
    double heading_to_target = bearingToTarget(own);
    double heading_diff = std::abs(geo::angleDifference(own.heading, heading_to_target));

    double altitude_diff = std::abs(own.position.altitude - config_.approach_altitude_m);

    bool heading_ok = heading_diff < config_.align_heading_tolerance_deg;
    bool altitude_ok = altitude_diff < config_.align_altitude_tolerance_m;

    double elapsed = time::elapsedS(phase_start_);
    if (elapsed > 30.0) {
        LOG_WARN("QRDiveMode: align timeout, forcing dive");
        phase_ = DivePhase::DIVE;
        phase_start_ = time::nowSteady();
        return;
    }

    if (heading_ok && altitude_ok) {
        LOG_INFO("QRDiveMode: aligned (hdg_diff={:.1f}, alt_diff={:.1f}), starting DIVE",
                 heading_diff, altitude_diff);
        phase_ = DivePhase::DIVE;
        phase_start_ = time::nowSteady();
        return;
    }

    core::Position align_target = entry_point_;
    align_target.altitude = config_.approach_altitude_m;
    nav_->flyDirect(align_target);
}

void QRDiveMode::tickDive(const core::UAVState& own) {
    double dist = distanceToTarget(own);

    if (qr_callback_) {
        last_qr_ = qr_callback_();
    }

    double elapsed = time::elapsedS(phase_start_);
    if (dist < config_.target_pass_distance_m ||
        own.position.altitude <= config_.min_altitude_m ||
        elapsed > 30.0) {
        LOG_INFO("QRDiveMode: dive complete (dist={:.1f}m, alt={:.1f}m, qr={}, elapsed={:.1f}s)",
                 dist, own.position.altitude,
                 last_qr_.detected ? last_qr_.qr_data : "none", elapsed);
        phase_ = DivePhase::CLIMB;
        phase_start_ = time::nowSteady();
        return;
    }

    if (config_.use_position_dive) {
        tickDivePosition(own);
    } else {
        tickDiveVelocity(own);
    }
}

void QRDiveMode::tickDivePosition(const core::UAVState& own) {
    double pitch_rad = config_.dive_pitch_deg * geo::kDegToRad;
    double descent_rate = config_.dive_speed_ms * std::sin(std::abs(pitch_rad));
    double forward_speed = config_.dive_speed_ms * std::cos(std::abs(pitch_rad));

    double step_s = config_.dive_step_interval_s;
    double step_forward = forward_speed * step_s;
    double step_down = descent_rate * step_s;

    double bearing_to_target = geo::bearing(own.position, *dive_target_);

    auto next_pos = geo::destinationPoint(own.position, bearing_to_target, step_forward);
    next_pos.altitude = own.position.altitude - step_down;
    next_pos.altitude = std::max(next_pos.altitude, config_.min_altitude_m);

    nav_->flyDirect(next_pos, static_cast<float>(config_.dive_speed_ms));
}

void QRDiveMode::tickDiveVelocity(const core::UAVState& own) {
    auto target_ned = geo::positionToNED(*dive_target_, own.position);

    double pitch_rad = config_.dive_pitch_deg * geo::kDegToRad;
    double descent_rate = config_.dive_speed_ms * std::sin(std::abs(pitch_rad));
    double forward_speed = config_.dive_speed_ms * std::cos(std::abs(pitch_rad));

    double bearing_rad = std::atan2(target_ned.east, target_ned.north);
    float vn = static_cast<float>(forward_speed * std::cos(bearing_rad));
    float ve = static_cast<float>(forward_speed * std::sin(bearing_rad));
    float vd = static_cast<float>(descent_rate);

    nav_->flyLocalVelocity(vn, ve, vd);
}

void QRDiveMode::tickClimb(const core::UAVState& own) {
    double elapsed = time::elapsedS(phase_start_);

    if (own.position.altitude >= config_.climb_altitude_m ||
        elapsed > config_.climb_timeout_s) {
        LOG_INFO("QRDiveMode: climb complete (alt={:.1f}m, elapsed={:.1f}s)",
                 own.position.altitude, elapsed);
        phase_ = DivePhase::COMPLETED;
        if (completion_cb_) completion_cb_();
        return;
    }

    auto own_pos = own.position;
    own_pos.altitude = config_.climb_altitude_m;
    nav_->flyDirect(own_pos);
}

void QRDiveMode::calculateEntryPoint() {
    if (!dive_target_.has_value()) return;

    auto own = world_model_->getOwnState();
    double approach_bearing = geo::bearing(own.position, *dive_target_);

    double opposite_bearing = geo::normalizeAngle(approach_bearing + 180.0);
    entry_point_ = geo::destinationPoint(*dive_target_, opposite_bearing,
                                          config_.entry_corridor_m);
    entry_point_.altitude = config_.approach_altitude_m;
    entry_valid_ = true;

    LOG_INFO("QRDiveMode: entry point at bearing {:.1f}, {:.0f}m from target",
             opposite_bearing, config_.entry_corridor_m);
}

double QRDiveMode::distanceToTarget(const core::UAVState& own) const {
    if (!dive_target_.has_value()) return INFINITY;
    return geo::haversineDistance(own.position, *dive_target_);
}

double QRDiveMode::bearingToTarget(const core::UAVState& own) const {
    if (!dive_target_.has_value()) return 0.0;
    return geo::bearing(own.position, *dive_target_);
}

} // namespace uav::mission
