#include "uav/safety/safety_gate.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cmath>

namespace uav::safety {

SafetyGate::SafetyGate(std::shared_ptr<ZoneChecker> checker,
                       std::shared_ptr<world_model::WorldModel> world_model,
                       Config config)
    : checker_(std::move(checker)),
      world_model_(std::move(world_model)),
      config_(config) {}

bool SafetyGate::validatePosition(const core::Position& target) const {
    if (target.altitude < config_.min_altitude_m ||
        target.altitude > config_.max_altitude_m) {
        return false;
    }

    auto boundary = world_model_->getBoundary();
    auto zones = world_model_->getActiveHSSZones();

    if (boundary.has_value()) {
        if (!checker_->isInsideBoundary(target, *boundary)) return false;

        double dist_to_bnd = checker_->distanceToBoundary(target, *boundary);
        if (dist_to_bnd < config_.boundary_buffer_m) return false;
    }

    if (checker_->isInsideAnyHSS(target, zones, config_.hss_buffer_m)) return false;

    return true;
}

bool SafetyGate::isInsideHSS(const core::Position& pos, bool include_buffer) const {
    auto zones = world_model_->getActiveHSSZones();
    return checker_->isInsideAnyHSS(pos, zones, include_buffer ? config_.hss_buffer_m : 0.0);
}

bool SafetyGate::isPositionSafeNow(const core::Position& pos) const {
    return validatePosition(pos);
}

bool SafetyGate::validateSegment(const core::Position& from, const core::Position& target) const {
    if (!validatePosition(target)) return false;

    auto boundary = world_model_->getBoundary();
    if (boundary.has_value()) {
        if (!checker_->isInsideBoundary(from, *boundary)) return false;
        if (checker_->segmentExitsBoundary(from, target, *boundary)) return false;
    }

    auto zones = world_model_->getActiveHSSZones();
    if (checker_->isInsideAnyHSS(from, zones, config_.hss_buffer_m)) return false;
    if (checker_->segmentIntersectsAnyHSS(from, target, zones, config_.hss_buffer_m)) return false;

    return true;
}

bool SafetyGate::validateEscapeSegment(const core::Position& from, const core::Position& target) const {
    if (!validatePosition(target)) return false;

    auto boundary = world_model_->getBoundary();
    if (boundary.has_value()) {
        if (!checker_->isInsideBoundary(target, *boundary)) return false;
    }

    auto zones = world_model_->getActiveHSSZones();
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        const double safe_dist = zone.radius + config_.hss_buffer_m;
        const double from_dist = geo::haversineDistance(from, zone.center);
        const double target_dist = geo::haversineDistance(target, zone.center);
        const bool starts_in_zone = from_dist < safe_dist;

        if (starts_in_zone) {
            if (target_dist < safe_dist || target_dist <= from_dist) {
                return false;
            }
            continue;
        }

        if (geo::segmentIntersectsCircle(from, target, zone.center, safe_dist)) {
            return false;
        }
    }

    return true;
}

bool SafetyGate::validateVelocity(float vn, float ve, float vd) const {
    double speed = std::sqrt(vn * vn + ve * ve + vd * vd);
    if (speed > config_.max_velocity_ms) return false;

    auto own = world_model_->getOwnState();
    if (!own.isValid()) return true;

    double predict_s = 3.0;
    double pred_north = own.position.latitude + (vn * predict_s) / 111320.0;
    double pred_east = own.position.longitude +
        (ve * predict_s) / (111320.0 * std::cos(own.position.latitude * geo::kDegToRad));
    double pred_alt = own.position.altitude - vd * predict_s;

    core::Position predicted(pred_north, pred_east, pred_alt);

    auto boundary = world_model_->getBoundary();
    if (boundary.has_value() && !checker_->isInsideBoundary(predicted, *boundary)) {
        return false;
    }

    auto zones = world_model_->getActiveHSSZones();
    if (checker_->isInsideAnyHSS(predicted, zones, config_.hss_buffer_m)) {
        return false;
    }

    return true;
}

core::Position SafetyGate::clampToSafe(const core::Position& target) const {
    core::Position safe = target;

    safe.altitude = std::clamp(safe.altitude, config_.min_altitude_m, config_.max_altitude_m);

    auto boundary = world_model_->getBoundary();
    if (boundary.has_value()) {
        if (!checker_->isInsideBoundary(safe, *boundary)) {
            auto center = boundary->getCenter();
            double brng = geo::bearing(safe, center);
            double dist = geo::haversineDistance(safe, center);
            double step = dist * 0.1;
            for (int i = 0; i < 15; ++i) {
                safe = geo::destinationPoint(safe, brng, step);
                if (checker_->isInsideBoundary(safe, *boundary)) {
                    double d = checker_->distanceToBoundary(safe, *boundary);
                    if (d >= config_.boundary_buffer_m) break;
                    safe = geo::destinationPoint(safe, brng, config_.boundary_buffer_m - d);
                    break;
                }
            }
        } else {
            double d = checker_->distanceToBoundary(safe, *boundary);
            if (d < config_.boundary_buffer_m) {
                auto center = boundary->getCenter();
                double brng = geo::bearing(safe, center);
                safe = geo::destinationPoint(safe, brng, config_.boundary_buffer_m - d);
            }
        }
    }

    auto zones = world_model_->getActiveHSSZones();
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        double dist = geo::haversineDistance(safe, zone.center);
        double safe_dist = zone.radius + config_.hss_buffer_m;
        if (dist < safe_dist) {
            double brng = geo::bearing(zone.center, safe);
            safe = geo::destinationPoint(zone.center, brng, safe_dist + 10.0);
            safe.altitude = target.altitude;
        }
    }

    safe.altitude = std::clamp(safe.altitude, config_.min_altitude_m, config_.max_altitude_m);
    return safe;
}

void SafetyGate::clampVelocity(float& vn, float& ve, float& vd) const {
    double speed = std::sqrt(vn * vn + ve * ve + vd * vd);
    if (speed > config_.max_velocity_ms && speed > 0.01) {
        float scale = static_cast<float>(config_.max_velocity_ms / speed);
        vn *= scale;
        ve *= scale;
        vd *= scale;
    }
}

} // namespace uav::safety
