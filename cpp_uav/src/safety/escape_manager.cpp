#include "uav/safety/escape_manager.hpp"
#include "uav/safety/safety_gate.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>

namespace uav::safety {

EscapeManager::EscapeManager(std::shared_ptr<ZoneChecker> checker,
                             std::shared_ptr<world_model::WorldModel> world_model,
                             std::shared_ptr<SafetyGate> safety_gate)
    : checker_(std::move(checker)),
      world_model_(std::move(world_model)),
      safety_gate_(std::move(safety_gate)) {}

std::optional<core::Position> EscapeManager::validateOrFindAlternative(
    const core::Position& candidate,
    const core::Position& own_pos,
    double base_bearing, double escape_dist) const {

    if (!safety_gate_) return candidate;

    if (safety_gate_->validatePosition(candidate)) return candidate;

    static const double kOffsets[] = {30, -30, 60, -60, 90, -90, 120, -120, 150, -150};
    for (double offset : kOffsets) {
        double alt_bearing = geo::normalizeAngle(base_bearing + offset);
        auto alt = geo::destinationPoint(own_pos, alt_bearing, escape_dist);
        alt.altitude = candidate.altitude;
        if (safety_gate_->validatePosition(alt)) {
            LOG_DEBUG("EscapeManager: original bearing {:.0f} unsafe, using {:.0f}",
                      base_bearing, alt_bearing);
            return alt;
        }
    }

    auto boundary = world_model_->getBoundary();
    if (boundary.has_value()) {
        auto center = boundary->getCenter();
        center.altitude = own_pos.altitude;
        if (safety_gate_->validatePosition(center)) return center;
    }

    LOG_WARN("EscapeManager: no safe escape found, returning best candidate");
    return safety_gate_->clampToSafe(candidate);
}

std::optional<core::Position> EscapeManager::generateEscapeTarget(
    const core::UAVState& own) const {

    auto boundary = world_model_->getBoundary();
    auto zones = world_model_->getActiveHSSZones();

    if (boundary.has_value() && !checker_->isInsideBoundary(own.position, *boundary)) {
        return generateBoundaryEscape(own, *boundary);
    }

    for (const auto& zone : zones) {
        double dist = geo::haversineDistance(own.position, zone.center);
        if (dist < zone.radius) {
            return generateHSSEscape(own, zones);
        }
    }

    return std::nullopt;
}

std::optional<core::Position> EscapeManager::generateBoundaryEscape(
    const core::UAVState& own, const core::Boundary& bnd) const {

    auto center = bnd.getCenter();
    double bearing_to_center = geo::bearing(own.position, center);

    auto escape = geo::destinationPoint(own.position, bearing_to_center, 200.0);
    escape.altitude = own.position.altitude;

    auto validated = validateOrFindAlternative(escape, own.position, bearing_to_center, 200.0);
    if (validated.has_value()) {
        LOG_INFO("EscapeManager: boundary escape bearing={:.1f}", bearing_to_center);
    }
    return validated;
}

std::optional<core::Position> EscapeManager::generateHSSEscape(
    const core::UAVState& own, const std::vector<core::HSSZone>& zones) const {

    double nearest_dist = std::numeric_limits<double>::max();
    const core::HSSZone* nearest_zone = nullptr;

    for (const auto& zone : zones) {
        double dist = geo::haversineDistance(own.position, zone.center);
        if (dist < zone.radius && dist < nearest_dist) {
            nearest_dist = dist;
            nearest_zone = &zone;
        }
    }

    if (!nearest_zone) return std::nullopt;

    double bearing_away = geo::bearing(nearest_zone->center, own.position);
    double escape_dist = nearest_zone->radius - nearest_dist + 80.0;

    auto escape = geo::destinationPoint(own.position, bearing_away, escape_dist);
    escape.altitude = own.position.altitude;

    auto validated = validateOrFindAlternative(escape, own.position, bearing_away, escape_dist);
    if (validated.has_value()) {
        LOG_INFO("EscapeManager: HSS escape from zone {}, bearing={:.1f}",
                 nearest_zone->id, bearing_away);
    }
    return validated;
}

std::optional<core::Position> EscapeManager::generateCollisionEscape(
    const core::UAVState& own, const core::Threat& threat) const {

    double escape_bearing = geo::normalizeAngle(threat.bearing + 90.0);

    if (own.velocity.horizontalSpeed() > 5.0) {
        double own_heading = std::atan2(own.velocity.east, own.velocity.north) * geo::kRadToDeg;
        double angle_diff = geo::angleDifference(threat.bearing, own_heading);
        if (angle_diff > 0) {
            escape_bearing = geo::normalizeAngle(own_heading + 45.0);
        } else {
            escape_bearing = geo::normalizeAngle(own_heading - 45.0);
        }
    }

    auto escape = geo::destinationPoint(own.position, escape_bearing, 150.0);
    escape.altitude = own.position.altitude + 20.0;

    auto validated = validateOrFindAlternative(escape, own.position, escape_bearing, 150.0);
    if (validated.has_value()) {
        LOG_INFO("EscapeManager: collision escape from aircraft {}, bearing={:.1f}",
                 threat.aircraft_id, escape_bearing);
    }
    return validated;
}

} // namespace uav::safety
