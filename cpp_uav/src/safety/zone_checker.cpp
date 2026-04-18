#include "uav/safety/zone_checker.hpp"
#include "uav/core/geo.hpp"

#include <algorithm>
#include <limits>

namespace uav::safety {

bool ZoneChecker::isInsideBoundary(const core::Position& pos,
                                   const core::Boundary& boundary) const {
    if (!boundary.isValid()) return true;
    return geo::isPointInPolygon(pos, boundary.vertices);
}

bool ZoneChecker::isInsideAnyHSS(const core::Position& pos,
                                 const std::vector<core::HSSZone>& zones,
                                 double extra_buffer) const {
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        double dist = geo::haversineDistance(pos, zone.center);
        if (dist < zone.radius + extra_buffer) {
            return true;
        }
    }
    return false;
}

double ZoneChecker::distanceToBoundary(const core::Position& pos,
                                       const core::Boundary& boundary) const {
    if (!boundary.isValid()) return std::numeric_limits<double>::max();

    double min_dist = std::numeric_limits<double>::max();
    size_t n = boundary.vertices.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double d = geo::distanceToSegment(pos, boundary.vertices[i], boundary.vertices[j]);
        min_dist = std::min(min_dist, d);
    }
    return min_dist;
}

double ZoneChecker::distanceToNearestHSS(const core::Position& pos,
                                         const std::vector<core::HSSZone>& zones) const {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        double dist = geo::haversineDistance(pos, zone.center) - zone.radius;
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

bool ZoneChecker::segmentIntersectsAnyHSS(const core::Position& from,
                                          const core::Position& to,
                                          const std::vector<core::HSSZone>& zones,
                                          double buffer) const {
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        if (geo::segmentIntersectsCircle(from, to, zone.center,
                                         zone.radius + buffer)) {
            return true;
        }
    }
    return false;
}

bool ZoneChecker::segmentExitsBoundary(const core::Position& from,
                                       const core::Position& to,
                                       const core::Boundary& boundary) const {
    if (!boundary.isValid()) return false;

    size_t n = boundary.vertices.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        if (geo::segmentsIntersect(from, to, boundary.vertices[i], boundary.vertices[j])) {
            return true;
        }
    }

    if (!isInsideBoundary(to, boundary)) return true;
    return false;
}

bool ZoneChecker::isPositionSafe(const core::Position& pos,
                                 const core::Boundary& boundary,
                                 const std::vector<core::HSSZone>& zones,
                                 double hss_buffer) const {
    if (!isInsideBoundary(pos, boundary)) return false;
    if (isInsideAnyHSS(pos, zones, hss_buffer)) return false;
    return true;
}

} // namespace uav::safety
