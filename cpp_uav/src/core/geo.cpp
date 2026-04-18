#include "uav/core/geo.hpp"

#include <algorithm>
#include <cmath>

namespace uav::geo {

double haversineDistance(const core::Position& a, const core::Position& b) {
    double dlat = (b.latitude - a.latitude) * kDegToRad;
    double dlon = (b.longitude - a.longitude) * kDegToRad;
    double lat1 = a.latitude * kDegToRad;
    double lat2 = b.latitude * kDegToRad;

    double s = std::sin(dlat / 2);
    double c = std::sin(dlon / 2);
    double h = s * s + std::cos(lat1) * std::cos(lat2) * c * c;
    double horizontal = 2.0 * kEarthRadius * std::asin(std::sqrt(h));

    double dalt = b.altitude - a.altitude;
    return std::sqrt(horizontal * horizontal + dalt * dalt);
}

double bearing(const core::Position& from, const core::Position& to) {
    double lat1 = from.latitude * kDegToRad;
    double lat2 = to.latitude * kDegToRad;
    double dlon = (to.longitude - from.longitude) * kDegToRad;

    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double brng = std::atan2(y, x) * kRadToDeg;
    return normalizeAngle(brng);
}

core::Position destinationPoint(const core::Position& from,
                                double bearing_deg, double distance_m) {
    double lat1 = from.latitude * kDegToRad;
    double lon1 = from.longitude * kDegToRad;
    double brng = bearing_deg * kDegToRad;
    double d = distance_m / kEarthRadius;

    double lat2 = std::asin(std::sin(lat1) * std::cos(d) +
                            std::cos(lat1) * std::sin(d) * std::cos(brng));
    double lon2 = lon1 + std::atan2(
        std::sin(brng) * std::sin(d) * std::cos(lat1),
        std::cos(d) - std::sin(lat1) * std::sin(lat2));

    return {lat2 * kRadToDeg, lon2 * kRadToDeg, from.altitude};
}

bool isPointInPolygon(const core::Position& point,
                      const std::vector<core::Position>& polygon) {
    if (polygon.size() < 3) return false;

    bool inside = false;
    size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double yi = polygon[i].latitude, yj = polygon[j].latitude;
        double xi = polygon[i].longitude, xj = polygon[j].longitude;

        if (((yi > point.latitude) != (yj > point.latitude)) &&
            (point.longitude < (xj - xi) * (point.latitude - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

double distanceToSegment(const core::Position& point,
                         const core::Position& seg_a,
                         const core::Position& seg_b) {
    auto nearest = nearestPointOnSegment(point, seg_a, seg_b);
    return haversineDistance(point, nearest);
}

core::Position nearestPointOnSegment(const core::Position& p,
                                     const core::Position& a,
                                     const core::Position& b) {
    NED ap = positionToNED(p, a);
    NED ab = positionToNED(b, a);

    double ab_sq = ab.north * ab.north + ab.east * ab.east;
    if (ab_sq < 1e-12) return a;

    double t = (ap.north * ab.north + ap.east * ab.east) / ab_sq;
    t = std::clamp(t, 0.0, 1.0);

    NED result{ab.north * t, ab.east * t, 0.0};
    return nedToPosition(result, a);
}

bool segmentIntersectsCircle(const core::Position& p1, const core::Position& p2,
                             const core::Position& center, double radius) {
    double dist = distanceToSegment(center, p1, p2);
    return dist <= radius;
}

bool segmentsIntersect(const core::Position& a1, const core::Position& a2,
                       const core::Position& b1, const core::Position& b2) {
    NED ref_origin = {0, 0, 0};
    auto toXY = [&](const core::Position& p) -> std::pair<double, double> {
        NED ned = positionToNED(p, a1);
        return {ned.east, ned.north};
    };

    auto [ax1, ay1] = toXY(a1);
    auto [ax2, ay2] = toXY(a2);
    auto [bx1, by1] = toXY(b1);
    auto [bx2, by2] = toXY(b2);

    double d1x = ax2 - ax1, d1y = ay2 - ay1;
    double d2x = bx2 - bx1, d2y = by2 - by1;

    double denom = d1x * d2y - d1y * d2x;
    if (std::abs(denom) < 1e-12) return false;

    double t = ((bx1 - ax1) * d2y - (by1 - ay1) * d2x) / denom;
    double u = ((bx1 - ax1) * d1y - (by1 - ay1) * d1x) / denom;

    return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

NED positionToNED(const core::Position& pos, const core::Position& reference) {
    double dlat = (pos.latitude - reference.latitude) * kDegToRad;
    double dlon = (pos.longitude - reference.longitude) * kDegToRad;
    double cos_lat = std::cos(reference.latitude * kDegToRad);

    return {
        dlat * kEarthRadius,
        dlon * kEarthRadius * cos_lat,
        -(pos.altitude - reference.altitude)
    };
}

core::Position nedToPosition(const NED& ned, const core::Position& reference) {
    double cos_lat = std::cos(reference.latitude * kDegToRad);
    double dlat = ned.north / kEarthRadius;
    double dlon = (cos_lat > 1e-12) ? ned.east / (kEarthRadius * cos_lat) : 0.0;

    return {
        reference.latitude + dlat * kRadToDeg,
        reference.longitude + dlon * kRadToDeg,
        reference.altitude - ned.down
    };
}

double normalizeAngle(double degrees) {
    degrees = std::fmod(degrees, 360.0);
    if (degrees < 0.0) degrees += 360.0;
    return degrees;
}

double angleDifference(double a, double b) {
    double diff = std::fmod(a - b + 540.0, 360.0) - 180.0;
    return diff;
}

} // namespace uav::geo
