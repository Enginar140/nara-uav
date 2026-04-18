#pragma once

#include "uav/core/types.hpp"
#include <vector>

namespace uav::geo {

struct NED {
    double north;
    double east;
    double down;
};

constexpr double kEarthRadius = 6371000.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double haversineDistance(const core::Position& a, const core::Position& b);

double bearing(const core::Position& from, const core::Position& to);

core::Position destinationPoint(const core::Position& from, double bearing_deg, double distance_m);

bool isPointInPolygon(const core::Position& point, const std::vector<core::Position>& polygon);

double distanceToSegment(const core::Position& point,
                         const core::Position& seg_a, const core::Position& seg_b);

core::Position nearestPointOnSegment(const core::Position& p,
                                     const core::Position& a, const core::Position& b);

bool segmentIntersectsCircle(const core::Position& p1, const core::Position& p2,
                             const core::Position& center, double radius);

bool segmentsIntersect(const core::Position& a1, const core::Position& a2,
                       const core::Position& b1, const core::Position& b2);

NED positionToNED(const core::Position& pos, const core::Position& reference);

core::Position nedToPosition(const NED& ned, const core::Position& reference);

double normalizeAngle(double degrees);

double angleDifference(double a, double b);

} // namespace uav::geo
