#pragma once

#include "uav/core/types.hpp"
#include <vector>

namespace uav::safety {

class ZoneChecker {
public:
    ZoneChecker() = default;

    bool isInsideBoundary(const core::Position& pos,
                          const core::Boundary& boundary) const;

    bool isInsideAnyHSS(const core::Position& pos,
                        const std::vector<core::HSSZone>& zones,
                        double extra_buffer = 0.0) const;

    double distanceToBoundary(const core::Position& pos,
                              const core::Boundary& boundary) const;

    double distanceToNearestHSS(const core::Position& pos,
                                const std::vector<core::HSSZone>& zones) const;

    bool segmentIntersectsAnyHSS(const core::Position& from,
                                 const core::Position& to,
                                 const std::vector<core::HSSZone>& zones,
                                 double buffer = 0.0) const;

    bool segmentExitsBoundary(const core::Position& from,
                              const core::Position& to,
                              const core::Boundary& boundary) const;

    bool isPositionSafe(const core::Position& pos,
                        const core::Boundary& boundary,
                        const std::vector<core::HSSZone>& zones,
                        double hss_buffer = 40.0) const;
};

} // namespace uav::safety
