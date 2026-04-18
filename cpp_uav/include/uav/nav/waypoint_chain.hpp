#pragma once

#include "uav/core/types.hpp"
#include "uav/safety/zone_checker.hpp"

#include <optional>
#include <string>
#include <vector>

namespace uav::nav {

struct Waypoint {
    core::Position position;
    float airspeed = -1.0f;
    double acceptance_radius_m = 80.0;
    std::string label;
};

enum class ChainStatus { EMPTY, IN_PROGRESS, WAYPOINT_REACHED, CHAIN_COMPLETE };

class WaypointChain {
public:
    WaypointChain() = default;

    void load(std::vector<Waypoint> chain, bool circular = false);
    ChainStatus update(const core::Position& own_pos);
    std::optional<Waypoint> currentTarget() const;
    std::optional<Waypoint> peekNext() const;
    std::vector<Waypoint> remainingWaypoints() const;

    bool isEmpty() const { return chain_.empty(); }
    bool isComplete() const { return !chain_.empty() && complete_; }
    size_t currentIndex() const { return idx_; }
    size_t size() const { return chain_.size(); }
    void clear();

    void replace(std::vector<Waypoint> new_chain);

    bool validate(const safety::ZoneChecker& checker,
                  const core::Boundary& boundary,
                  const std::vector<core::HSSZone>& zones,
                  double hss_buffer = 40.0) const;

private:
    std::vector<Waypoint> chain_;
    size_t idx_ = 0;
    bool circular_ = false;
    bool complete_ = false;
};

} // namespace uav::nav
