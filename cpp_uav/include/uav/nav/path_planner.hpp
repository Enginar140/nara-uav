#pragma once

#include "uav/core/types.hpp"
#include "uav/safety/zone_checker.hpp"

#include <memory>
#include <string>
#include <vector>

namespace uav::nav {

struct PathResult {
    bool success = false;
    std::vector<core::Position> waypoints;
    double total_distance_m = 0.0;
    double planning_time_ms = 0.0;
    std::string error;
};

class PathPlanner {
public:
    struct Config {
        double hss_buffer_m = 40.0;
        double boundary_buffer_m = 20.0;
        double min_turn_radius_m = 50.0;
        int polygon_segments = 12;
    };

    PathPlanner(std::shared_ptr<safety::ZoneChecker> checker, Config config = {});

    PathResult plan(const core::Position& start, const core::Position& goal,
                    const core::Boundary& boundary,
                    const std::vector<core::HSSZone>& hss_zones);

    bool isDirectPathClear(const core::Position& from, const core::Position& to,
                           const core::Boundary& boundary,
                           const std::vector<core::HSSZone>& zones) const;

    void setMinTurnRadius(double radius_m) { config_.min_turn_radius_m = radius_m; }
    void setConfig(const Config& c) { config_ = c; }
    const Config& getConfig() const { return config_; }
    const std::shared_ptr<safety::ZoneChecker>& getZoneChecker() const { return zone_checker_; }

private:
    std::shared_ptr<safety::ZoneChecker> zone_checker_;
    Config config_;

    struct VNode {
        core::Position pos;
        int obstacle_id;  // -1 for start/goal
    };

    using AdjList = std::vector<std::vector<std::pair<int, double>>>;

    std::vector<std::vector<core::Position>> expandObstacles(
        const std::vector<core::HSSZone>& zones) const;

    std::vector<core::Position> circleToPolygon(
        const core::Position& center, double radius) const;

    std::vector<VNode> buildNodes(
        const core::Position& start, const core::Position& goal,
        const std::vector<std::vector<core::Position>>& obstacles) const;

    AdjList buildAdjacency(
        const std::vector<VNode>& nodes,
        const std::vector<std::vector<core::Position>>& obstacles,
        const core::Boundary& boundary) const;

    std::vector<int> dijkstra(const AdjList& adj, int src, int dst) const;

    std::vector<core::Position> simplifyPath(
        const std::vector<core::Position>& path,
        const std::vector<std::vector<core::Position>>& obstacles,
        const core::Boundary& boundary) const;

    std::vector<core::Position> smoothTurns(
        const std::vector<core::Position>& path,
        const core::Boundary& boundary,
        const std::vector<core::HSSZone>& hss_zones) const;

    bool isEdgeVisible(
        const core::Position& a, const core::Position& b,
        const std::vector<std::vector<core::Position>>& obstacles,
        const core::Boundary& boundary,
        int skip_obs_a = -1, int skip_obs_b = -1) const;

    int findContainingObstacle(
        const core::Position& pos,
        const std::vector<std::vector<core::Position>>& obstacles) const;
};

} // namespace uav::nav
