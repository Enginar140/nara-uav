#include "uav/nav/path_planner.hpp"
#include "uav/core/geo.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace uav::nav {

PathPlanner::PathPlanner(std::shared_ptr<safety::ZoneChecker> checker, Config config)
    : zone_checker_(std::move(checker)), config_(config) {
    sanitizeConfig();
}

void PathPlanner::sanitizeConfig() {
    auto finiteOr = [](double value, double fallback) {
        return std::isfinite(value) ? value : fallback;
    };

    config_.hss_buffer_m = std::max(0.0, finiteOr(config_.hss_buffer_m, 40.0));
    config_.boundary_buffer_m = std::max(0.0, finiteOr(config_.boundary_buffer_m, 20.0));
    config_.min_turn_radius_m = std::max(0.0, finiteOr(config_.min_turn_radius_m, 50.0));
    config_.turn_safety_margin_m = std::max(0.0, finiteOr(config_.turn_safety_margin_m, 50.0));
    config_.max_arc_step_deg = std::clamp(finiteOr(config_.max_arc_step_deg, 10.0), 2.0, 45.0);
    config_.max_segment_length_m = std::max(10.0, finiteOr(config_.max_segment_length_m, 80.0));
    config_.polygon_segments = std::max(8, config_.polygon_segments);
}

double PathPlanner::planningHSSMargin() const {
    return config_.hss_buffer_m + std::max(0.0, config_.turn_safety_margin_m);
}

PathResult PathPlanner::plan(const core::Position& start, const core::Position& goal,
                             const core::Boundary& boundary,
                             const std::vector<core::HSSZone>& hss_zones) {
    auto t0 = time::nowSteady();
    PathResult result;

    if (isDirectPathClear(start, goal, boundary, hss_zones)) {
        result.success = true;
        result.waypoints = {start, goal};
        result.total_distance_m = geo::haversineDistance(start, goal);
        result.planning_time_ms = time::elapsedMs(t0);
        return result;
    }

    auto obstacles = expandObstacles(hss_zones);
    auto nodes = buildNodes(start, goal, obstacles);

    if (nodes.size() < 2) {
        result.error = "Not enough nodes for planning";
        result.planning_time_ms = time::elapsedMs(t0);
        return result;
    }

    auto adj = buildAdjacency(nodes, obstacles, boundary, hss_zones);

    size_t total_edges = 0;
    size_t connected_nodes = 0;
    for (size_t i = 0; i < adj.size(); ++i) {
        total_edges += adj[i].size();
        if (!adj[i].empty()) connected_nodes++;
    }

    int start_idx = 0;
    int goal_idx = 1;
    auto idx_path = dijkstra(adj, start_idx, goal_idx);

    if (idx_path.empty()) {
        LOG_DEBUG("PathPlanner: {} nodes, {} edges, {} connected. start_edges={}, goal_edges={}",
                  nodes.size(), total_edges / 2, connected_nodes,
                  adj[0].size(), adj[1].size());
        result.error = "No path found through visibility graph";
        result.planning_time_ms = time::elapsedMs(t0);
        return result;
    }

    std::vector<core::Position> path;
    path.reserve(idx_path.size());
    for (int i : idx_path) {
        path.push_back(nodes[i].pos);
    }

    path = simplifyPath(path, obstacles, boundary);
    if (config_.dubins_smoothing_enabled) {
        path = smoothTurns(path, boundary, hss_zones);
    }
    path = densifyPath(path);

    if (!isPathSafe(path, boundary, hss_zones, config_.hss_buffer_m)) {
        result.error = "Planner produced unsafe path after smoothing";
        result.planning_time_ms = time::elapsedMs(t0);
        return result;
    }

    for (auto& wp : path) {
        if (std::abs(wp.altitude) < 1e-3) {
            wp.altitude = goal.altitude;
        }
    }

    double total_dist = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        total_dist += geo::haversineDistance(path[i - 1], path[i]);
    }

    result.success = true;
    result.waypoints = std::move(path);
    result.total_distance_m = total_dist;
    result.planning_time_ms = time::elapsedMs(t0);
    return result;
}

bool PathPlanner::isDirectPathClear(const core::Position& from, const core::Position& to,
                                    const core::Boundary& boundary,
                                    const std::vector<core::HSSZone>& zones) const {
    return isSegmentSafe(from, to, boundary, zones, planningHSSMargin());
}

// ---------------------------------------------------------------------------
// Obstacle expansion: HSS circles -> convex polygons with buffer
// ---------------------------------------------------------------------------

std::vector<std::vector<core::Position>> PathPlanner::expandObstacles(
    const std::vector<core::HSSZone>& zones) const {

    std::vector<std::vector<core::Position>> obstacles;
    for (const auto& zone : zones) {
        if (!zone.active) continue;
        constexpr double kPi = 3.14159265358979323846;
        double chord_compensation = 1.0 / std::cos(kPi / config_.polygon_segments);
        double r = (zone.radius + planningHSSMargin()) * chord_compensation;
        obstacles.push_back(circleToPolygon(zone.center, r));
    }
    return obstacles;
}

std::vector<core::Position> PathPlanner::circleToPolygon(
    const core::Position& center, double radius) const {

    std::vector<core::Position> poly;
    poly.reserve(config_.polygon_segments);
    double step = 360.0 / config_.polygon_segments;
    for (int i = 0; i < config_.polygon_segments; ++i) {
        double bearing = i * step;
        poly.push_back(geo::destinationPoint(center, bearing, radius));
    }
    return poly;
}

// ---------------------------------------------------------------------------
// Visibility graph: nodes = start + goal + all polygon vertices
// ---------------------------------------------------------------------------

std::vector<PathPlanner::VNode> PathPlanner::buildNodes(
    const core::Position& start, const core::Position& goal,
    const std::vector<std::vector<core::Position>>& obstacles) const {

    std::vector<VNode> nodes;
    nodes.push_back({start, -1, -1});
    nodes.push_back({goal, -1, -1});

    for (int obs_idx = 0; obs_idx < static_cast<int>(obstacles.size()); ++obs_idx) {
        for (int vertex_idx = 0; vertex_idx < static_cast<int>(obstacles[obs_idx].size()); ++vertex_idx) {
            nodes.push_back({obstacles[obs_idx][vertex_idx], obs_idx, vertex_idx});
        }
    }
    return nodes;
}

PathPlanner::AdjList PathPlanner::buildAdjacency(
    const std::vector<VNode>& nodes,
    const std::vector<std::vector<core::Position>>& obstacles,
    const core::Boundary& boundary,
    const std::vector<core::HSSZone>& hss_zones) const {

    int n = static_cast<int>(nodes.size());
    AdjList adj(n);

    int start_containing = findContainingObstacle(nodes[0].pos, obstacles);
    int goal_containing = findContainingObstacle(nodes[1].pos, obstacles);

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (nodes[i].obstacle_id >= 0 &&
                nodes[i].obstacle_id == nodes[j].obstacle_id) {
                const int obs = nodes[i].obstacle_id;
                const int vertex_count = static_cast<int>(obstacles[obs].size());
                const int diff = std::abs(nodes[i].vertex_id - nodes[j].vertex_id);
                const bool adjacent = diff == 1 || diff == vertex_count - 1;
                if (!adjacent) {
                    continue;
                }
            }

            int skip_a = -1;
            int skip_b = -1;
            if (i == 0 && start_containing >= 0) skip_a = start_containing;
            if (i == 1 && goal_containing >= 0) skip_a = goal_containing;
            if (j == 0 && start_containing >= 0) skip_b = start_containing;
            if (j == 1 && goal_containing >= 0) skip_b = goal_containing;

            if (!isSegmentSafe(nodes[i].pos, nodes[j].pos, boundary, hss_zones,
                               config_.hss_buffer_m)) {
                continue;
            }

            if (isEdgeVisible(nodes[i].pos, nodes[j].pos, obstacles, boundary,
                              skip_a, skip_b)) {
                double dist = geo::haversineDistance(nodes[i].pos, nodes[j].pos);
                adj[i].emplace_back(j, dist);
                adj[j].emplace_back(i, dist);
            }
        }
    }
    return adj;
}

bool PathPlanner::isEdgeVisible(
    const core::Position& a, const core::Position& b,
    const std::vector<std::vector<core::Position>>& obstacles,
    const core::Boundary& boundary,
    int skip_obs_a, int skip_obs_b) const {

    auto posMatch = [](const core::Position& p1, const core::Position& p2) {
        return std::abs(p1.latitude - p2.latitude) < 1e-10 &&
               std::abs(p1.longitude - p2.longitude) < 1e-10;
    };

    for (int obs_idx = 0; obs_idx < static_cast<int>(obstacles.size()); ++obs_idx) {
        if (obs_idx == skip_obs_a || obs_idx == skip_obs_b) continue;

        const auto& poly = obstacles[obs_idx];
        int n = static_cast<int>(poly.size());
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            if (posMatch(a, poly[i]) || posMatch(a, poly[j]) ||
                posMatch(b, poly[i]) || posMatch(b, poly[j]))
                continue;
            if (geo::segmentsIntersect(a, b, poly[i], poly[j])) {
                return false;
            }
        }

        for (double t : {0.25, 0.5, 0.75}) {
            core::Position sample{
                a.latitude + (b.latitude - a.latitude) * t,
                a.longitude + (b.longitude - a.longitude) * t,
                a.altitude + (b.altitude - a.altitude) * t};
            if (!geo::isPointInPolygon(sample, poly)) continue;

            double edge_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < n; ++i) {
                int j = (i + 1) % n;
                edge_dist = std::min(edge_dist, geo::distanceToSegment(sample, poly[i], poly[j]));
            }
            if (edge_dist > 0.5) {
                return false;
            }
        }
    }

    if (boundary.isValid()) {
        int n = static_cast<int>(boundary.vertices.size());
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            if (geo::segmentsIntersect(a, b, boundary.vertices[i], boundary.vertices[j])) {
                return false;
            }
        }
        core::Position mid{
            (a.latitude + b.latitude) / 2.0,
            (a.longitude + b.longitude) / 2.0,
            (a.altitude + b.altitude) / 2.0};
        if (!geo::isPointInPolygon(mid, boundary.vertices)) {
            return false;
        }
    }

    return true;
}

int PathPlanner::findContainingObstacle(
    const core::Position& pos,
    const std::vector<std::vector<core::Position>>& obstacles) const {
    for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
        if (geo::isPointInPolygon(pos, obstacles[i])) return i;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Dijkstra shortest path
// ---------------------------------------------------------------------------

std::vector<int> PathPlanner::dijkstra(const AdjList& adj, int src, int dst) const {
    int n = static_cast<int>(adj.size());
    std::vector<double> dist(n, std::numeric_limits<double>::max());
    std::vector<int> prev(n, -1);

    using PQEntry = std::pair<double, int>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        if (u == dst) break;

        for (auto [v, w] : adj[u]) {
            double nd = dist[u] + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (dist[dst] == std::numeric_limits<double>::max()) {
        return {};
    }

    std::vector<int> path;
    for (int v = dst; v != -1; v = prev[v]) {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ---------------------------------------------------------------------------
// Path simplification: remove redundant waypoints with line-of-sight
// ---------------------------------------------------------------------------

std::vector<core::Position> PathPlanner::simplifyPath(
    const std::vector<core::Position>& path,
    const std::vector<std::vector<core::Position>>& obstacles,
    const core::Boundary& boundary) const {

    if (path.size() <= 2) return path;

    std::vector<core::Position> simplified;
    simplified.push_back(path.front());

    size_t current = 0;
    while (current < path.size() - 1) {
        size_t farthest = current + 1;
        for (size_t candidate = path.size() - 1; candidate > current + 1; --candidate) {
            if (isEdgeVisible(path[current], path[candidate], obstacles, boundary)) {
                farthest = candidate;
                break;
            }
        }
        simplified.push_back(path[farthest]);
        current = farthest;
    }

    return simplified;
}

// ---------------------------------------------------------------------------
// Turn smoothing: insert arc waypoints at sharp corners
// ---------------------------------------------------------------------------

std::vector<core::Position> PathPlanner::smoothTurns(
    const std::vector<core::Position>& path,
    const core::Boundary& boundary,
    const std::vector<core::HSSZone>& hss_zones) const {

    if (path.size() <= 2 || config_.min_turn_radius_m < 1.0) return path;

    const double R = config_.min_turn_radius_m;
    const double kMinTurnDeg = 15.0;
    constexpr double kPi = 3.14159265358979323846;
    const double max_step_rad = std::max(2.0, config_.max_arc_step_deg) * geo::kDegToRad;

    std::vector<core::Position> smoothed;
    smoothed.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& A = path[i - 1];
        const auto& B = path[i];
        const auto& C = path[i + 1];

        auto nedA = geo::positionToNED(A, B);
        auto nedC = geo::positionToNED(C, B);

        double lenAB = std::sqrt(nedA.north * nedA.north + nedA.east * nedA.east);
        double lenBC = std::sqrt(nedC.north * nedC.north + nedC.east * nedC.east);

        if (lenAB < 5.0 || lenBC < 5.0) {
            smoothed.push_back(B);
            continue;
        }

        double din_n = -nedA.north / lenAB;
        double din_e = -nedA.east / lenAB;
        double dout_n = nedC.north / lenBC;
        double dout_e = nedC.east / lenBC;

        double cross = din_e * dout_n - din_n * dout_e;
        double dot = din_n * dout_n + din_e * dout_e;
        double turn_angle = std::acos(std::clamp(dot, -1.0, 1.0));

        if (turn_angle < kMinTurnDeg * geo::kDegToRad) {
            smoothed.push_back(B);
            continue;
        }

        double half_turn = turn_angle / 2.0;
        double tan_dist = R / std::tan(half_turn);

        double max_tan = std::min(lenAB, lenBC) * 0.4;
        double actual_R = R;
        if (tan_dist > max_tan) {
            tan_dist = max_tan;
            actual_R = tan_dist * std::tan(half_turn);
        }

        double t1_n = (nedA.north / lenAB) * tan_dist;
        double t1_e = (nedA.east / lenAB) * tan_dist;
        double t2_n = (nedC.north / lenBC) * tan_dist;
        double t2_e = (nedC.east / lenBC) * tan_dist;

        double perp_n, perp_e;
        if (cross < 0) {
            perp_n = -din_e; perp_e = din_n;
        } else {
            perp_n = din_e;  perp_e = -din_n;
        }

        double center_n = t1_n + perp_n * actual_R;
        double center_e = t1_e + perp_e * actual_R;

        double angle_t1 = std::atan2(t1_e - center_e, t1_n - center_n);
        double angle_t2 = std::atan2(t2_e - center_e, t2_n - center_n);

        double arc_span = angle_t2 - angle_t1;
        if (cross < 0 && arc_span < 0) arc_span += 2.0 * kPi;
        if (cross > 0 && arc_span > 0) arc_span -= 2.0 * kPi;

        int n_pts = std::max(3, static_cast<int>(std::ceil(std::abs(arc_span) / max_step_rad)));

        bool arc_safe = true;
        std::vector<core::Position> arc_points;
        arc_points.reserve(n_pts + 1);

        for (int j = 0; j <= n_pts; ++j) {
            double t = static_cast<double>(j) / n_pts;
            double angle = angle_t1 + arc_span * t;
            double pt_n = center_n + actual_R * std::cos(angle);
            double pt_e = center_e + actual_R * std::sin(angle);

            geo::NED pt_ned{pt_n, pt_e, 0.0};
            auto pos = geo::nedToPosition(pt_ned, B);
            pos.altitude = B.altitude;

            if (!isPointSafe(pos, boundary, hss_zones, planningHSSMargin())) {
                arc_safe = false;
                break;
            }

            arc_points.push_back(pos);
        }

        if (arc_safe && !arc_points.empty()) {
            if (!isSegmentSafe(smoothed.back(), arc_points.front(), boundary, hss_zones,
                               planningHSSMargin())) {
                smoothed.push_back(B);
                continue;
            }
            bool arc_segments_safe = true;
            for (size_t j = 1; j < arc_points.size(); ++j) {
                if (!isSegmentSafe(arc_points[j - 1], arc_points[j], boundary, hss_zones,
                                   planningHSSMargin())) {
                    arc_segments_safe = false;
                    break;
                }
            }
            if (!arc_segments_safe) {
                smoothed.push_back(B);
                continue;
            }
            for (const auto& ap : arc_points) {
                smoothed.push_back(ap);
            }
        } else {
            smoothed.push_back(B);
        }
    }

    smoothed.push_back(path.back());
    if (!isPathSafe(smoothed, boundary, hss_zones, config_.hss_buffer_m)) {
        return path;
    }
    return smoothed;
}

std::vector<core::Position> PathPlanner::densifyPath(
    const std::vector<core::Position>& path) const {

    if (path.size() < 2 || config_.max_segment_length_m <= 1.0) return path;

    std::vector<core::Position> dense;
    dense.push_back(path.front());

    for (size_t i = 1; i < path.size(); ++i) {
        const auto a = dense.back();
        const auto b = path[i];
        const double dist = geo::haversineDistance(a, b);
        const int pieces = std::max(1, static_cast<int>(std::ceil(dist / config_.max_segment_length_m)));

        auto ned = geo::positionToNED(b, a);
        for (int j = 1; j <= pieces; ++j) {
            const double t = static_cast<double>(j) / pieces;
            core::Position p = geo::nedToPosition(
                geo::NED{ned.north * t, ned.east * t, ned.down * t}, a);
            dense.push_back(p);
        }
    }

    return dense;
}

bool PathPlanner::isPointSafe(
    const core::Position& pos,
    const core::Boundary& boundary,
    const std::vector<core::HSSZone>& hss_zones,
    double hss_margin_m) const {

    if (boundary.isValid()) {
        if (!geo::isPointInPolygon(pos, boundary.vertices)) return false;
        const double boundary_dist = zone_checker_->distanceToBoundary(pos, boundary);
        if (boundary_dist < config_.boundary_buffer_m) return false;
    }

    for (const auto& zone : hss_zones) {
        if (!zone.active) continue;
        const double d = geo::haversineDistance(pos, zone.center);
        if (d < zone.radius + hss_margin_m) return false;
    }

    return true;
}

bool PathPlanner::isSegmentSafe(
    const core::Position& a, const core::Position& b,
    const core::Boundary& boundary,
    const std::vector<core::HSSZone>& hss_zones,
    double hss_margin_m) const {

    if (!isPointSafe(a, boundary, hss_zones, hss_margin_m)) return false;
    if (!isPointSafe(b, boundary, hss_zones, hss_margin_m)) return false;

    for (const auto& zone : hss_zones) {
        if (!zone.active) continue;
        if (geo::segmentIntersectsCircle(a, b, zone.center, zone.radius + hss_margin_m)) {
            return false;
        }
    }

    if (boundary.isValid() && zone_checker_->segmentExitsBoundary(a, b, boundary)) {
        return false;
    }

    const double dist = geo::haversineDistance(a, b);
    const int samples = std::max(1, static_cast<int>(std::ceil(dist / std::max(20.0, config_.max_segment_length_m * 0.5))));
    auto ned = geo::positionToNED(b, a);
    for (int i = 1; i < samples; ++i) {
        const double t = static_cast<double>(i) / samples;
        auto p = geo::nedToPosition(geo::NED{ned.north * t, ned.east * t, ned.down * t}, a);
        if (!isPointSafe(p, boundary, hss_zones, hss_margin_m)) return false;
    }

    return true;
}

bool PathPlanner::isPathSafe(
    const std::vector<core::Position>& path,
    const core::Boundary& boundary,
    const std::vector<core::HSSZone>& hss_zones,
    double hss_margin_m) const {

    if (path.empty()) return false;
    for (const auto& p : path) {
        if (!isPointSafe(p, boundary, hss_zones, hss_margin_m)) return false;
    }
    for (size_t i = 1; i < path.size(); ++i) {
        if (!isSegmentSafe(path[i - 1], path[i], boundary, hss_zones, hss_margin_m)) {
            return false;
        }
    }
    return true;
}

} // namespace uav::nav
