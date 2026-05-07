#include "uav/nav/waypoint_chain.hpp"
#include "uav/core/geo.hpp"

#include <limits>

namespace uav::nav {

void WaypointChain::load(std::vector<Waypoint> chain, bool circular) {
    chain_ = std::move(chain);
    circular_ = circular;
    idx_ = 0;
    complete_ = chain_.empty();
}

ChainStatus WaypointChain::update(const core::Position& own_pos) {
    if (chain_.empty()) return ChainStatus::EMPTY;
    if (complete_ && !circular_) return ChainStatus::CHAIN_COMPLETE;

    const auto& target = chain_[idx_];
    double dist = geo::haversineDistance(own_pos, target.position);

    if (dist <= target.acceptance_radius_m) {
        idx_++;
        if (idx_ >= chain_.size()) {
            if (circular_) {
                idx_ = 0;
            } else {
                complete_ = true;
                return ChainStatus::CHAIN_COMPLETE;
            }
        }
        return ChainStatus::WAYPOINT_REACHED;
    }

    return ChainStatus::IN_PROGRESS;
}

std::optional<Waypoint> WaypointChain::currentTarget() const {
    if (chain_.empty() || (complete_ && !circular_)) return std::nullopt;
    return chain_[idx_];
}

std::optional<Waypoint> WaypointChain::peekNext() const {
    if (chain_.empty() || (complete_ && !circular_)) return std::nullopt;
    size_t next = idx_ + 1;
    if (next >= chain_.size()) {
        if (circular_) next = 0;
        else return std::nullopt;
    }
    return chain_[next];
}

std::vector<Waypoint> WaypointChain::remainingWaypoints() const {
    if (chain_.empty() || complete_) return {};
    return {chain_.begin() + static_cast<long>(idx_), chain_.end()};
}

void WaypointChain::clear() {
    chain_.clear();
    idx_ = 0;
    circular_ = false;
    complete_ = false;
}

void WaypointChain::replace(std::vector<Waypoint> new_chain) {
    if (new_chain.empty()) {
        clear();
        return;
    }

    chain_ = std::move(new_chain);
    complete_ = false;
    idx_ = 0;
}

bool WaypointChain::validate(const safety::ZoneChecker& checker,
                             const core::Boundary& boundary,
                             const std::vector<core::HSSZone>& zones,
                             double hss_buffer) const {
    for (size_t i = idx_; i < chain_.size(); ++i) {
        if (!checker.isPositionSafe(chain_[i].position, boundary, zones, hss_buffer)) {
            return false;
        }
        if (i > idx_) {
            if (checker.segmentIntersectsAnyHSS(
                    chain_[i - 1].position, chain_[i].position, zones, hss_buffer)) {
                return false;
            }
            if (checker.segmentExitsBoundary(chain_[i - 1].position,
                                             chain_[i].position,
                                             boundary)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace uav::nav
