#include "uav/world_model/world_model.hpp"
#include "uav/core/geo.hpp"

namespace uav::world_model {

// --- Own state ---

void WorldModel::setOwnState(const core::UAVState& state) {
    std::unique_lock lock(own_mutex_);
    own_state_ = state;
}

core::UAVState WorldModel::getOwnState() const {
    std::shared_lock lock(own_mutex_);
    return own_state_;
}

// --- Opponents ---

void WorldModel::updateOpponent(const core::UAVState& state) {
    std::unique_lock lock(opp_mutex_);
    opponents_[state.id] = state;
}

void WorldModel::removeOpponent(int id) {
    std::unique_lock lock(opp_mutex_);
    opponents_.erase(id);
}

std::optional<core::UAVState> WorldModel::getOpponent(int id) const {
    std::shared_lock lock(opp_mutex_);
    auto it = opponents_.find(id);
    if (it != opponents_.end()) return it->second;
    return std::nullopt;
}

std::vector<core::UAVState> WorldModel::getAllOpponents() const {
    std::shared_lock lock(opp_mutex_);
    std::vector<core::UAVState> result;
    result.reserve(opponents_.size());
    for (const auto& [id, state] : opponents_) {
        result.push_back(state);
    }
    return result;
}

std::optional<core::UAVState> WorldModel::getNearestOpponent(
    const core::Position& pos) const {
    std::shared_lock lock(opp_mutex_);
    if (opponents_.empty()) return std::nullopt;

    double min_dist = std::numeric_limits<double>::max();
    const core::UAVState* nearest = nullptr;

    for (const auto& [id, state] : opponents_) {
        double d = geo::haversineDistance(pos, state.position);
        if (d < min_dist) {
            min_dist = d;
            nearest = &state;
        }
    }
    if (nearest) return *nearest;
    return std::nullopt;
}

size_t WorldModel::getOpponentCount() const {
    std::shared_lock lock(opp_mutex_);
    return opponents_.size();
}

void WorldModel::removeStaleOpponents(double max_age_s) {
    std::unique_lock lock(opp_mutex_);
    for (auto it = opponents_.begin(); it != opponents_.end();) {
        if (it->second.ageSeconds() > max_age_s) {
            it = opponents_.erase(it);
        } else {
            ++it;
        }
    }
}

// --- HSS Zones ---

void WorldModel::updateHSSZone(const core::HSSZone& zone) {
    std::unique_lock lock(hss_mutex_);
    hss_zones_[zone.id] = zone;
}

void WorldModel::removeHSSZone(int id) {
    std::unique_lock lock(hss_mutex_);
    hss_zones_.erase(id);
}

std::vector<core::HSSZone> WorldModel::getAllHSSZones() const {
    std::shared_lock lock(hss_mutex_);
    std::vector<core::HSSZone> result;
    result.reserve(hss_zones_.size());
    for (const auto& [id, zone] : hss_zones_) {
        result.push_back(zone);
    }
    return result;
}

std::vector<core::HSSZone> WorldModel::getActiveHSSZones() const {
    std::shared_lock lock(hss_mutex_);
    std::vector<core::HSSZone> result;
    for (const auto& [id, zone] : hss_zones_) {
        if (zone.active) result.push_back(zone);
    }
    return result;
}

// --- Boundary ---

void WorldModel::setBoundary(const core::Boundary& boundary) {
    std::unique_lock lock(bnd_mutex_);
    boundary_ = boundary;
}

std::optional<core::Boundary> WorldModel::getBoundary() const {
    std::shared_lock lock(bnd_mutex_);
    return boundary_;
}

bool WorldModel::hasBoundary() const {
    std::shared_lock lock(bnd_mutex_);
    return boundary_.has_value() && boundary_->isValid();
}

} // namespace uav::world_model
