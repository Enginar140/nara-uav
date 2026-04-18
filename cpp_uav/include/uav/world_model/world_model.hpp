#pragma once

#include "uav/core/types.hpp"

#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace uav::world_model {

class WorldModel {
public:
    WorldModel() = default;
    ~WorldModel() = default;

    // Own UAV state
    void setOwnState(const core::UAVState& state);
    core::UAVState getOwnState() const;

    // Opponents
    void updateOpponent(const core::UAVState& state);
    void removeOpponent(int id);
    std::optional<core::UAVState> getOpponent(int id) const;
    std::vector<core::UAVState> getAllOpponents() const;
    std::optional<core::UAVState> getNearestOpponent(const core::Position& pos) const;
    size_t getOpponentCount() const;
    void removeStaleOpponents(double max_age_s);

    // HSS zones
    void updateHSSZone(const core::HSSZone& zone);
    void removeHSSZone(int id);
    std::vector<core::HSSZone> getAllHSSZones() const;
    std::vector<core::HSSZone> getActiveHSSZones() const;

    // Boundary
    void setBoundary(const core::Boundary& boundary);
    std::optional<core::Boundary> getBoundary() const;
    bool hasBoundary() const;

private:
    core::UAVState own_state_;
    mutable std::shared_mutex own_mutex_;

    std::unordered_map<int, core::UAVState> opponents_;
    mutable std::shared_mutex opp_mutex_;

    std::unordered_map<int, core::HSSZone> hss_zones_;
    mutable std::shared_mutex hss_mutex_;

    std::optional<core::Boundary> boundary_;
    mutable std::shared_mutex bnd_mutex_;
};

} // namespace uav::world_model
