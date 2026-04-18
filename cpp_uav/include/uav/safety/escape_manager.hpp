#pragma once

#include "uav/core/types.hpp"
#include "uav/safety/zone_checker.hpp"
#include "uav/world_model/world_model.hpp"

#include <memory>
#include <optional>

namespace uav::safety {

class SafetyGate;

class EscapeManager {
public:
    EscapeManager(std::shared_ptr<ZoneChecker> checker,
                  std::shared_ptr<world_model::WorldModel> world_model,
                  std::shared_ptr<SafetyGate> safety_gate = nullptr);

    void setSafetyGate(std::shared_ptr<SafetyGate> gate) { safety_gate_ = std::move(gate); }

    std::optional<core::Position> generateEscapeTarget(const core::UAVState& own) const;
    std::optional<core::Position> generateBoundaryEscape(const core::UAVState& own,
                                                         const core::Boundary& bnd) const;
    std::optional<core::Position> generateHSSEscape(const core::UAVState& own,
                                                    const std::vector<core::HSSZone>& zones) const;
    std::optional<core::Position> generateCollisionEscape(const core::UAVState& own,
                                                          const core::Threat& threat) const;

private:
    std::shared_ptr<ZoneChecker> checker_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    std::shared_ptr<SafetyGate> safety_gate_;

    std::optional<core::Position> validateOrFindAlternative(
        const core::Position& candidate,
        const core::Position& own_pos,
        double base_bearing, double escape_dist) const;
};

} // namespace uav::safety
