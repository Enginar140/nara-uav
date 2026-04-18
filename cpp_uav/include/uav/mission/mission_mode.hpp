#pragma once

#include "uav/core/types.hpp"
#include <string>

namespace uav::mission {

class IMissionMode {
public:
    virtual ~IMissionMode() = default;

    virtual void onEnter() = 0;
    virtual void onExit() = 0;
    virtual void tick(const core::UAVState& own_state) = 0;
    virtual core::MissionGoal getMissionGoal() = 0;
    virtual std::string getName() const = 0;
};

} // namespace uav::mission
