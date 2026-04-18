#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/comm/flight_commander.hpp"

#include <memory>

namespace uav::mission {

class ArduPilotPassthrough : public IMissionMode {
public:
    explicit ArduPilotPassthrough(std::shared_ptr<comm::FlightCommander> commander);

    void onEnter() override;
    void onExit() override;
    void tick(const core::UAVState& own_state) override;
    core::MissionGoal getMissionGoal() override;
    std::string getName() const override { return "ARDUPILOT"; }

private:
    std::shared_ptr<comm::FlightCommander> commander_;
};

} // namespace uav::mission
