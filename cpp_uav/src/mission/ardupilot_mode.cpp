#include "uav/mission/ardupilot_mode.hpp"
#include "uav/infra/log_manager.hpp"

namespace uav::mission {

ArduPilotPassthrough::ArduPilotPassthrough(std::shared_ptr<comm::FlightCommander> commander)
    : commander_(std::move(commander)) {}

void ArduPilotPassthrough::onEnter() {
    LOG_INFO("ArduPilotPassthrough: entering - ArduPilot has full control");
}

void ArduPilotPassthrough::onExit() {
    LOG_INFO("ArduPilotPassthrough: exiting");
}

void ArduPilotPassthrough::tick(const core::UAVState& /*own_state*/) {
    // ArduPilot controls the aircraft. Nothing to do.
}

core::MissionGoal ArduPilotPassthrough::getMissionGoal() {
    return core::MissionGoal(core::GoalType::NONE);
}

} // namespace uav::mission
