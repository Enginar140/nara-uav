#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/mission/patrol_mode.hpp"
#include "uav/mission/follow_mode.hpp"
#include "uav/mission/qr_dive_mode.hpp"
#include "uav/mission/ardupilot_mode.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/perception/perception_engine.hpp"
#include "uav/world_model/world_model.hpp"
#include "uav/infra/event_bus.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace uav::mission {

/// Post-QR_DIVE behaviour. Driven by `mission.qr_dive.on_complete` in yaml.
enum class PostDiveAction {
    RTL,     // hand control to ArduPilot RTL (default; safest)
    PATROL,  // resume autonomous patrol pattern
    LOITER   // ArduPilot LOITER mode (in-place orbit at current altitude)
};

const char* toString(PostDiveAction action);
PostDiveAction parsePostDiveAction(const std::string& s, PostDiveAction fallback);

class MissionDirector {
public:
    struct Config {
        PatrolMode::Config patrol;
        FollowMode::Config follow;
        QRDiveMode::Config qr_dive;
        PostDiveAction qr_dive_on_complete = PostDiveAction::RTL;
        bool follow_vision_fallback = false;
        std::string follow_vision_module = "vision_tracker";
        std::string qr_dive_perception_module = "qr_simple";
    };

    MissionDirector(std::shared_ptr<nav::NavigationEngine> nav,
                    std::shared_ptr<comm::FlightCommander> commander,
                    std::shared_ptr<world_model::WorldModel> world_model,
                    std::shared_ptr<infra::EventBus> event_bus,
                    std::shared_ptr<perception::PerceptionEngine> perception,
                    Config config = {});

    void initialize();

    bool requestMode(core::MissionMode mode);
    void forceMode(core::MissionMode mode);
    core::MissionMode getCurrentMode() const;

    void setFollowTarget(int target_id);
    void setDiveTarget(const core::Position& target);

    void tick();

    void handleModeCommand(const comm::ParsedModeCommand& cmd);

    void triggerRTL(const std::string& reason);

    struct Stats {
        core::MissionMode current_mode = core::MissionMode::ARDUPILOT;
        uint64_t mode_transitions = 0;
        std::string active_phase;
    };
    Stats getStats() const;

private:
    std::shared_ptr<nav::NavigationEngine> nav_;
    std::shared_ptr<comm::FlightCommander> commander_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    std::shared_ptr<infra::EventBus> event_bus_;
    std::shared_ptr<perception::PerceptionEngine> perception_;
    Config config_;

    void onQRDiveComplete();

    std::unique_ptr<PatrolMode> patrol_;
    std::unique_ptr<FollowMode> follow_;
    std::unique_ptr<QRDiveMode> qr_dive_;
    std::unique_ptr<ArduPilotPassthrough> ardupilot_;

    core::MissionMode current_mode_ = core::MissionMode::ARDUPILOT;
    mutable std::recursive_mutex mode_mutex_;
    mutable std::recursive_mutex transition_mutex_;
    uint64_t transition_count_ = 0;

    IMissionMode* activeHandler();
    IMissionMode* handlerForMode(core::MissionMode mode);
    bool validateTransition(core::MissionMode from, core::MissionMode to) const;
    bool executeTransition(core::MissionMode from, core::MissionMode to);
};

} // namespace uav::mission
