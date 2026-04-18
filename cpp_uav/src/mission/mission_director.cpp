#include "uav/mission/mission_director.hpp"
#include "uav/infra/log_manager.hpp"

namespace uav::mission {

MissionDirector::MissionDirector(
    std::shared_ptr<nav::NavigationEngine> nav,
    std::shared_ptr<comm::FlightCommander> commander,
    std::shared_ptr<world_model::WorldModel> world_model,
    std::shared_ptr<infra::EventBus> event_bus,
    Config config)
    : nav_(std::move(nav)),
      commander_(std::move(commander)),
      world_model_(std::move(world_model)),
      event_bus_(std::move(event_bus)),
      config_(config) {}

void MissionDirector::initialize() {
    ardupilot_ = std::make_unique<ArduPilotPassthrough>(commander_);
    patrol_    = std::make_unique<PatrolMode>(nav_, world_model_, config_.patrol);
    follow_    = std::make_unique<FollowMode>(nav_, world_model_, config_.follow);
    qr_dive_   = std::make_unique<QRDiveMode>(nav_, world_model_, commander_, config_.qr_dive);

    qr_dive_->setCompletionCallback([this] {
        LOG_INFO("MissionDirector: QR_DIVE completed, switching to PATROL");
        requestMode(core::MissionMode::PATROL);
    });

    follow_->setTargetLostCallback([this] {
        LOG_INFO("MissionDirector: target permanently lost, switching to PATROL");
        requestMode(core::MissionMode::PATROL);
    });

    ardupilot_->onEnter();
}

bool MissionDirector::requestMode(core::MissionMode mode) {
    std::lock_guard lock(mode_mutex_);

    if (mode == current_mode_) return true;

    if (!validateTransition(current_mode_, mode)) {
        LOG_WARN("MissionDirector: invalid transition {} -> {}",
                 core::toString(current_mode_), core::toString(mode));
        return false;
    }

    executeTransition(current_mode_, mode);
    return true;
}

void MissionDirector::forceMode(core::MissionMode mode) {
    std::lock_guard lock(mode_mutex_);
    if (mode == current_mode_) return;
    LOG_INFO("MissionDirector: operator force {} -> {}",
             core::toString(current_mode_), core::toString(mode));
    executeTransition(current_mode_, mode);
}

core::MissionMode MissionDirector::getCurrentMode() const {
    std::lock_guard lock(mode_mutex_);
    return current_mode_;
}

void MissionDirector::setFollowTarget(int target_id) {
    follow_->setTargetId(target_id);
}

void MissionDirector::setDiveTarget(const core::Position& target) {
    qr_dive_->setDiveTarget(target);
}

void MissionDirector::tick() {
    std::lock_guard lock(mode_mutex_);
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return;

    if (current_mode_ != core::MissionMode::ARDUPILOT && !commander_->isInGuidedMode()) {
        static auto last_retry = std::chrono::steady_clock::time_point{};
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_retry).count();
        if (elapsed > 2.0) {
            LOG_WARN("MissionDirector: AP not in GUIDED (current={}), retrying setMode",
                     comm::toString(commander_->getCurrentMode()));
            (void)commander_->setMode(comm::ArduPilotMode::GUIDED);
            last_retry = now;
        }
    }

    auto* handler = activeHandler();
    if (handler) {
        handler->tick(own);
    }
}

void MissionDirector::handleModeCommand(const comm::ParsedModeCommand& cmd) {
    LOG_INFO("MissionDirector: received mode command -> {}",
             core::toString(cmd.target_mode));

    if (cmd.target_id.has_value()) {
        setFollowTarget(*cmd.target_id);
    }
    if (cmd.target_pos.has_value()) {
        setDiveTarget(*cmd.target_pos);
    }

    forceMode(cmd.target_mode);
}

void MissionDirector::triggerRTL(const std::string& reason) {
    LOG_WARN("MissionDirector: RTL triggered - {}", reason);

    std::lock_guard lock(mode_mutex_);
    auto* handler = activeHandler();
    if (handler) handler->onExit();

    current_mode_ = core::MissionMode::ARDUPILOT;
    ardupilot_->onEnter();
    (void)commander_->setMode(comm::ArduPilotMode::RTL);

    event_bus_->publish(infra::EventType::MODE_CHANGED);
}

MissionDirector::Stats MissionDirector::getStats() const {
    std::lock_guard lock(mode_mutex_);
    Stats s;
    s.current_mode = current_mode_;
    s.mode_transitions = transition_count_;

    if (current_mode_ == core::MissionMode::FOLLOW) {
        s.active_phase = toString(follow_->getPhase());
    } else if (current_mode_ == core::MissionMode::QR_DIVE) {
        s.active_phase = toString(qr_dive_->getPhase());
    }

    return s;
}

IMissionMode* MissionDirector::activeHandler() {
    switch (current_mode_) {
        case core::MissionMode::ARDUPILOT: return ardupilot_.get();
        case core::MissionMode::PATROL:    return patrol_.get();
        case core::MissionMode::FOLLOW:    return follow_.get();
        case core::MissionMode::QR_DIVE:   return qr_dive_.get();
    }
    return ardupilot_.get();
}

bool MissionDirector::validateTransition(core::MissionMode from,
                                         core::MissionMode to) const {
    // ARDUPILOT → anything: always allowed (operator override)
    if (from == core::MissionMode::ARDUPILOT) return true;

    // anything → ARDUPILOT: always allowed (safety fallback)
    if (to == core::MissionMode::ARDUPILOT) return true;

    // QR_DIVE → only ARDUPILOT or PATROL (must complete or abort)
    if (from == core::MissionMode::QR_DIVE) {
        return to == core::MissionMode::PATROL;
    }

    // FOLLOW → PATROL or QR_DIVE
    if (from == core::MissionMode::FOLLOW) {
        return to == core::MissionMode::PATROL || to == core::MissionMode::QR_DIVE;
    }

    // PATROL → anything
    if (from == core::MissionMode::PATROL) return true;

    return true;
}

void MissionDirector::executeTransition(core::MissionMode from,
                                        core::MissionMode to) {
    LOG_INFO("MissionDirector: {} -> {}", core::toString(from), core::toString(to));

    auto* old_handler = activeHandler();
    if (old_handler) old_handler->onExit();

    // GUIDED mode MUST be set BEFORE onEnter, because onEnter sends flight commands
    if (to != core::MissionMode::ARDUPILOT) {
        if (!commander_->isInGuidedMode()) {
            (void)commander_->setMode(comm::ArduPilotMode::GUIDED);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    current_mode_ = to;
    transition_count_++;

    auto* new_handler = activeHandler();
    if (new_handler) new_handler->onEnter();

    event_bus_->publish(infra::EventType::MODE_CHANGED,
                        static_cast<int>(to));
}

} // namespace uav::mission
