#include "uav/mission/mission_director.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>

namespace uav::mission {

const char* toString(PostDiveAction action) {
    switch (action) {
        case PostDiveAction::RTL:    return "RTL";
        case PostDiveAction::PATROL: return "PATROL";
        case PostDiveAction::LOITER: return "LOITER";
    }
    return "UNKNOWN";
}

PostDiveAction parsePostDiveAction(const std::string& s, PostDiveAction fallback) {
    std::string lower;
    lower.reserve(s.size());
    for (char c : s) lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (lower == "rtl")    return PostDiveAction::RTL;
    if (lower == "patrol") return PostDiveAction::PATROL;
    if (lower == "loiter") return PostDiveAction::LOITER;
    return fallback;
}

MissionDirector::MissionDirector(
    std::shared_ptr<nav::NavigationEngine> nav,
    std::shared_ptr<comm::FlightCommander> commander,
    std::shared_ptr<world_model::WorldModel> world_model,
    std::shared_ptr<infra::EventBus> event_bus,
    std::shared_ptr<perception::PerceptionEngine> perception,
    Config config)
    : nav_(std::move(nav)),
      commander_(std::move(commander)),
      world_model_(std::move(world_model)),
      event_bus_(std::move(event_bus)),
      perception_(std::move(perception)),
      config_(config) {}

void MissionDirector::initialize() {
    ardupilot_ = std::make_unique<ArduPilotPassthrough>(commander_);
    patrol_    = std::make_unique<PatrolMode>(nav_, world_model_, config_.patrol);
    follow_    = std::make_unique<FollowMode>(nav_, world_model_, config_.follow);
    qr_dive_   = std::make_unique<QRDiveMode>(nav_, world_model_, commander_, config_.qr_dive);

    if (perception_) {
        follow_->setPerceptionEngine(perception_, event_bus_);
        qr_dive_->setPerceptionEngine(perception_, event_bus_);
    }

    qr_dive_->setCompletionCallback([this] {
        onQRDiveComplete();
    });

    follow_->setTargetLostCallback([this] {
        LOG_INFO("MissionDirector: target permanently lost, switching to PATROL");
        requestMode(core::MissionMode::PATROL);
    });

    ardupilot_->onEnter();
}

void MissionDirector::onQRDiveComplete() {
    LOG_INFO("MissionDirector: QR_DIVE completed, on_complete={}",
             toString(config_.qr_dive_on_complete));

    // Always release stale GUIDED state before next mode takes over.
    if (nav_) (void)nav_->releaseGuidedHold();

    switch (config_.qr_dive_on_complete) {
        case PostDiveAction::PATROL:
            requestMode(core::MissionMode::PATROL);
            return;
        case PostDiveAction::LOITER: {
            // Demote to passthrough and command ArduPilot LOITER directly.
            forceMode(core::MissionMode::ARDUPILOT);
            (void)commander_->setMode(comm::ArduPilotMode::LOITER);
            return;
        }
        case PostDiveAction::RTL:
        default:
            triggerRTL("qr_dive_complete");
            return;
    }
}

bool MissionDirector::requestMode(core::MissionMode mode) {
    std::lock_guard transition_lock(transition_mutex_);
    core::MissionMode from;
    {
        std::lock_guard lock(mode_mutex_);
        if (mode == current_mode_) return true;
        if (!validateTransition(current_mode_, mode)) {
            LOG_WARN("MissionDirector: invalid transition {} -> {}",
                     core::toString(current_mode_), core::toString(mode));
            return false;
        }
        from = current_mode_;
    }
    return executeTransition(from, mode);
}

void MissionDirector::forceMode(core::MissionMode mode) {
    std::lock_guard transition_lock(transition_mutex_);
    core::MissionMode from;
    {
        std::lock_guard lock(mode_mutex_);
        if (mode == current_mode_) return;
        from = current_mode_;
    }
    LOG_INFO("MissionDirector: operator force {} -> {}",
             core::toString(from), core::toString(mode));
    (void)executeTransition(from, mode);
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
    auto own = world_model_->getOwnState();
    if (!own.isValid()) return;

    std::lock_guard transition_lock(transition_mutex_);
    core::MissionMode mode_snapshot;
    IMissionMode* handler = nullptr;
    bool needs_operator_override = false;
    {
        std::lock_guard lock(mode_mutex_);
        mode_snapshot = current_mode_;
        // If the operator pulled us out of GUIDED while we were running an
        // autonomous mode, treat that as an explicit handover and demote to
        // ARDUPILOT instead of fighting the operator with repeated setMode.
        if (mode_snapshot != core::MissionMode::ARDUPILOT &&
            !commander_->isInGuidedMode()) {
            needs_operator_override = true;
        } else {
            handler = activeHandler();
        }
    }

    if (needs_operator_override) {
        LOG_WARN("MissionDirector: AP left GUIDED (mode={}); handing control back to operator (ARDUPILOT)",
                 comm::toString(commander_->getCurrentMode()));
        forceMode(core::MissionMode::ARDUPILOT);
        return;
    }

    if (handler) handler->tick(own);
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

    std::lock_guard transition_lock(transition_mutex_);
    std::lock_guard lock(mode_mutex_);
    auto* handler = activeHandler();
    if (handler) handler->onExit();

    current_mode_ = core::MissionMode::ARDUPILOT;
    transition_count_++;
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
    return handlerForMode(current_mode_);
}

IMissionMode* MissionDirector::handlerForMode(core::MissionMode mode) {
    switch (mode) {
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

    // QR_DIVE: only PATROL allowed as a clean exit (ARDUPILOT was handled above
    // as the universal safety fallback). Direct QR_DIVE -> FOLLOW is rejected
    // so the dive sequence can either complete or be cleanly aborted.
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

bool MissionDirector::executeTransition(core::MissionMode from,
                                        core::MissionMode to) {
    std::lock_guard transition_lock(transition_mutex_);
    LOG_INFO("MissionDirector: {} -> {}", core::toString(from), core::toString(to));

    // Run onExit on the *previous* handler without holding mode_mutex_.
    {
        IMissionMode* old_handler = nullptr;
        {
            std::lock_guard lock(mode_mutex_);
            old_handler = handlerForMode(current_mode_);
        }
        if (old_handler) old_handler->onExit();
    }

    // GUIDED MUST be active before onEnter (onEnter may send flight commands).
    // The setMode + confirmation wait is serialized by transition_mutex_ so
    // tick/event handlers cannot run the old and new mode concurrently.
    if (to != core::MissionMode::ARDUPILOT && !commander_->isInGuidedMode()) {
        if (!commander_->setMode(comm::ArduPilotMode::GUIDED)) {
            LOG_WARN("MissionDirector: failed to send GUIDED mode command; staying in ARDUPILOT");
            {
                std::lock_guard lock(mode_mutex_);
                current_mode_ = core::MissionMode::ARDUPILOT;
                transition_count_++;
            }
            ardupilot_->onEnter();
            event_bus_->publish(infra::EventType::MODE_CHANGED,
                                static_cast<int>(core::MissionMode::ARDUPILOT));
            return false;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!commander_->isInGuidedMode() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!commander_->isInGuidedMode()) {
            LOG_WARN("MissionDirector: GUIDED mode not confirmed; staying in ARDUPILOT");
            {
                std::lock_guard lock(mode_mutex_);
                current_mode_ = core::MissionMode::ARDUPILOT;
                transition_count_++;
            }
            ardupilot_->onEnter();
            event_bus_->publish(infra::EventType::MODE_CHANGED,
                                static_cast<int>(core::MissionMode::ARDUPILOT));
            return false;
        }
    }

    // Clear sticky guided_state (yaw target, last waypoint) on every transition
    // between autonomous handlers so the new handler's first command is not
    // overridden by leftovers from the previous one.
    if (from != to && nav_ && to != core::MissionMode::ARDUPILOT) {
        (void)nav_->releaseGuidedHold();
    }

    IMissionMode* new_handler = nullptr;
    {
        std::lock_guard lock(mode_mutex_);
        current_mode_ = to;
        transition_count_++;
        new_handler = handlerForMode(to);
    }

    if (new_handler) new_handler->onEnter();

    event_bus_->publish(infra::EventType::MODE_CHANGED,
                        static_cast<int>(to));
    return true;
}

} // namespace uav::mission
