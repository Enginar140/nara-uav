#pragma once

#include "uav/config/config_manager.hpp"
#include "uav/infra/event_bus.hpp"
#include "uav/infra/thread_pool.hpp"
#include "uav/world_model/world_model.hpp"
#include "uav/comm/mavlink_bridge.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/safety/zone_checker.hpp"
#include "uav/safety/safety_gate.hpp"
#include "uav/safety/safety_monitor.hpp"
#include "uav/safety/escape_manager.hpp"
#include "uav/nav/path_planner.hpp"
#include "uav/nav/speed_manager.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/perception/perception_engine.hpp"
#include "uav/perception/simple_qr_detector.hpp"
#include "uav/perception/simple_vision_tracker.hpp"
#ifdef UAV_HAS_OPENCV
#include "uav/perception/opencv_vision_tracker.hpp"
#endif
#include "uav/mission/mission_director.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace uav::app {

class Application {
public:
    explicit Application(const std::string& config_path);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    bool initialize();
    bool start();
    void run();
    void stop();
    void requestShutdown();

    static void setupSignalHandlers();

private:
    std::string config_path_;
    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_requested_{false};

    std::shared_ptr<config::ConfigManager> config_;
    std::shared_ptr<infra::EventBus> event_bus_;
    std::shared_ptr<infra::ThreadPool> thread_pool_;
    std::shared_ptr<world_model::WorldModel> world_model_;

    std::shared_ptr<comm::MAVLinkBridge> mavlink_;
    std::shared_ptr<comm::FlightCommander> commander_;

    std::shared_ptr<safety::ZoneChecker> zone_checker_;
    std::shared_ptr<safety::SafetyGate> safety_gate_;
    std::shared_ptr<safety::SafetyMonitor> safety_monitor_;
    std::shared_ptr<safety::EscapeManager> escape_manager_;

    std::shared_ptr<nav::PathPlanner> planner_;
    std::shared_ptr<nav::SpeedManager> speed_manager_;
    std::shared_ptr<nav::NavigationEngine> nav_engine_;

    std::shared_ptr<perception::PerceptionEngine> perception_;
    std::shared_ptr<perception::SimpleVisionTracker> vision_tracker_;
    std::shared_ptr<perception::SimpleQRDetector> qr_detector_;
#ifdef UAV_HAS_OPENCV
    std::shared_ptr<perception::OpenCvVisionTracker> opencv_vision_tracker_;
#endif

    std::shared_ptr<mission::MissionDirector> mission_;

    bool initCore();
    bool initComm();
    bool initSafety();
    bool initNav();
    bool initPerception();
    bool initMission();
    void subscribeEvents();
    void loadBoundaryFromConfig();
    void loadHSSFromConfig();

    void mainLoop();
    void printDashboard() const;

    void handleFailsafe(const infra::Event& event);
    void handleEscapeNeeded(const core::SafetyState& safety);
    void triggerRTL(const std::string& reason);
    void applyArduPilotParams();

    static Application* instance_;
    static void signalHandler(int sig);
};

} // namespace uav::app
