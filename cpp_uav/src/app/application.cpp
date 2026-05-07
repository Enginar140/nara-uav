#include "uav/app/application.hpp"
#include "uav/infra/log_manager.hpp"
#include "uav/core/time_utils.hpp"
#ifdef UAV_HAS_OPENCV
#include "uav/perception/camera/opencv_camera.hpp"
#include "uav/perception/detector/round_detector.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>

namespace uav::app {

Application* Application::instance_ = nullptr;

Application::Application(const std::string& config_path)
    : config_path_(config_path) {
    instance_ = this;
}

Application::~Application() {
    stop();
    instance_ = nullptr;
}

bool Application::initialize() {
    if (!initCore())       return false;
    if (!initComm())       return false;
    if (!initSafety())     return false;
    if (!initNav())        return false;
    if (!initPerception()) return false;
    if (!initMission())    return false;
    subscribeEvents();
    loadBoundaryFromConfig();
    loadHSSFromConfig();
    return true;
}

bool Application::start() {
    LOG_INFO("Application: starting...");

    if (!mavlink_->start()) {
        LOG_ERROR("Application: failed to start MAVLink");
        return false;
    }

    if (!safety_monitor_->start()) {
        LOG_ERROR("Application: failed to start safety monitor");
        return false;
    }

    event_bus_->start();
    if (perception_) perception_->start();
    running_ = true;
    event_bus_->publish(infra::EventType::SYSTEM_STARTED);

    LOG_INFO("Application: started successfully");
    return true;
}

void Application::run() {
    LOG_INFO("Application: entering main loop");
    mainLoop();
    LOG_INFO("Application: main loop exited");
}

void Application::stop() {
    if (!running_) return;
    LOG_INFO("Application: stopping...");

    running_ = false;
    event_bus_->publish(infra::EventType::SYSTEM_STOPPING);

    if (perception_) perception_->stop();
    safety_monitor_->stop();
    mavlink_->stop();
    event_bus_->stop();

    infra::LogManager::instance().flush();
    LOG_INFO("Application: stopped");
}

void Application::requestShutdown() {
    shutdown_requested_ = true;
}

// =============================================================================
// Initialization
// =============================================================================

bool Application::initCore() {
    try {
        config_ = std::make_shared<config::ConfigManager>(config_path_);
    } catch (const config::ConfigException& e) {
        std::cerr << "Config error: " << e.what() << std::endl;
        return false;
    }

    infra::LogManager::instance().initialize(
        config_->getString("logging.dir", "logs"),
        config_->getString("logging.level", "info"),
        config_->getInt("logging.max_size_mb", 10),
        config_->getInt("logging.backup_count", 5));

    LOG_INFO("Application: config loaded from {}", config_path_);

    event_bus_ = std::make_shared<infra::EventBus>();
    thread_pool_ = std::make_shared<infra::ThreadPool>(
        config_->getInt("system.thread_pool_size", 4));
    world_model_ = std::make_shared<world_model::WorldModel>();

    return true;
}

bool Application::initComm() {
    auto pixhawk_conn = config_->getPixhawkConnection();
    auto gcs_conn = config_->getGCSConnection();

    mavlink_ = std::make_shared<comm::MAVLinkBridge>(
        pixhawk_conn, gcs_conn, thread_pool_, world_model_, event_bus_);
    commander_ = mavlink_->getFlightCommander();

    LOG_INFO("Application: comm initialized (pixhawk={}, gcs={})",
             pixhawk_conn, gcs_conn);
    return true;
}

bool Application::initSafety() {
    zone_checker_ = std::make_shared<safety::ZoneChecker>();

    safety::SafetyGate::Config gate_cfg;
    gate_cfg.hss_buffer_m = config_->getDouble("safety.hss_buffer_m", 40.0);
    gate_cfg.boundary_buffer_m = config_->getDouble("safety.boundary_buffer_m", 20.0);
    gate_cfg.min_altitude_m = config_->getDouble("safety.min_altitude_m", 30.0);
    gate_cfg.max_altitude_m = config_->getDouble("safety.max_altitude_m", 400.0);
    gate_cfg.max_velocity_ms = config_->getDouble("safety.max_velocity_ms", 40.0);

    safety_gate_ = std::make_shared<safety::SafetyGate>(
        zone_checker_, world_model_, gate_cfg);

    safety::SafetyMonitor::Config mon_cfg;
    mon_cfg.collision_critical_m = config_->getDouble("safety.collision_critical_m", 30.0);
    mon_cfg.collision_high_m = config_->getDouble("safety.collision_high_m", 80.0);
    mon_cfg.collision_medium_m = config_->getDouble("safety.collision_medium_m", 200.0);
    mon_cfg.boundary_warn_m = config_->getDouble("safety.boundary_warn_m", 30.0);
    mon_cfg.hss_buffer_m = config_->getDouble("safety.hss_buffer_m", 40.0);

    safety_monitor_ = std::make_shared<safety::SafetyMonitor>(
        world_model_, event_bus_, mon_cfg);
    safety_monitor_->setMonitoringRate(
        config_->getDouble("safety.monitoring_rate_hz", 20.0));

    escape_manager_ = std::make_shared<safety::EscapeManager>(
        zone_checker_, world_model_, safety_gate_);

    LOG_INFO("Application: safety initialized");
    return true;
}

bool Application::initNav() {
    nav::PathPlanner::Config plan_cfg;
    const double safety_hss_buffer = config_->getDouble("safety.hss_buffer_m", 40.0);
    const double safety_boundary_buffer = config_->getDouble("safety.boundary_buffer_m", 20.0);
    plan_cfg.hss_buffer_m = config_->getDouble("nav.planner.hss_buffer_m", safety_hss_buffer);
    plan_cfg.boundary_buffer_m = config_->getDouble(
        "nav.planner.boundary_buffer_m", safety_boundary_buffer);
    plan_cfg.min_turn_radius_m = config_->getDouble("nav.planner.min_turn_radius_m", 50.0);
    plan_cfg.turn_safety_margin_m = config_->getDouble(
        "nav.planner.turn_safety_margin_m", plan_cfg.min_turn_radius_m);
    plan_cfg.max_arc_step_deg = config_->getDouble("nav.planner.max_arc_step_deg", 10.0);
    plan_cfg.max_segment_length_m = config_->getDouble("nav.planner.max_segment_length_m", 80.0);
    plan_cfg.dubins_smoothing_enabled = config_->getBool(
        "nav.planner.dubins_smoothing_enabled", true);
    plan_cfg.polygon_segments = config_->getInt("nav.planner.polygon_segments", 12);

    planner_ = std::make_shared<nav::PathPlanner>(zone_checker_, plan_cfg);

    nav::SpeedManager::Config speed_cfg;
    speed_cfg.cruise = static_cast<float>(config_->getDouble("nav.speed.cruise", 20.0));
    speed_cfg.max_speed = static_cast<float>(config_->getDouble("nav.speed.max", 35.0));
    speed_cfg.min_speed = static_cast<float>(config_->getDouble("nav.speed.min", 12.0));
    speed_cfg.patrol = static_cast<float>(config_->getDouble("nav.speed.patrol", 18.0));
    speed_cfg.pursuit = static_cast<float>(config_->getDouble("nav.speed.pursuit", 30.0));
    speed_cfg.dive = static_cast<float>(config_->getDouble("nav.speed.dive", 25.0));

    speed_manager_ = std::make_shared<nav::SpeedManager>(speed_cfg);

    nav_engine_ = std::make_shared<nav::NavigationEngine>(
        planner_, commander_, safety_gate_, speed_manager_, world_model_);

    nav_engine_->setMinCommandInterval(std::chrono::milliseconds(
        config_->getInt("nav.command.min_interval_ms", 250)));
    nav_engine_->setMinPlanInterval(std::chrono::milliseconds(
        config_->getInt("nav.command.min_plan_interval_ms", 2000)));

    LOG_INFO("Application: navigation initialized");
    return true;
}

bool Application::initPerception() {
    perception::PerceptionEngine::Config pcfg;
    pcfg.tick_rate_hz = config_->getDouble("perception.tick_rate_hz", 10.0);
    pcfg.min_confidence = static_cast<float>(config_->getDouble("perception.min_confidence", 0.4));
    pcfg.lock_timeout_ms = config_->getDouble("perception.lock_timeout_ms", 1000.0);
    pcfg.publish_misses = config_->getBool("perception.publish_misses", true);

    perception_ = std::make_shared<perception::PerceptionEngine>(
        event_bus_, world_model_, pcfg);

    perception::SimpleVisionTracker::Config vt_cfg;
    vt_cfg.detection_timeout_s = config_->getDouble("perception.vision.detection_timeout_s", 1.0);
    vt_cfg.default_range_m = static_cast<float>(config_->getDouble("perception.vision.default_range_m", 100.0));
    vt_cfg.min_confidence = static_cast<float>(config_->getDouble("perception.vision.min_confidence", 0.5));
    vt_cfg.target_id = config_->getInt("perception.vision.target_id", -1);
    vt_cfg.use_world_fallback = config_->getBool("perception.vision.use_world_fallback", true);
    vision_tracker_ = std::make_shared<perception::SimpleVisionTracker>(world_model_, vt_cfg);

    perception::SimpleQRDetector::Config qr_cfg;
    qr_cfg.detection_timeout_s = config_->getDouble("perception.qr.detection_timeout_s", 0.7);
    qr_cfg.default_range_m = static_cast<float>(config_->getDouble("perception.qr.default_range_m", 60.0));
    qr_cfg.min_confidence = static_cast<float>(config_->getDouble("perception.qr.min_confidence", 0.6));
    qr_cfg.expected_payload = config_->getString("perception.qr.expected_payload", "");
    qr_detector_ = std::make_shared<perception::SimpleQRDetector>(qr_cfg);

    perception_->registerModule(vision_tracker_);
    perception_->registerModule(qr_detector_);

#ifdef UAV_HAS_OPENCV
    if (config_->getBool("perception.opencv_vision.enabled", false)) {
        perception::OpenCvCamera::Config cam_cfg;
        cam_cfg.type           = config_->getString("perception.opencv_vision.camera.type",         "device");
        cam_cfg.device_index   = config_->getInt   ("perception.opencv_vision.camera.device_index", 0);
        cam_cfg.url            = config_->getString("perception.opencv_vision.camera.url",          "");
        cam_cfg.file_path      = config_->getString("perception.opencv_vision.camera.file_path",    "");
        cam_cfg.width          = config_->getInt   ("perception.opencv_vision.camera.width",        640);
        cam_cfg.height         = config_->getInt   ("perception.opencv_vision.camera.height",       480);
        cam_cfg.fps            = config_->getDouble("perception.opencv_vision.camera.fps",          30.0);
        cam_cfg.loop_file      = config_->getBool  ("perception.opencv_vision.camera.loop_file",    true);
        auto camera = std::make_shared<perception::OpenCvCamera>(cam_cfg);

        perception::RoundDetector::Config det_cfg;
        det_cfg.dp                = config_->getDouble("perception.opencv_vision.detector.dp",                1.2);
        det_cfg.min_dist_factor   = config_->getDouble("perception.opencv_vision.detector.min_dist_factor",   0.25);
        det_cfg.param1            = config_->getDouble("perception.opencv_vision.detector.param1",            100.0);
        det_cfg.param2            = config_->getDouble("perception.opencv_vision.detector.param2",            30.0);
        det_cfg.min_radius_px     = config_->getInt   ("perception.opencv_vision.detector.min_radius_px",     15);
        det_cfg.max_radius_px     = config_->getInt   ("perception.opencv_vision.detector.max_radius_px",     200);
        det_cfg.gaussian_kernel   = config_->getInt   ("perception.opencv_vision.detector.gaussian_kernel",   9);
        det_cfg.use_hsv_mask      = config_->getBool  ("perception.opencv_vision.detector.use_hsv_mask",      false);
        det_cfg.hsv_lo_h1         = config_->getInt   ("perception.opencv_vision.detector.hsv_lo_h1",         0);
        det_cfg.hsv_hi_h1         = config_->getInt   ("perception.opencv_vision.detector.hsv_hi_h1",         10);
        det_cfg.hsv_lo_h2         = config_->getInt   ("perception.opencv_vision.detector.hsv_lo_h2",         170);
        det_cfg.hsv_hi_h2         = config_->getInt   ("perception.opencv_vision.detector.hsv_hi_h2",         179);
        det_cfg.hsv_lo_s          = config_->getInt   ("perception.opencv_vision.detector.hsv_lo_s",          100);
        det_cfg.hsv_hi_s          = config_->getInt   ("perception.opencv_vision.detector.hsv_hi_s",          255);
        det_cfg.hsv_lo_v          = config_->getInt   ("perception.opencv_vision.detector.hsv_lo_v",          80);
        det_cfg.hsv_hi_v          = config_->getInt   ("perception.opencv_vision.detector.hsv_hi_v",          255);
        det_cfg.base_confidence   = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.detector.base_confidence", 0.7));
        det_cfg.draw_overlay      = false; // tracker handles preview overlay
        auto detector = std::make_shared<perception::RoundDetector>(det_cfg);

        perception::OpenCvVisionTracker::Config tk_cfg;
        tk_cfg.id_tag             = config_->getString("perception.opencv_vision.id",                  "vision_tracker_opencv");
        tk_cfg.h_fov_deg          = config_->getDouble("perception.opencv_vision.h_fov_deg",           60.0);
        tk_cfg.v_fov_deg          = config_->getDouble("perception.opencv_vision.v_fov_deg",           45.0);
        tk_cfg.detection_timeout_s= config_->getDouble("perception.opencv_vision.detection_timeout_s", 1.0);
        tk_cfg.min_confidence     = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.min_confidence",    0.5));
        tk_cfg.fixed_range_m      = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.range.fixed_m",     50.0));
        tk_cfg.real_diameter_m    = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.range.real_diameter_m", 0.30));
        tk_cfg.min_range_m        = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.range.min_m",       1.0));
        tk_cfg.max_range_m        = static_cast<float>(
                                      config_->getDouble("perception.opencv_vision.range.max_m",       1000.0));
        const std::string range_mode = config_->getString("perception.opencv_vision.range.mode", "fixed");
        if      (range_mode == "size")     tk_cfg.range_mode = perception::OpenCvVisionTracker::RangeMode::SIZE;
        else if (range_mode == "altitude") tk_cfg.range_mode = perception::OpenCvVisionTracker::RangeMode::ALTITUDE;
        else                                tk_cfg.range_mode = perception::OpenCvVisionTracker::RangeMode::FIXED;
        tk_cfg.preview_enabled    = config_->getBool  ("perception.opencv_vision.preview.enabled",     false);
        tk_cfg.preview_title      = config_->getString("perception.opencv_vision.preview.title",       "uav vision (round)");
        tk_cfg.worker_sleep_ms    = config_->getInt   ("perception.opencv_vision.worker_sleep_ms",     5);

        opencv_vision_tracker_ = std::make_shared<perception::OpenCvVisionTracker>(
            camera, detector, tk_cfg);
        perception_->registerModule(opencv_vision_tracker_);
        LOG_INFO("Application: OpenCV vision tracker registered as '{}' (camera={}, detector={})",
                 tk_cfg.id_tag, cam_cfg.type, det_cfg.use_hsv_mask ? "round+hsv" : "round");

        // Debug-only: open the camera immediately at startup instead of waiting
        // for a mission mode to activate the module. Useful for verifying that
        // the webcam + detector + preview pipeline works before wiring it into
        // FOLLOW/QR_DIVE flows. Production builds should leave this false so
        // the camera only powers on when actually needed.
        if (config_->getBool("perception.opencv_vision.auto_start", false)) {
            opencv_vision_tracker_->start();
            LOG_INFO("Application: OpenCV vision tracker AUTO-STARTED (debug)");
        }
    } else {
        LOG_INFO("Application: OpenCV vision tracker disabled by config");
    }
#else
    if (config_->getBool("perception.opencv_vision.enabled", false)) {
        LOG_WARN("Application: perception.opencv_vision.enabled=true but build has UAV_HAS_OPENCV=OFF "
                 "(install OpenCV and re-run CMake to enable real-camera tracking)");
    }
#endif

    LOG_INFO("Application: perception initialized ({} modules)",
             perception_->registered().size());
    return true;
}

bool Application::initMission() {
    mission::MissionDirector::Config mcfg;

    mcfg.patrol.boundary_inset_m = config_->getDouble("mission.patrol.boundary_inset_m", 50.0);
    mcfg.patrol.cruise_altitude_m = config_->getDouble("mission.patrol.cruise_altitude_m", 100.0);
    mcfg.patrol.patrol_speed_ms = static_cast<float>(config_->getDouble("nav.speed.patrol", 18.0));

    mcfg.follow.pursuit_range_m = config_->getDouble("mission.follow.pursuit_range_m", 200.0);
    mcfg.follow.reacquire_timeout_s = config_->getDouble("mission.follow.reacquire_timeout_s", 5.0);
    mcfg.follow.max_reacquire_s = config_->getDouble("mission.follow.max_reacquire_s", 60.0);
    mcfg.follow.prediction_horizon_s = config_->getDouble("mission.follow.prediction_horizon_s", 3.0);
    mcfg.follow.target_stale_s = config_->getDouble("mission.follow.target_stale_s", 3.0);
    mcfg.follow.plan_update_interval_s = config_->getDouble("mission.follow.plan_update_interval_s", 3.0);
    mcfg.follow.approach_speed_ms = static_cast<float>(config_->getDouble("mission.follow.approach_speed_ms", 20.0));
    mcfg.follow.pursuit_speed_ms = static_cast<float>(config_->getDouble("mission.follow.pursuit_speed_ms", 30.0));
    mcfg.follow.pn_gain = config_->getDouble("mission.follow.pn_gain", 3.0);
    mcfg.follow.pn_max_lead_deg = config_->getDouble("mission.follow.pn_max_lead_deg", 45.0);

    mcfg.qr_dive.approach_altitude_m = config_->getDouble("mission.qr_dive.approach_altitude_m", 120.0);
    mcfg.qr_dive.min_altitude_m = config_->getDouble("mission.qr_dive.min_altitude_m", 20.0);
    mcfg.qr_dive.climb_altitude_m = config_->getDouble("mission.qr_dive.climb_altitude_m", 100.0);
    mcfg.qr_dive.entry_corridor_m = config_->getDouble("mission.qr_dive.entry_corridor_m", 500.0);
    mcfg.qr_dive.align_heading_tolerance_deg = config_->getDouble("mission.qr_dive.align_heading_tolerance_deg", 15.0);
    mcfg.qr_dive.align_altitude_tolerance_m = config_->getDouble("mission.qr_dive.align_altitude_tolerance_m", 10.0);
    mcfg.qr_dive.dive_pitch_deg = config_->getDouble("mission.qr_dive.dive_pitch_deg", -45.0);
    mcfg.qr_dive.dive_speed_ms = config_->getDouble("mission.qr_dive.dive_speed_ms", 25.0);
    mcfg.qr_dive.target_pass_distance_m = config_->getDouble("mission.qr_dive.target_pass_distance_m", 15.0);
    mcfg.qr_dive.climb_timeout_s = config_->getDouble("mission.qr_dive.climb_timeout_s", 30.0);
    mcfg.qr_dive.use_position_dive = config_->getBool("mission.qr_dive.use_position_dive", true);
    mcfg.qr_dive.dive_step_interval_s = config_->getDouble("mission.qr_dive.dive_step_interval_s", 0.5);
    mcfg.qr_dive.use_perception_terminal = config_->getBool("mission.qr_dive.use_perception_terminal", true);
    mcfg.qr_dive.perception_lock_age_s = config_->getDouble("mission.qr_dive.perception_lock_age_s", 0.5);
    mcfg.qr_dive.perception_module_id = config_->getString("mission.qr_dive.perception_module", "qr_simple");

    mcfg.qr_dive_on_complete = mission::parsePostDiveAction(
        config_->getString("mission.qr_dive.on_complete", "rtl"),
        mission::PostDiveAction::RTL);
    mcfg.qr_dive_perception_module = mcfg.qr_dive.perception_module_id;

    mcfg.follow_vision_fallback = config_->getBool("mission.follow.vision_fallback", false);
    mcfg.follow_vision_module = config_->getString("mission.follow.vision_module", "vision_tracker");
    mcfg.follow.vision_fallback = mcfg.follow_vision_fallback;
    mcfg.follow.vision_module_id = mcfg.follow_vision_module;
    mcfg.follow.vision_lock_age_s = config_->getDouble("mission.follow.vision_lock_age_s", 0.5);

    mission_ = std::make_shared<mission::MissionDirector>(
        nav_engine_, commander_, world_model_, event_bus_, perception_, mcfg);
    mission_->initialize();

    LOG_INFO("Application: mission initialized");
    return true;
}

void Application::subscribeEvents() {
    event_bus_->subscribe(infra::EventType::GPS_QUALITY_CRITICAL,
        [this](const infra::Event& e) { handleFailsafe(e); });

    event_bus_->subscribe(infra::EventType::BATTERY_CRITICAL,
        [this](const infra::Event& e) { handleFailsafe(e); });

    event_bus_->subscribe(infra::EventType::HEARTBEAT_TIMEOUT,
        [this](const infra::Event& e) { handleFailsafe(e); });

    event_bus_->subscribe(infra::EventType::HEARTBEAT_RECOVERED,
        [this](const infra::Event&) {
            safety_monitor_->notifyHeartbeat();
        });

    event_bus_->subscribe(infra::EventType::HEARTBEAT_RECEIVED,
        [this](const infra::Event&) {
            safety_monitor_->notifyHeartbeat();
        });

    event_bus_->subscribe(infra::EventType::PARAMS_READY,
        [this](const infra::Event&) {
            applyArduPilotParams();
        });

    event_bus_->subscribe(infra::EventType::HSS_ZONE_UPDATED,
        [this](const infra::Event&) { nav_engine_->revalidateRoute(); });

    event_bus_->subscribe(infra::EventType::BOUNDARY_UPDATED,
        [this](const infra::Event&) {
            // Overlay configured altitude limits onto any boundary delivered
            // via MAVLink/STATUSTEXT so we never run with parser defaults.
            auto b = world_model_->getBoundary();
            if (b.has_value()) {
                b->min_altitude = config_->getDouble("safety.min_altitude_m", b->min_altitude);
                b->max_altitude = config_->getDouble("safety.max_altitude_m", b->max_altitude);
                world_model_->setBoundary(*b);
            }
            nav_engine_->revalidateRoute();
        });

    event_bus_->subscribe(infra::EventType::MODE_COMMAND_RECEIVED,
        [this](const infra::Event& e) {
            if (e.hasPayload<comm::ParsedModeCommand>()) {
                mission_->handleModeCommand(e.getPayload<comm::ParsedModeCommand>());
            }
        });

    event_bus_->subscribe(infra::EventType::SAFETY_VIOLATION,
        [this](const infra::Event& e) {
            if (e.hasPayload<core::SafetyState>()) {
                auto safety = e.getPayload<core::SafetyState>();
                handleEscapeNeeded(safety);
            }
        });

    event_bus_->subscribe(infra::EventType::COLLISION_WARNING,
        [this](const infra::Event& e) {
            if (e.hasPayload<core::Threat>()) {
                auto threat = e.getPayload<core::Threat>();
                if (threat.risk_level >= core::RiskLevel::CRITICAL) {
                    auto own = world_model_->getOwnState();
                    auto escape = escape_manager_->generateCollisionEscape(own, threat);
                    if (escape.has_value()) {
                        nav_engine_->flyEscapeDirect(*escape);
                    }
                }
            }
        });
}

void Application::loadBoundaryFromConfig() {
    auto node = config_->getNode("boundary.vertices");
    if (!node || !node.IsSequence()) return;

    core::Boundary boundary;
    for (const auto& v : node) {
        // Use sentinel NaN to detect missing keys so that valid boundaries near
        // the equator/prime meridian (lat=0 or lon=0) are not silently dropped.
        double lat = v["lat"].as<double>(std::numeric_limits<double>::quiet_NaN());
        double lon = v["lon"].as<double>(std::numeric_limits<double>::quiet_NaN());
        if (std::isnan(lat) || std::isnan(lon)) continue;
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) continue;
        boundary.vertices.emplace_back(lat, lon, 0.0);
    }

    boundary.min_altitude = config_->getDouble("safety.min_altitude_m", 30.0);
    boundary.max_altitude = config_->getDouble("safety.max_altitude_m", 400.0);

    if (boundary.isValid()) {
        world_model_->setBoundary(boundary);
        LOG_INFO("Application: boundary loaded with {} vertices",
                 boundary.vertices.size());
    }
}

void Application::loadHSSFromConfig() {
    auto node = config_->getNode("hss_zones");
    if (!node || !node.IsSequence()) return;

    int count = 0;
    for (const auto& z : node) {
        core::HSSZone zone;
        zone.id = z["id"].as<int>(count);
        zone.center.latitude = z["lat"].as<double>(0.0);
        zone.center.longitude = z["lon"].as<double>(0.0);
        zone.center.altitude = z["alt"].as<double>(0.0);
        zone.radius = z["radius"].as<double>(0.0);
        zone.active = z["active"].as<bool>(true);
        zone.buffer_distance = config_->getDouble("safety.hss_buffer_m", 40.0);

        if (zone.radius > 0) {
            world_model_->updateHSSZone(zone);
            count++;
        }
    }

    if (count > 0) {
        LOG_INFO("Application: {} HSS zones loaded", count);
    }
}

// =============================================================================
// Failsafe & Escape
// =============================================================================

void Application::handleFailsafe(const infra::Event& event) {
    // Throttle each failsafe type independently so that, e.g., a battery event
    // is not silenced by a recent GPS event.
    static std::map<infra::EventType, std::chrono::steady_clock::time_point> last_by_type;
    static std::mutex throttle_mutex;
    {
        std::lock_guard lock(throttle_mutex);
        auto now = time::nowSteady();
        auto& last = last_by_type[event.type];
        if (time::durationS(last, now) < 10.0) return;
        last = now;
    }

    switch (event.type) {
        case infra::EventType::GPS_QUALITY_CRITICAL:
            LOG_WARN("Application: FAILSAFE - GPS critical");
            triggerRTL("gps_critical");
            break;

        case infra::EventType::BATTERY_CRITICAL:
            LOG_WARN("Application: FAILSAFE - Battery critical");
            triggerRTL("battery_critical");
            break;

        case infra::EventType::HEARTBEAT_TIMEOUT:
            LOG_WARN("Application: FAILSAFE - Heartbeat timeout");
            safety_monitor_->notifyHeartbeatLost();
            triggerRTL("heartbeat_timeout");
            break;

        default:
            break;
    }
}

void Application::handleEscapeNeeded(const core::SafetyState& safety) {
    if (mission_->getCurrentMode() == core::MissionMode::ARDUPILOT) return;

    auto own = world_model_->getOwnState();
    if (!own.isValid()) return;

    if (safety.boundary_status == core::BoundaryStatus::OUTSIDE) {
        auto boundary = world_model_->getBoundary();
        if (boundary.has_value()) {
            auto escape = escape_manager_->generateBoundaryEscape(own, *boundary);
            if (escape.has_value()) {
                LOG_WARN("Application: executing boundary escape");
                nav_engine_->flyEscapeDirect(*escape);
                return;
            }
        }
    }

    if (!safety.hss_violations.empty()) {
        auto zones = world_model_->getActiveHSSZones();
        auto escape = escape_manager_->generateHSSEscape(own, zones);
        if (escape.has_value()) {
            LOG_WARN("Application: executing HSS escape");
            nav_engine_->flyEscapeDirect(*escape);
            return;
        }
    }

    if (safety.collision_risk >= core::RiskLevel::CRITICAL && !safety.active_threats.empty()) {
        auto escape = escape_manager_->generateCollisionEscape(
            own, safety.active_threats.front());
        if (escape.has_value()) {
            LOG_WARN("Application: executing collision escape");
            nav_engine_->flyEscapeDirect(*escape);
        }
    }
}

void Application::triggerRTL(const std::string& reason) {
    mission_->triggerRTL(reason);
}

void Application::applyArduPilotParams() {
    auto pr = mavlink_->getParamReader();
    if (!pr || !pr->areCriticalReady()) return;

    LOG_INFO("Application: applying ArduPilot parameters to subsystems");

    // --- SpeedManager ---
    auto speed_cfg = speed_manager_->getConfig();
    float old_cruise = speed_cfg.cruise;
    float old_min = speed_cfg.min_speed;
    float old_max = speed_cfg.max_speed;

    speed_cfg.min_speed = pr->getStallSpeed();
    speed_cfg.max_speed = pr->getMaxAirspeed();
    speed_cfg.cruise = pr->getCruiseSpeed();

    speed_cfg.patrol = std::clamp(speed_cfg.patrol, speed_cfg.min_speed, speed_cfg.max_speed);
    speed_cfg.pursuit = std::clamp(speed_cfg.pursuit, speed_cfg.min_speed, speed_cfg.max_speed);
    speed_cfg.dive = std::clamp(speed_cfg.dive, speed_cfg.min_speed, speed_cfg.max_speed);

    speed_manager_->setConfig(speed_cfg);
    LOG_INFO("  SpeedManager: cruise {:.1f}->{:.1f}  min {:.1f}->{:.1f}  max {:.1f}->{:.1f}",
             old_cruise, speed_cfg.cruise, old_min, speed_cfg.min_speed, old_max, speed_cfg.max_speed);

    // --- PathPlanner: turn radius ---
    auto plan_cfg = planner_->getConfig();
    float old_turn_r = static_cast<float>(plan_cfg.min_turn_radius_m);
    float estimated_r = pr->estimateTurnRadius();
    float safety_factor = 1.2f;
    plan_cfg.min_turn_radius_m = static_cast<double>(estimated_r * safety_factor);
    plan_cfg.turn_safety_margin_m = std::max(
        plan_cfg.turn_safety_margin_m, plan_cfg.min_turn_radius_m);
    planner_->setConfig(plan_cfg);
    LOG_INFO("  PathPlanner: turn radius {:.0f}m -> {:.0f}m (estimated {:.0f}m * {:.1f}), corridor margin {:.0f}m",
             old_turn_r, plan_cfg.min_turn_radius_m, estimated_r, safety_factor,
             plan_cfg.turn_safety_margin_m);

    // --- SafetyGate: altitude limits + max velocity ---
    auto gate_cfg = safety_gate_->getConfig();
    float old_max_alt = static_cast<float>(gate_cfg.max_altitude_m);
    float old_min_alt = static_cast<float>(gate_cfg.min_altitude_m);
    float old_max_vel = static_cast<float>(gate_cfg.max_velocity_ms);

    if (pr->hasParam("FENCE_ALT_MAX") && pr->getFenceAltMax() > 10.0f) {
        gate_cfg.max_altitude_m = static_cast<double>(pr->getFenceAltMax());
    }
    if (pr->hasParam("FENCE_ALT_MIN") && pr->getFenceAltMin() > 0.0f) {
        gate_cfg.min_altitude_m = static_cast<double>(pr->getFenceAltMin());
    }
    gate_cfg.max_velocity_ms = static_cast<double>(pr->getMaxAirspeed() * 1.1f);

    safety_gate_->setConfig(gate_cfg);
    LOG_INFO("  SafetyGate: alt [{:.0f},{:.0f}]->[{:.0f},{:.0f}]  maxVel {:.0f}->{:.0f}",
             old_min_alt, old_max_alt,
             gate_cfg.min_altitude_m, gate_cfg.max_altitude_m,
             old_max_vel, gate_cfg.max_velocity_ms);

    // --- Waypoint acceptance radius ---
    float wp_radius = pr->getWPRadius();
    LOG_INFO("  WP_RADIUS={:.0f}m (AP) | loiter_rad={:.0f}m",
             wp_radius, pr->getLoiterRadius());

    LOG_INFO("Application: ArduPilot parameter sync complete");
}

// =============================================================================
// Main Loop
// =============================================================================

void Application::mainLoop() {
    constexpr int LOOP_HZ = 5;
    constexpr auto LOOP_PERIOD = std::chrono::milliseconds(1000 / LOOP_HZ);
    auto next_tick = std::chrono::steady_clock::now();
    auto last_dashboard = std::chrono::steady_clock::now();
    auto last_cleanup = std::chrono::steady_clock::now();

    const double stale_timeout_s = config_->getDouble("system.opponent_stale_timeout_s", 10.0);

    while (running_ && !shutdown_requested_) {
        mission_->tick();

#ifdef UAV_HAS_OPENCV
        // OpenCV HighGUI requires imshow / waitKey to run on the main
        // thread; the vision tracker stages frames from its worker and we
        // pump them out here so the preview window actually appears under
        // Windows.
        if (opencv_vision_tracker_ && opencv_vision_tracker_->previewEnabled()) {
            opencv_vision_tracker_->tickPreviewMainThread();
        }
#endif

        auto now = std::chrono::steady_clock::now();

        if (now - last_cleanup >= std::chrono::seconds(2)) {
            world_model_->removeStaleOpponents(stale_timeout_s);
            last_cleanup = now;
        }

        if (now - last_dashboard >= std::chrono::seconds(2)) {
            printDashboard();
            last_dashboard = now;
        }

        next_tick += LOOP_PERIOD;
        std::this_thread::sleep_until(next_tick);
    }
}

void Application::printDashboard() const {
    auto own = world_model_->getOwnState();
    auto mstats = mission_->getStats();
    auto safety = safety_monitor_->getSafetyState();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "\n=== UAV v10 Dashboard ===\n";
    ss << "Mode: " << core::toString(mstats.current_mode);
    if (!mstats.active_phase.empty()) {
        ss << " [" << mstats.active_phase << "]";
    }
    ss << " | Transitions: " << mstats.mode_transitions << "\n";

    if (own.isValid()) {
        ss << "Pos: " << std::setprecision(6) << own.position.latitude
           << ", " << own.position.longitude
           << " | Alt: " << std::setprecision(1) << own.position.altitude << "m"
           << " | Spd: " << own.airspeed << "m/s"
           << " | Hdg: " << own.heading << "\n";
        ss << "GPS: fix=" << static_cast<int>(own.gps_fix_type)
           << " sat=" << static_cast<int>(own.satellites_visible)
           << " | Bat: " << static_cast<int>(own.battery_remaining) << "%\n";
    } else {
        ss << "No telemetry\n";
    }

    ss << "Safety: " << (safety.isSafe() ? "OK" : "WARNING")
       << " | Collision: " << core::toString(safety.collision_risk)
       << " | Boundary: " << core::toString(safety.boundary_status)
       << " | CommLoss: " << (safety.comm_loss ? "YES" : "NO") << "\n";

    auto opponents = world_model_->getAllOpponents();
    auto hss_zones = world_model_->getActiveHSSZones();
    ss << "World: Opponents=" << opponents.size()
       << " | HSS=" << hss_zones.size()
       << " | Boundary=" << (world_model_->hasBoundary() ? "YES" : "NO") << "\n";

    for (const auto& opp : opponents) {
        ss << "  OPP[" << opp.id << "] "
           << std::setprecision(6) << opp.position.latitude
           << "," << opp.position.longitude
           << std::setprecision(1)
           << " alt=" << opp.position.altitude
           << " spd=" << opp.groundspeed
           << " hdg=" << opp.heading
           << " age=" << opp.ageSeconds() << "s\n";
    }

    auto mavstats = mavlink_->getStats();
    ss << "MAVLink: PIX=" << (mavstats.pixhawk_connected ? "OK" : "LOST")
       << " GCS=" << (mavstats.gcs_connected ? "OK" : "LOST")
       << " | Mode=" << mavstats.flight_mode
       << " | Armed=" << (mavstats.armed ? "YES" : "NO")
       << " | RX=" << mavstats.messages_received
       << " TX=" << mavstats.messages_sent << "\n";

    auto param_reader = mavlink_->getParamReader();
    if (param_reader && param_reader->totalCount() > 0) {
        ss << "Params: " << param_reader->receivedCount()
           << "/" << param_reader->totalCount()
           << " | TurnR=" << std::setprecision(0)
           << param_reader->estimateTurnRadius() << "m\n";
    }

    std::cout << ss.str() << std::flush;
}

// =============================================================================
// Signal handling
// =============================================================================

void Application::setupSignalHandlers() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
}

void Application::signalHandler(int sig) {
    if (instance_) {
        instance_->requestShutdown();
    }
    (void)sig;
}

} // namespace uav::app
