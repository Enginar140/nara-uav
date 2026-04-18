#pragma once

#include "uav/mission/mission_mode.hpp"
#include "uav/nav/navigation_engine.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/world_model/world_model.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace uav::mission {

enum class DivePhase { APPROACH, ALIGN, DIVE, CLIMB, COMPLETED };

const char* toString(DivePhase phase);

struct QRDetectionResult {
    bool detected = false;
    std::string qr_data;
    double confidence = 0.0;
};

class QRDiveMode : public IMissionMode {
public:
    using QRCallback = std::function<QRDetectionResult()>;

    struct Config {
        double approach_altitude_m = 120.0;
        double min_altitude_m = 20.0;
        double climb_altitude_m = 100.0;
        double entry_corridor_m = 500.0;
        double align_heading_tolerance_deg = 15.0;
        double align_altitude_tolerance_m = 10.0;
        double dive_pitch_deg = -45.0;
        double dive_speed_ms = 25.0;
        double target_pass_distance_m = 15.0;
        double climb_timeout_s = 30.0;
        bool use_position_dive = true;
        double dive_step_interval_s = 0.5;
    };

    QRDiveMode(std::shared_ptr<nav::NavigationEngine> nav,
               std::shared_ptr<world_model::WorldModel> world_model,
               std::shared_ptr<comm::FlightCommander> commander,
               Config config = {});

    void setDiveTarget(const core::Position& target);
    void setQRCallback(QRCallback cb) { qr_callback_ = std::move(cb); }
    void setCompletionCallback(std::function<void()> cb) { completion_cb_ = std::move(cb); }

    DivePhase getPhase() const { return phase_; }
    bool isCompleted() const { return phase_ == DivePhase::COMPLETED; }
    core::Position getEntryPoint() const { return entry_point_; }

    void onEnter() override;
    void onExit() override;
    void tick(const core::UAVState& own_state) override;
    core::MissionGoal getMissionGoal() override;
    std::string getName() const override { return "QR_DIVE"; }

private:
    std::shared_ptr<nav::NavigationEngine> nav_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    std::shared_ptr<comm::FlightCommander> commander_;
    Config config_;
    QRCallback qr_callback_;
    std::function<void()> completion_cb_;

    std::optional<core::Position> dive_target_;
    core::Position entry_point_;
    bool entry_valid_ = false;
    DivePhase phase_ = DivePhase::APPROACH;
    std::chrono::steady_clock::time_point phase_start_;
    QRDetectionResult last_qr_;

    void calculateEntryPoint();
    void tickApproach(const core::UAVState& own);
    void tickAlign(const core::UAVState& own);
    void tickDive(const core::UAVState& own);
    void tickDivePosition(const core::UAVState& own);
    void tickDiveVelocity(const core::UAVState& own);
    void tickClimb(const core::UAVState& own);

    double distanceToTarget(const core::UAVState& own) const;
    double bearingToTarget(const core::UAVState& own) const;
};

} // namespace uav::mission
