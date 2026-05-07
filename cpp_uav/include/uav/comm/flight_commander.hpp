#pragma once

#include "uav/core/types.hpp"
#include "uav/comm/mavlink_message.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>

namespace uav::comm {

class ConnectionManager;

enum class ArduPilotMode : uint32_t {
    MANUAL      = 0,
    CIRCLE      = 1,
    STABILIZE   = 2,
    TRAINING    = 3,
    ACRO        = 4,
    FBWA        = 5,
    FBWB        = 6,
    CRUISE      = 7,
    AUTOTUNE    = 8,
    AUTO        = 10,
    RTL         = 11,
    LOITER      = 12,
    TAKEOFF     = 13,
    AVOID_ADSB  = 14,
    GUIDED      = 15,
    INITIALISING = 16,
    QSTABILIZE  = 17,
    QHOVER      = 18,
    QLOITER     = 19,
    QLAND       = 20,
    QRTL        = 21,
    QAUTOTUNE   = 22,
    QACRO       = 23,
    THERMAL     = 24
};

// HEADING_TYPE for MAV_CMD_GUIDED_CHANGE_HEADING (ardupilotmega.xml).
// Values match the MAVLink enum so the underlying integer is sent verbatim.
enum class HeadingType : uint8_t {
    COG     = 0,  // Course Over Ground
    HEADING = 1,  // Absolute heading
    NONE    = 2   // DEFAULT / no heading control
};

enum class AckStatus { NONE, WAITING, ACCEPTED, REJECTED, TIMEOUT };

class FlightCommander {
public:
    struct Config {
        uint8_t target_sys = 1;
        uint8_t target_comp = 1;
        int max_retries = 3;
        int ack_timeout_ms = 2000;
        double min_command_delta_m = 5.0;
    };

    FlightCommander(std::shared_ptr<ConnectionManager> conn,
                    uint8_t system_id = 1,
                    uint8_t component_id = MAV_COMP_ID_ONBOARD_COMPUTER,
                    Config config = {});

    /// MAV_CMD_DO_REPOSITION. \p ground_speed_ms is the ArduPilot ground-speed
    /// hint (param1, m/s). Pass -1.0f to keep the autopilot's current default.
    [[nodiscard]] bool flyToPosition(const core::Position& target,
                                     float ground_speed_ms = -1.0f);

    /// Legacy: SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_LOCAL_NED.
    /// Stock ArduPlane silently discards this frame; only works with our
    /// feature/local-control fork (treats it as offset-from-current-loc).
    /// Prefer flyToBodyOffset() / flyToLocalOffset() / flyToBearingOffset() /
    /// flyToVisionTarget() below.
    [[nodiscard]] bool flyToLocalPosition(float north_m, float east_m, float down_m);
    /// Legacy velocity setpoint. Stock ArduPlane ignores this entirely; even
    /// our fork does not implement velocity control (see PATCHES.md). Kept
    /// only for binary compatibility with older callers.
    [[nodiscard]] bool setLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);

    // ---------------------------------------------------------------------
    // Local control (ArduPlane fork: feature/local-control)
    //
    // Set-and-forget position-offset commands. Companion may resend at low
    // rate (1-5 Hz); no continuous stream required. All four wrappers below
    // marshal to a single MAVLink SET_POSITION_TARGET_LOCAL_NED message --
    // the difference is purely in how the offset is described.
    //
    // Pass NaN for yaw_deg to leave heading uncommanded (the existing L1
    // controller will steer toward the synthetic waypoint naturally).
    // Vehicle must be in GUIDED mode; out-of-fence targets are rejected by
    // the firmware with a STATUSTEXT warning.
    // ---------------------------------------------------------------------

    /// Body-frame offset: x=forward, y=right, z=down (metres).
    /// Companion does not need to know the current heading -- firmware
    /// rotates the (x,y) into NED via AHRS yaw before generating the WP.
    [[nodiscard]] bool flyToBodyOffset(float forward_m, float right_m, float down_m,
                                       float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Local NED offset: x=north, y=east, z=down (metres) relative to the
    /// current location. Useful for map-frame planning.
    [[nodiscard]] bool flyToLocalOffset(float north_m, float east_m, float down_m,
                                        float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Polar offset: bearing in degrees from north, distance in metres,
    /// altitude_offset_m positive for climb. Internally converts to a
    /// LOCAL_OFFSET_NED message.
    [[nodiscard]] bool flyToBearingOffset(float bearing_deg, float distance_m,
                                          float altitude_offset_m,
                                          float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// Vision-LOS pursuit: target appears at \p yaw_offset_deg right and
    /// \p pitch_offset_deg down from the camera bore-sight; estimated
    /// \p range_m metres ahead. Maps to a BODY_OFFSET_NED message; pass a
    /// fixed range (e.g. 50 m) when no depth estimate is available -- the
    /// resulting pure-pursuit behaviour still works because the companion
    /// keeps refreshing the command as the target moves in frame.
    [[nodiscard]] bool flyToVisionTarget(float yaw_offset_deg, float pitch_offset_deg,
                                         float range_m,
                                         float yaw_deg = std::numeric_limits<float>::quiet_NaN());

    /// ArduPilot GUIDED slew: MAV_CMD_GUIDED_CHANGE_HEADING (43002), param layout per ardupilotmega.xml
    [[nodiscard]] bool changeHeading(float heading_deg, HeadingType type,
                                     float accel_limit_rad_s2 = 10.0f);
    /// MAV_CMD_GUIDED_CHANGE_SPEED (43000): airspeed vs groundspeed via \p use_airspeed
    [[nodiscard]] bool changeAirspeed(float speed_ms, float acceleration_ms2 = 2.0f,
                                      bool use_airspeed = true);
    /// MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001): target altitude in param7, rate in param3
    [[nodiscard]] bool changeAltitude(float altitude_m, float climb_rate_ms = 3.0f);

    [[nodiscard]] bool setMode(ArduPilotMode mode);
    [[nodiscard]] bool arm();
    [[nodiscard]] bool disarm();

    ArduPilotMode getCurrentMode() const { return current_mode_.load(); }
    bool isInGuidedMode() const { return current_mode_ == ArduPilotMode::GUIDED; }
    bool isArmed() const { return armed_.load(); }
    void updateModeFromHeartbeat(uint32_t custom_mode);
    void updateArmedFromHeartbeat(uint8_t base_mode);

    void handleCommandAck(uint16_t cmd_id, uint8_t result);

    using AckCallback = std::function<void(uint16_t cmd_id, uint8_t result)>;
    void setAckCallback(AckCallback cb) { ack_callback_ = std::move(cb); }

    uint64_t getCommandsSent() const { return commands_sent_.load(); }
    uint64_t getCommandsAcked() const { return commands_acked_.load(); }
    AckStatus getLastAckStatus() const;

private:
    std::shared_ptr<ConnectionManager> conn_;
    uint8_t sys_id_;
    uint8_t comp_id_;
    Config config_;

    std::atomic<ArduPilotMode> current_mode_{ArduPilotMode::MANUAL};
    std::atomic<bool> armed_{false};
    std::atomic<uint64_t> commands_sent_{0};
    std::atomic<uint64_t> commands_acked_{0};
    mutable std::atomic<AckStatus> last_ack_status_{AckStatus::NONE};
    mutable std::atomic<uint16_t> pending_cmd_id_{0};
    mutable std::chrono::steady_clock::time_point pending_since_{};
    std::chrono::steady_clock::time_point last_ack_time_;

    AckCallback ack_callback_;

    mutable std::recursive_mutex cmd_mutex_;
    core::Position last_position_cmd_;
    mutable bool has_last_position_cmd_ = false;

    bool sendMAVLinkMessage(mavlink_message_t& msg);
    bool isPositionDuplicate(const core::Position& target) const;
    void markAckPending(uint16_t cmd_id);
    void updateAckTimeoutLocked() const;

    /// Internal transport for SET_POSITION_TARGET_LOCAL_NED. \p frame is one
    /// of MAV_FRAME_BODY_OFFSET_NED / MAV_FRAME_LOCAL_OFFSET_NED. Position
    /// fields are populated; velocity/accel/yaw_rate are masked off. yaw is
    /// included only if \p yaw_deg is finite (not NaN).
    bool sendLocalNedTarget(uint8_t frame, float x, float y, float z, float yaw_deg);
};

const char* toString(ArduPilotMode mode);

} // namespace uav::comm
