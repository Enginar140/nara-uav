#include "uav/comm/flight_commander.hpp"
#include "uav/comm/connection_manager.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace {

// ArduPilot extension commands (ardupilotmega.xml); not declared in MAVLink "common" dialect.
constexpr uint16_t kMavCmdGuidedChangeSpeed = 43000;
constexpr uint16_t kMavCmdGuidedChangeAltitude = 43001;
constexpr uint16_t kMavCmdGuidedChangeHeading = 43002;

// HEADING_TYPE values mirror MAVLink (ardupilotmega.xml): COG=0, HEADING=1, NONE/DEFAULT=2.
float mavlinkHeadingTypeParam(uav::comm::HeadingType type) {
    return static_cast<float>(static_cast<uint8_t>(type));
}

} // namespace

namespace uav::comm {

FlightCommander::FlightCommander(std::shared_ptr<ConnectionManager> conn,
                                 uint8_t system_id, uint8_t component_id,
                                 Config config)
    : conn_(std::move(conn)), sys_id_(system_id), comp_id_(component_id),
      config_(config) {}

bool FlightCommander::flyToPosition(const core::Position& target, float ground_speed_ms) {
    std::lock_guard lock(cmd_mutex_);
    updateAckTimeoutLocked();

    if (isPositionDuplicate(target)) return true;

    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_DO_REPOSITION,
        0, 0,
        ground_speed_ms,  // param1: ground speed in m/s (ArduPilot extension; -1 keeps default)
        0, 0, 0,
        static_cast<int32_t>(target.latitude * 1e7),
        static_cast<int32_t>(target.longitude * 1e7),
        static_cast<float>(target.altitude)
    );

    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        last_position_cmd_ = target;
        has_last_position_cmd_ = true;
        commands_sent_++;
        markAckPending(MAV_CMD_DO_REPOSITION);
    }
    return sent;
}

bool FlightCommander::flyToLocalPosition(float north_m, float east_m, float down_m) {
    std::lock_guard lock(cmd_mutex_);

    mavlink_message_t msg{};
    uint16_t type_mask = 0b0000111111111000;

    mavlink_msg_set_position_target_local_ned_pack(
        sys_id_, comp_id_, &msg,
        0,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_LOCAL_NED,
        type_mask,
        north_m, east_m, down_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    );

    bool sent = sendMAVLinkMessage(msg);
    if (sent) commands_sent_++;
    return sent;
}

bool FlightCommander::setLocalVelocity(float vn, float ve, float vd, float yaw_rate) {
    std::lock_guard lock(cmd_mutex_);

    mavlink_message_t msg{};
    // bits: pos_xyz=ignore, vel_xyz=use, acc_xyz=ignore, force=ignore, yaw=ignore
    uint16_t type_mask = 0b0000111111000111;
    if (std::abs(yaw_rate) > 0.001f) {
        type_mask &= ~(1 << 11);  // enable yaw_rate
    }

    mavlink_msg_set_position_target_local_ned_pack(
        sys_id_, comp_id_, &msg,
        0,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,
        vn, ve, vd,
        0, 0, 0,
        0, yaw_rate
    );

    bool sent = sendMAVLinkMessage(msg);
    if (sent) commands_sent_++;
    return sent;
}

// =====================================================================
// Local control (ArduPlane fork: feature/local-control)
// All four wrappers below funnel into sendLocalNedTarget(); they differ
// only in how the offset is described to the caller.
// =====================================================================

// Internal transport. Marshals a SET_POSITION_TARGET_LOCAL_NED that
// requests a position offset, masks off velocity/accel/yaw_rate, and
// optionally includes a yaw target.
bool FlightCommander::sendLocalNedTarget(uint8_t frame, float x, float y, float z, float yaw_deg) {
    std::lock_guard lock(cmd_mutex_);

    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::sendLocalNedTarget: Not in GUIDED mode");
        return false;
    }

    // POSITION_TARGET_TYPEMASK bits: 1=ignore.
    //  bit 0..2  : x, y, z
    //  bit 3..5  : vx, vy, vz
    //  bit 6..8  : ax, ay, az
    //  bit 9     : force (use accel as force)
    //  bit 10    : yaw
    //  bit 11    : yaw_rate
    // We honour position; ignore velocity, accel, force, yaw_rate.
    uint16_t type_mask = 0;
    type_mask |= (1u << 3) | (1u << 4) | (1u << 5);   // vx, vy, vz ignore
    type_mask |= (1u << 6) | (1u << 7) | (1u << 8);   // ax, ay, az ignore
    type_mask |= (1u << 9);                           // force flag (treat as ignore)
    type_mask |= (1u << 11);                          // yaw_rate ignore

    const bool yaw_valid = std::isfinite(yaw_deg);
    if (!yaw_valid) {
        type_mask |= (1u << 10);  // yaw ignore
    }
    const float yaw_rad = yaw_valid ? (yaw_deg * static_cast<float>(M_PI) / 180.0f) : 0.0f;

    mavlink_message_t msg{};
    mavlink_msg_set_position_target_local_ned_pack(
        sys_id_, comp_id_, &msg,
        0,                                 // time_boot_ms (firmware ignores)
        config_.target_sys, config_.target_comp,
        frame,
        type_mask,
        x, y, z,                           // position offset
        0.0f, 0.0f, 0.0f,                  // velocity (masked off)
        0.0f, 0.0f, 0.0f,                  // accel (masked off)
        yaw_rad, 0.0f                      // yaw (rad), yaw_rate (masked off)
    );

    bool sent = sendMAVLinkMessage(msg);
    if (sent) commands_sent_++;
    return sent;
}

bool FlightCommander::flyToBodyOffset(float forward_m, float right_m, float down_m, float yaw_deg) {
    LOG_INFO("FlightCommander::flyToBodyOffset: fwd={:.1f} right={:.1f} down={:.1f} yaw={:.1f}",
             forward_m, right_m, down_m, yaw_deg);
    return sendLocalNedTarget(MAV_FRAME_BODY_OFFSET_NED, forward_m, right_m, down_m, yaw_deg);
}

bool FlightCommander::flyToLocalOffset(float north_m, float east_m, float down_m, float yaw_deg) {
    LOG_INFO("FlightCommander::flyToLocalOffset: N={:.1f} E={:.1f} D={:.1f} yaw={:.1f}",
             north_m, east_m, down_m, yaw_deg);
    return sendLocalNedTarget(MAV_FRAME_LOCAL_OFFSET_NED, north_m, east_m, down_m, yaw_deg);
}

bool FlightCommander::flyToBearingOffset(float bearing_deg, float distance_m,
                                          float altitude_offset_m, float yaw_deg) {
    const float br = bearing_deg * static_cast<float>(M_PI) / 180.0f;
    const float north = std::cos(br) * distance_m;
    const float east  = std::sin(br) * distance_m;
    const float down  = -altitude_offset_m;  // positive alt offset = climb = down negative
    LOG_INFO("FlightCommander::flyToBearingOffset: bearing={:.1f}° dist={:.1f} alt_off={:.1f} -> N={:.1f} E={:.1f} D={:.1f}",
             bearing_deg, distance_m, altitude_offset_m, north, east, down);
    return sendLocalNedTarget(MAV_FRAME_LOCAL_OFFSET_NED, north, east, down, yaw_deg);
}

bool FlightCommander::flyToVisionTarget(float yaw_offset_deg, float pitch_offset_deg,
                                         float range_m, float yaw_deg) {
    // Camera-frame angles -> body-frame offset.
    //   yaw_offset_deg : positive = target to the right
    //   pitch_offset_deg : positive = target below the bore-sight (pitch down)
    //   range_m : estimated distance to the target
    const float yo = yaw_offset_deg   * static_cast<float>(M_PI) / 180.0f;
    const float po = pitch_offset_deg * static_cast<float>(M_PI) / 180.0f;
    const float fwd   = std::cos(po) * std::cos(yo) * range_m;
    const float right = std::cos(po) * std::sin(yo) * range_m;
    const float down  = std::sin(po) * range_m;
    LOG_INFO("FlightCommander::flyToVisionTarget: yaw_off={:.1f}° pitch_off={:.1f}° range={:.1f} -> fwd={:.1f} right={:.1f} down={:.1f}",
             yaw_offset_deg, pitch_offset_deg, range_m, fwd, right, down);
    return sendLocalNedTarget(MAV_FRAME_BODY_OFFSET_NED, fwd, right, down, yaw_deg);
}

// MAV_CMD_GUIDED_CHANGE_HEADING (43002) — param order per ardupilotmega.xml
bool FlightCommander::changeHeading(float heading_deg, HeadingType type,
                                     float accel_limit_rad_s2) {
    std::lock_guard lock(cmd_mutex_);

    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeHeading: Not in GUIDED mode");
        return false;
    }

    while (heading_deg < 0.0f) heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        kMavCmdGuidedChangeHeading,
        0,
        mavlinkHeadingTypeParam(type), // param1: HEADING_TYPE
        heading_deg,                    // param2: target heading (deg)
        accel_limit_rad_s2,             // param3: max centripetal accel (rad/s² spec says m/s/s in xml — AP uses this slot)
        0, 0, 0, 0);

    LOG_INFO("FlightCommander::changeHeading: heading={:.1f}° mavlink_type={:.0f} accel={:.1f}",
             heading_deg, mavlinkHeadingTypeParam(type), accel_limit_rad_s2);

    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(kMavCmdGuidedChangeHeading);
    }
    return sent;
}

// MAV_CMD_GUIDED_CHANGE_SPEED (43000) — param1 SPEED_TYPE, param2 speed (m/s), param3 accel (m/s²)
bool FlightCommander::changeAirspeed(float speed_ms, float acceleration_ms2, bool use_airspeed) {
    std::lock_guard lock(cmd_mutex_);

    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeAirspeed: Not in GUIDED mode");
        return false;
    }

    if (speed_ms < 5.0f || speed_ms > 100.0f) {
        LOG_WARN("FlightCommander::changeAirspeed: Invalid speed {:.1f} m/s", speed_ms);
        return false;
    }

    const float speed_type = use_airspeed ? 0.0f : 1.0f; // SPEED_TYPE_AIRSPEED / GROUNDSPEED

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        kMavCmdGuidedChangeSpeed,
        0,
        speed_type,
        speed_ms,
        acceleration_ms2,
        0, 0, 0, 0);

    LOG_INFO("FlightCommander::changeAirspeed: speed={:.1f} m/s airspeed={} accel={:.1f} m/s²",
             speed_ms, use_airspeed ? "yes" : "no", acceleration_ms2);

    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(kMavCmdGuidedChangeSpeed);
    }
    return sent;
}

// MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) — rate param3, altitude param7
bool FlightCommander::changeAltitude(float altitude_m, float climb_rate_ms) {
    std::lock_guard lock(cmd_mutex_);

    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeAltitude: Not in GUIDED mode");
        return false;
    }

    if (altitude_m < 0.0f || altitude_m > 1000.0f) {
        LOG_WARN("FlightCommander::changeAltitude: Invalid altitude {:.1f} m", altitude_m);
        return false;
    }

    climb_rate_ms = std::clamp(climb_rate_ms, 0.5f, 15.0f);

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        kMavCmdGuidedChangeAltitude,
        0,
        0,
        0,
        climb_rate_ms,
        0,
        0,
        0,
        altitude_m);

    LOG_INFO("FlightCommander::changeAltitude: alt={:.1f} m rate={:.1f} m/s",
             altitude_m, climb_rate_ms);

    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(kMavCmdGuidedChangeAltitude);
    }
    return sent;
}

bool FlightCommander::setMode(ArduPilotMode mode) {
    std::lock_guard lock(cmd_mutex_);

    mavlink_message_t msg{};
    mavlink_command_long_t cmd{};

    cmd.target_system = config_.target_sys;
    cmd.target_component = config_.target_comp;
    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = 0;
    cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    cmd.param2 = static_cast<float>(static_cast<uint32_t>(mode));

    mavlink_msg_command_long_encode(sys_id_, comp_id_, &msg, &cmd);

    LOG_INFO("FlightCommander: setMode({}) param1={} param2={} (current={})",
             toString(mode), cmd.param1, cmd.param2, toString(current_mode_.load()));

    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(MAV_CMD_DO_SET_MODE);
    }
    return sent;
}

bool FlightCommander::arm() {
    std::lock_guard lock(cmd_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0f, 0, 0, 0, 0, 0, 0
    );

    LOG_INFO("FlightCommander: arming");
    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(MAV_CMD_COMPONENT_ARM_DISARM);
    }
    return sent;
}

bool FlightCommander::disarm() {
    std::lock_guard lock(cmd_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0f, 0, 0, 0, 0, 0, 0
    );

    LOG_INFO("FlightCommander: disarming");
    bool sent = sendMAVLinkMessage(msg);
    if (sent) {
        commands_sent_++;
        markAckPending(MAV_CMD_COMPONENT_ARM_DISARM);
    }
    return sent;
}

void FlightCommander::updateModeFromHeartbeat(uint32_t custom_mode) {
    auto mode = static_cast<ArduPilotMode>(custom_mode);
    const char* name = toString(mode);
    if (std::string(name) == "UNKNOWN") {
        static uint32_t last_unknown = UINT32_MAX;
        if (custom_mode != last_unknown) {
            LOG_WARN("FlightCommander: unknown custom_mode={}", custom_mode);
            last_unknown = custom_mode;
        }
    }
    current_mode_ = mode;
}

void FlightCommander::updateArmedFromHeartbeat(uint8_t base_mode) {
    armed_ = (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

AckStatus FlightCommander::getLastAckStatus() const {
    std::lock_guard lock(cmd_mutex_);
    updateAckTimeoutLocked();
    return last_ack_status_.load();
}

void FlightCommander::handleCommandAck(uint16_t cmd_id, uint8_t result) {
    AckCallback cb;
    {
        std::lock_guard lock(cmd_mutex_);
        updateAckTimeoutLocked();

        uint16_t pending = pending_cmd_id_.load();
        if (pending != 0 && cmd_id != pending) {
            LOG_DEBUG("FlightCommander: ignoring ACK for CMD {} while waiting for CMD {}",
                      cmd_id, pending);
            return;
        }

        commands_acked_++;
        last_ack_time_ = std::chrono::steady_clock::now();

        if (result == MAV_RESULT_ACCEPTED) {
            last_ack_status_ = AckStatus::ACCEPTED;
            pending_cmd_id_ = 0;
            LOG_DEBUG("FlightCommander: CMD {} accepted", cmd_id);
        } else if (result == MAV_RESULT_IN_PROGRESS) {
            last_ack_status_ = AckStatus::WAITING;
            LOG_DEBUG("FlightCommander: CMD {} in progress", cmd_id);
        } else {
            last_ack_status_ = AckStatus::REJECTED;
            pending_cmd_id_ = 0;
            if (cmd_id == MAV_CMD_DO_REPOSITION) {
                has_last_position_cmd_ = false;
            }
            LOG_WARN("FlightCommander: CMD {} rejected (result={})", cmd_id, result);
        }

        cb = ack_callback_;
    }

    if (cb) {
        cb(cmd_id, result);
    }
}

void FlightCommander::markAckPending(uint16_t cmd_id) {
    pending_cmd_id_ = cmd_id;
    pending_since_ = std::chrono::steady_clock::now();
    last_ack_status_ = AckStatus::WAITING;
}

void FlightCommander::updateAckTimeoutLocked() const {
    if (last_ack_status_.load() != AckStatus::WAITING) return;
    uint16_t pending = pending_cmd_id_.load();
    if (pending == 0) return;

    auto elapsed = std::chrono::steady_clock::now() - pending_since_;
    if (elapsed >= std::chrono::milliseconds(config_.ack_timeout_ms)) {
        last_ack_status_ = AckStatus::TIMEOUT;
        if (pending == MAV_CMD_DO_REPOSITION) {
            has_last_position_cmd_ = false;
        }
        pending_cmd_id_ = 0;
        LOG_WARN("FlightCommander: CMD {} ACK timeout after {}ms",
                 pending, config_.ack_timeout_ms);
    }
}

bool FlightCommander::sendMAVLinkMessage(mavlink_message_t& msg) {
    if (!conn_ || !conn_->isConnected()) return false;

    MAVLinkMessage wrapped;
    wrapped.raw = msg;
    return conn_->sendMessage(wrapped);
}

bool FlightCommander::isPositionDuplicate(const core::Position& target) const {
    if (!has_last_position_cmd_) return false;
    double dist = geo::haversineDistance(last_position_cmd_, target);
    return dist < config_.min_command_delta_m;
}

const char* toString(ArduPilotMode mode) {
    switch (mode) {
        case ArduPilotMode::MANUAL:       return "MANUAL";
        case ArduPilotMode::CIRCLE:       return "CIRCLE";
        case ArduPilotMode::STABILIZE:    return "STABILIZE";
        case ArduPilotMode::TRAINING:     return "TRAINING";
        case ArduPilotMode::ACRO:         return "ACRO";
        case ArduPilotMode::FBWA:         return "FBWA";
        case ArduPilotMode::FBWB:         return "FBWB";
        case ArduPilotMode::CRUISE:       return "CRUISE";
        case ArduPilotMode::AUTOTUNE:     return "AUTOTUNE";
        case ArduPilotMode::AUTO:         return "AUTO";
        case ArduPilotMode::RTL:          return "RTL";
        case ArduPilotMode::LOITER:       return "LOITER";
        case ArduPilotMode::TAKEOFF:      return "TAKEOFF";
        case ArduPilotMode::AVOID_ADSB:   return "AVOID_ADSB";
        case ArduPilotMode::GUIDED:       return "GUIDED";
        case ArduPilotMode::INITIALISING: return "INITIALISING";
        case ArduPilotMode::QSTABILIZE:   return "QSTABILIZE";
        case ArduPilotMode::QHOVER:       return "QHOVER";
        case ArduPilotMode::QLOITER:      return "QLOITER";
        case ArduPilotMode::QLAND:        return "QLAND";
        case ArduPilotMode::QRTL:         return "QRTL";
        case ArduPilotMode::QAUTOTUNE:    return "QAUTOTUNE";
        case ArduPilotMode::QACRO:        return "QACRO";
        case ArduPilotMode::THERMAL:      return "THERMAL";
    }
    return "UNKNOWN";
}

} // namespace uav::comm
