#include "uav/comm/flight_commander.hpp"
#include "uav/comm/connection_manager.hpp"
#include "uav/core/geo.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>
#include <string>

namespace uav::comm {

FlightCommander::FlightCommander(std::shared_ptr<ConnectionManager> conn,
                                 uint8_t system_id, uint8_t component_id,
                                 Config config)
    : conn_(std::move(conn)), sys_id_(system_id), comp_id_(component_id),
      config_(config) {}

bool FlightCommander::flyToPosition(const core::Position& target, float airspeed) {
    std::lock_guard lock(cmd_mutex_);

    if (isPositionDuplicate(target)) return true;

    mavlink_message_t msg{};
    mavlink_msg_command_int_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_DO_REPOSITION,
        0, 0,
        airspeed,
        0, 0, 0,
        static_cast<int32_t>(target.latitude * 1e7),
        static_cast<int32_t>(target.longitude * 1e7),
        static_cast<float>(target.altitude)
    );

    last_position_cmd_ = target;
    pending_cmd_id_ = MAV_CMD_DO_REPOSITION;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
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

    commands_sent_++;
    return sendMAVLinkMessage(msg);
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

    commands_sent_++;
    return sendMAVLinkMessage(msg);
}

// MAV_CMD_GUIDED_CHANGE_HEADING (40002)
// Uçağın yönünü (heading) değiştirir.
// Parametreler:
//   heading_deg: Hedef yön (0-360 derece, kuzey=0)
//   type: NONE=kontrol kapat, COG=gidilen yöne dön, HEADING=mutlak yön
//   accel_limit_rad_s2: Yanal ivme limiti (varsayılan 10 rad/s², yaklaşık 25°/s²)
bool FlightCommander::changeHeading(float heading_deg, HeadingType type,
                                     float accel_limit_rad_s2) {
    std::lock_guard lock(cmd_mutex_);
    
    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeHeading: Not in GUIDED mode");
        return false;
    }
    
    // Normalize heading to 0-360
    while (heading_deg < 0.0f) heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_CMD_GUIDED_CHANGE_HEADING,  // 40002
        0,  // confirmation
        heading_deg,           // param1: Target heading in degrees
        static_cast<float>(static_cast<uint8_t>(type)),  // param2: Heading type
        accel_limit_rad_s2,    // param3: Acceleration limit (rad/s²)
        0, 0, 0, 0  // param4-7: unused
    );
    
    LOG_INFO("FlightCommander::changeHeading: heading={:.1f}° type={} accel={:.1f} rad/s²",
             heading_deg, static_cast<uint8_t>(type), accel_limit_rad_s2);
    
    pending_cmd_id_ = MAV_CMD_GUIDED_CHANGE_HEADING;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
}

// MAV_CMD_GUIDED_CHANGE_SPEED (40000)
// Uçağın hava hızını (airspeed) değiştirir.
// Parametreler:
//   airspeed_ms: Hedef hava hızı (m/s)
//   acceleration_ms2: İvme limiti (m/s²), varsayılan 2 m/s²
bool FlightCommander::changeAirspeed(float airspeed_ms, float acceleration_ms2) {
    std::lock_guard lock(cmd_mutex_);
    
    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeAirspeed: Not in GUIDED mode");
        return false;
    }
    
    // Validate airspeed range (typical fixed-wing: 10-50 m/s)
    if (airspeed_ms < 5.0f || airspeed_ms > 100.0f) {
        LOG_WARN("FlightCommander::changeAirspeed: Invalid airspeed {:.1f} m/s", airspeed_ms);
        return false;
    }
    
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_CMD_GUIDED_CHANGE_SPEED,  // 40000
        0,  // confirmation
        airspeed_ms,           // param1: Target airspeed (m/s)
        0,                     // param2: unused (for throttle)
        acceleration_ms2,      // param3: Acceleration limit (m/s²)
        0, 0, 0, 0  // param4-7: unused
    );
    
    LOG_INFO("FlightCommander::changeAirspeed: airspeed={:.1f} m/s accel={:.1f} m/s²",
             airspeed_ms, acceleration_ms2);
    
    pending_cmd_id_ = MAV_CMD_GUIDED_CHANGE_SPEED;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
}

// MAV_CMD_GUIDED_CHANGE_ALTITUDE (40001)
// Uçağın irtifasını değiştirir.
// Parametreler:
//   altitude_m: Hedef irtifa (metre)
//   climb_rate_ms: Tırmanma hızı (m/s), varsayılan 3 m/s
//   mav_frame: Referans çerçevesi (GLOBAL, LOCAL, vb.)
bool FlightCommander::changeAltitude(float altitude_m, float climb_rate_ms,
                                      uint8_t mav_frame) {
    std::lock_guard lock(cmd_mutex_);
    
    if (!isInGuidedMode()) {
        LOG_WARN("FlightCommander::changeAltitude: Not in GUIDED mode");
        return false;
    }
    
    // Validate altitude range (typical: 0-500m for safety)
    if (altitude_m < 0.0f || altitude_m > 1000.0f) {
        LOG_WARN("FlightCommander::changeAltitude: Invalid altitude {:.1f} m", altitude_m);
        return false;
    }
    
    // Validate climb rate (typical: 1-10 m/s for fixed-wing)
    climb_rate_ms = std::clamp(climb_rate_ms, 0.5f, 15.0f);
    
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        sys_id_, comp_id_, &msg,
        config_.target_sys, config_.target_comp,
        MAV_CMD_GUIDED_CHANGE_ALTITUDE,  // 40001
        0,  // confirmation
        altitude_m,            // param1: Target altitude (m)
        climb_rate_ms,         // param2: Climb/descent rate (m/s)
        0,                     // param3: unused (for heading)
        static_cast<float>(mav_frame),  // param4: MAV_FRAME
        0, 0, 0  // param5-7: unused
    );
    
    LOG_INFO("FlightCommander::changeAltitude: alt={:.1f} m rate={:.1f} m/s frame={}",
             altitude_m, climb_rate_ms, mav_frame);
    
    pending_cmd_id_ = MAV_CMD_GUIDED_CHANGE_ALTITUDE;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
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

    pending_cmd_id_ = MAV_CMD_DO_SET_MODE;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
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
    pending_cmd_id_ = MAV_CMD_COMPONENT_ARM_DISARM;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
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
    pending_cmd_id_ = MAV_CMD_COMPONENT_ARM_DISARM;
    commands_sent_++;
    last_ack_status_ = AckStatus::WAITING;
    return sendMAVLinkMessage(msg);
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

void FlightCommander::handleCommandAck(uint16_t cmd_id, uint8_t result) {
    commands_acked_++;
    last_ack_time_ = std::chrono::steady_clock::now();

    if (result == MAV_RESULT_ACCEPTED) {
        last_ack_status_ = AckStatus::ACCEPTED;
        LOG_DEBUG("FlightCommander: CMD {} accepted", cmd_id);
    } else if (result == MAV_RESULT_IN_PROGRESS) {
        LOG_DEBUG("FlightCommander: CMD {} in progress", cmd_id);
    } else {
        last_ack_status_ = AckStatus::REJECTED;
        LOG_WARN("FlightCommander: CMD {} rejected (result={})", cmd_id, result);
    }

    if (ack_callback_) {
        ack_callback_(cmd_id, result);
    }
}

bool FlightCommander::sendMAVLinkMessage(mavlink_message_t& msg) {
    if (!conn_ || !conn_->isConnected()) return false;

    MAVLinkMessage wrapped;
    wrapped.raw = msg;
    return conn_->sendMessage(wrapped);
}

bool FlightCommander::isPositionDuplicate(const core::Position& target) const {
    if (std::abs(last_position_cmd_.latitude) < 1e-9) return false;
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
