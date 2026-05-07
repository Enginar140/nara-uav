#include "uav/comm/mavlink_bridge.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <cstring>

namespace uav::comm {

MAVLinkBridge::MAVLinkBridge(const std::string& pixhawk_conn,
                             const std::string& gcs_conn,
                             std::shared_ptr<infra::ThreadPool> thread_pool,
                             std::shared_ptr<world_model::WorldModel> world_model,
                             std::shared_ptr<infra::EventBus> event_bus)
    : thread_pool_(std::move(thread_pool)),
      world_model_(std::move(world_model)),
      event_bus_(std::move(event_bus)) {

    pixhawk_ = std::make_shared<ConnectionManager>(pixhawk_conn);
    gcs_ = std::make_shared<ConnectionManager>(gcs_conn);
    flight_commander_ = std::make_shared<FlightCommander>(pixhawk_);
    param_reader_ = std::make_shared<ParamReader>(pixhawk_);
}

MAVLinkBridge::~MAVLinkBridge() { stop(); }

bool MAVLinkBridge::start() {
    if (running_) return true;

    if (!pixhawk_->connect()) {
        LOG_ERROR("MAVLinkBridge: pixhawk connection failed");
        return false;
    }

    pixhawk_->setMessageCallback([this](const MAVLinkMessage& msg) {
        handlePixhawkMessage(msg);
    });

    if (gcs_->connect()) {
        gcs_->setMessageCallback([this](const MAVLinkMessage& msg) {
            handleGCSMessage(msg);
        });
    } else {
        LOG_WARN("MAVLinkBridge: GCS connection failed (non-critical)");
    }

    running_ = true;
    heartbeat_thread_ = std::thread(&MAVLinkBridge::heartbeatLoop, this);

    param_reader_->setReadyCallback([this] {
        event_bus_->publish(infra::EventType::PARAMS_READY);
    });

    thread_pool_->submit([this] {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        for (int i = 0; running_.load() && !pixhawk_->isConnected() && i < 30; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (!running_.load() || !pixhawk_->isConnected()) {
            LOG_WARN("MAVLinkBridge: pixhawk not connected, deferring parameter request");
            return;
        }
        if (!params_requested_.exchange(true)) {
            param_reader_->requestCritical();
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (!param_reader_->areCriticalReady()) {
            LOG_WARN("MAVLinkBridge: critical params not received after 5s, requesting all");
            param_reader_->requestAll();
        }
    });

    LOG_INFO("MAVLinkBridge: started");
    return true;
}

void MAVLinkBridge::stop() {
    if (!running_) return;
    running_ = false;
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    pixhawk_->disconnect();
    gcs_->disconnect();
    LOG_INFO("MAVLinkBridge: stopped");
}

MAVLinkBridge::Stats MAVLinkBridge::getStats() const {
    std::lock_guard lock(stats_mutex_);
    Stats s = stats_;
    s.pixhawk_connected = pixhawk_->isConnected();
    s.gcs_connected = gcs_->isConnected();
    s.heartbeat_timeout = heartbeat_timeout_.load();
    s.armed = flight_commander_->isArmed();
    s.flight_mode = toString(flight_commander_->getCurrentMode());
    return s;
}

void MAVLinkBridge::handlePixhawkMessage(const MAVLinkMessage& msg) {
    {
        std::lock_guard lock(stats_mutex_);
        stats_.messages_received++;
    }

    switch (msg.msgid()) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gpos;
            mavlink_msg_global_position_int_decode(&msg.raw, &gpos);

            auto state = world_model_->getOwnState();
            state.id = 0;
            state.position.latitude = gpos.lat / 1e7;
            state.position.longitude = gpos.lon / 1e7;
            state.position.altitude = gpos.relative_alt / 1e3;
            state.velocity.north = gpos.vx / 100.0;
            state.velocity.east = gpos.vy / 100.0;
            state.velocity.down = gpos.vz / 100.0;
            state.heading = gpos.hdg / 100.0;
            state.groundspeed = state.velocity.horizontalSpeed();
            state.timestamp_us = time::nowUs();

            world_model_->setOwnState(state);
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(&msg.raw, &hud);

            auto state = world_model_->getOwnState();
            state.airspeed = hud.airspeed;
            state.heading = hud.heading;
            state.timestamp_us = time::nowUs();
            world_model_->setOwnState(state);
            break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg.raw, &gps);

            auto state = world_model_->getOwnState();
            state.gps_fix_type = gps.fix_type;
            state.satellites_visible = gps.satellites_visible;
            state.gps_hdop = gps.eph;
            state.timestamp_us = time::nowUs();
            world_model_->setOwnState(state);
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&msg.raw, &sys);

            auto state = world_model_->getOwnState();
            state.battery_voltage = sys.voltage_battery / 1000.0f;
            state.battery_remaining = sys.battery_remaining;
            state.timestamp_us = time::nowUs();
            world_model_->setOwnState(state);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg.raw, &att);

            auto state = world_model_->getOwnState();
            state.attitude.roll = att.roll * 57.2958;
            state.attitude.pitch = att.pitch * 57.2958;
            state.attitude.yaw = att.yaw * 57.2958;
            state.timestamp_us = time::nowUs();
            world_model_->setOwnState(state);
            break;
        }

        case MAVLINK_MSG_ID_STATUSTEXT:
            handleStatusText(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;

        case MAVLINK_MSG_ID_PARAM_VALUE:
            param_reader_->handleParamValue(msg.raw);
            break;

        default:
            break;
    }

    if (gcs_->isConnected()) {
        gcs_->sendMessage(msg);
    }
}

void MAVLinkBridge::handleGCSMessage(const MAVLinkMessage& msg) {
    if (msg.msgid() == MAVLINK_MSG_ID_STATUSTEXT) {
        handleStatusText(msg);
    }

    if (pixhawk_->isConnected()) {
        pixhawk_->sendMessage(msg);
    }
}

void MAVLinkBridge::handleHeartbeat(const MAVLinkMessage& msg) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg.raw, &hb);

    if (hb.type == MAV_TYPE_GCS || hb.type == MAV_TYPE_ONBOARD_CONTROLLER) {
        return;
    }

    auto prev_mode = flight_commander_->getCurrentMode();
    flight_commander_->updateModeFromHeartbeat(hb.custom_mode);
    flight_commander_->updateArmedFromHeartbeat(hb.base_mode);

    auto new_mode = flight_commander_->getCurrentMode();
    if (prev_mode != new_mode) {
        LOG_INFO("MAVLinkBridge: AP mode changed {} -> {} (custom_mode={}, base_mode=0x{:02X}, type={})",
                 toString(prev_mode), toString(new_mode),
                 hb.custom_mode, hb.base_mode, hb.type);
    }

    if (heartbeat_timeout_) {
        heartbeat_timeout_ = false;
        event_bus_->publish(infra::EventType::HEARTBEAT_RECOVERED);
        LOG_INFO("MAVLinkBridge: heartbeat recovered");
    }

    event_bus_->publish(infra::EventType::HEARTBEAT_RECEIVED);

    if (!params_requested_.exchange(true)) {
        param_reader_->requestCritical();
    }

    {
        std::lock_guard lock(stats_mutex_);
        stats_.last_heartbeat_us = time::nowUs();
        stats_.armed = flight_commander_->isArmed();
    }
}

void MAVLinkBridge::handleStatusText(const MAVLinkMessage& msg) {
    mavlink_statustext_t st;
    mavlink_msg_statustext_decode(&msg.raw, &st);

    char text_buf[51];
    std::memcpy(text_buf, st.text, 50);
    text_buf[50] = '\0';
    std::string text(text_buf);

    LOG_DEBUG("STATUSTEXT: [{}]", text);

    auto parsed = statustext_parser_.parse(text);
    if (!parsed.has_value()) return;

    switch (parsed->type) {
        case StatusTextType::RIV:
            if (parsed->opponent.has_value()) {
                world_model_->updateOpponent(*parsed->opponent);
                event_bus_->publish(infra::EventType::OPPONENT_UPDATED,
                                    *parsed->opponent);
            }
            break;

        case StatusTextType::HSS:
            world_model_->replaceHSSZones(parsed->hss_zones);
            if (parsed->hss_zones.empty()) {
                event_bus_->publish(infra::EventType::HSS_ZONE_UPDATED);
            }
            for (const auto& zone : parsed->hss_zones) {
                event_bus_->publish(infra::EventType::HSS_ZONE_UPDATED, zone);
            }
            break;

        case StatusTextType::BND:
            if (parsed->boundary.has_value()) {
                world_model_->setBoundary(*parsed->boundary);
                event_bus_->publish(infra::EventType::BOUNDARY_UPDATED,
                                    *parsed->boundary);
                LOG_INFO("MAVLinkBridge: boundary updated ({} vertices)",
                         parsed->boundary->vertices.size());
            }
            break;

        case StatusTextType::MODE_CMD:
            if (parsed->mode_command.has_value()) {
                LOG_INFO("MAVLinkBridge: mode command - {}", *parsed->mode_command);
                dispatchModeCommand(*parsed);
            }
            break;

        default:
            break;
    }
}

void MAVLinkBridge::handleCommandAck(const MAVLinkMessage& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg.raw, &ack);
    flight_commander_->handleCommandAck(ack.command, ack.result);
}

void MAVLinkBridge::dispatchModeCommand(const ParsedStatusText& parsed) {
    if (!parsed.mode_command.has_value()) return;

    const auto& cmd = *parsed.mode_command;

    ParsedModeCommand mode_cmd;

    if (cmd == "FREE" || parsed.isLockCommand()) {
        if (parsed.isLockCommand()) {
            mode_cmd.target_mode = core::MissionMode::FOLLOW;
            mode_cmd.target_id = parsed.lock_target_id;
        } else {
            mode_cmd.target_mode = core::MissionMode::PATROL;
        }
    } else if (cmd == "ARDUPILOT") {
        mode_cmd.target_mode = core::MissionMode::ARDUPILOT;
    } else if (parsed.isKamikazeCommand()) {
        mode_cmd.target_mode = core::MissionMode::QR_DIVE;
        mode_cmd.target_pos = parsed.kamikaze_target;
    } else {
        LOG_WARN("MAVLinkBridge: unknown mode command '{}'", cmd);
        return;
    }

    event_bus_->publish(infra::EventType::MODE_COMMAND_RECEIVED, mode_cmd);
}

void MAVLinkBridge::heartbeatLoop() {
    int reconnect_backoff_ms = 1000;

    while (running_) {
        // Send heartbeat to pixhawk
        mavlink_message_t hb_msg{};
        mavlink_msg_heartbeat_pack(
            1, MAV_COMP_ID_ONBOARD_COMPUTER, &hb_msg,
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            0, 0, MAV_STATE_ACTIVE);

        MAVLinkMessage wrapped;
        wrapped.raw = hb_msg;

        if (pixhawk_->isConnected()) {
            pixhawk_->sendMessage(wrapped);
        }
        if (gcs_->isConnected()) {
            gcs_->sendMessage(wrapped);
        }

        {
            std::lock_guard lock(stats_mutex_);
            stats_.messages_sent++;
        }

        // Check heartbeat timeout
        {
            std::lock_guard lock(stats_mutex_);
            if (stats_.last_heartbeat_us > 0) {
                auto age_ms = (time::nowUs() - stats_.last_heartbeat_us) / 1000;
                if (age_ms > static_cast<uint64_t>(HEARTBEAT_TIMEOUT_MS)) {
                    if (!heartbeat_timeout_) {
                        heartbeat_timeout_ = true;
                        event_bus_->publish(infra::EventType::HEARTBEAT_TIMEOUT);
                        LOG_WARN("MAVLinkBridge: heartbeat timeout");
                    }
                }
            }
        }

        // Auto-reconnect pixhawk if lost
        if (!pixhawk_->isConnected() &&
            pixhawk_->getState() != ConnectionState::CONNECTING) {
            LOG_WARN("MAVLinkBridge: pixhawk disconnected, reconnecting in {}ms",
                     reconnect_backoff_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_backoff_ms));
            if (pixhawk_->reconnect()) {
                pixhawk_->setMessageCallback([this](const MAVLinkMessage& m) {
                    handlePixhawkMessage(m);
                });
                reconnect_backoff_ms = 1000;
                LOG_INFO("MAVLinkBridge: pixhawk reconnected");
                event_bus_->publish(infra::EventType::MAVLINK_CONNECTED);
                if (!params_requested_.exchange(true)) {
                    param_reader_->requestCritical();
                }
            } else {
                reconnect_backoff_ms = std::min(reconnect_backoff_ms * 2, 30000);
            }
        } else {
            reconnect_backoff_ms = 1000;
        }

        // Auto-reconnect GCS if lost
        if (!gcs_->isConnected() &&
            gcs_->getState() != ConnectionState::CONNECTING) {
            gcs_->reconnect();
            if (gcs_->isConnected()) {
                gcs_->setMessageCallback([this](const MAVLinkMessage& m) {
                    handleGCSMessage(m);
                });
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

} // namespace uav::comm
