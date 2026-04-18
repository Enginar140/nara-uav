#pragma once

#include "uav/core/types.hpp"
#include "uav/comm/mavlink_message.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
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

// Guided Slew Commands için heading tipi
// ArduPilot'un GUIDED_HEADING_* değerleriyle uyumlu
enum class HeadingType : uint8_t {
    NONE    = 0,  // Heading kontrolü kapalı
    COG     = 1,  // Course Over Ground - uçağın gittiği yöne dön
    HEADING = 2   // Mutlak heading - belirtilen yöne dön
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

    [[nodiscard]] bool flyToPosition(const core::Position& target, float airspeed = -1.0f);
    [[nodiscard]] bool flyToLocalPosition(float north_m, float east_m, float down_m);
    [[nodiscard]] bool setLocalVelocity(float vn, float ve, float vd, float yaw_rate = 0.0f);

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
    AckStatus getLastAckStatus() const { return last_ack_status_.load(); }

private:
    std::shared_ptr<ConnectionManager> conn_;
    uint8_t sys_id_;
    uint8_t comp_id_;
    Config config_;

    std::atomic<ArduPilotMode> current_mode_{ArduPilotMode::MANUAL};
    std::atomic<bool> armed_{false};
    std::atomic<uint64_t> commands_sent_{0};
    std::atomic<uint64_t> commands_acked_{0};
    std::atomic<AckStatus> last_ack_status_{AckStatus::NONE};
    std::atomic<uint16_t> pending_cmd_id_{0};
    std::chrono::steady_clock::time_point last_ack_time_;

    AckCallback ack_callback_;

    std::recursive_mutex cmd_mutex_;
    core::Position last_position_cmd_;

    bool sendMAVLinkMessage(mavlink_message_t& msg);
    bool isPositionDuplicate(const core::Position& target) const;
};

const char* toString(ArduPilotMode mode);

} // namespace uav::comm
