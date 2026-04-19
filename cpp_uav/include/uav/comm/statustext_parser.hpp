#pragma once

#include "uav/core/types.hpp"

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace uav::comm {

// TEKNOFEST Competition STATUSTEXT Protocol
//
// RIV:ID:LAT:LON:ALT:HDG:SPD:TS        → Opponent data
// HSS:COUNT:ID1:LAT1:LON1:RAD1:...     → HSS zones
// BND:COUNT:LAT1:LON1:LAT2:LON2:...    → Boundary vertices
// LOCK:TARGET_ID                         → Follow/lock target
// KAMIKAZE:LAT:LON                       → Dive to ground target
// FREE                                   → Free mode (patrol)
// ARDUPILOT                              → ArduPilot passthrough
//
// Multi-part: PREFIX[PART/TOTAL]:content
//   e.g. HSS[1/3]:HSS:5:1:39.82:32.75:100:...

enum class StatusTextType {
    UNKNOWN,
    RIV,
    HSS,
    BND,
    MODE_CMD
};

struct ParsedStatusText {
    StatusTextType type = StatusTextType::UNKNOWN;

    std::optional<core::UAVState> opponent;

    std::vector<core::HSSZone> hss_zones;
    int hss_count = 0;

    std::optional<core::Boundary> boundary;
    int boundary_vertex_count = 0;

    std::optional<std::string> mode_command;
    std::optional<int> lock_target_id;
    std::optional<core::Position> kamikaze_target;

    bool isLockCommand() const {
        return type == StatusTextType::MODE_CMD && lock_target_id.has_value();
    }
    bool isKamikazeCommand() const {
        return type == StatusTextType::MODE_CMD && kamikaze_target.has_value();
    }
};

struct ParsedModeCommand {
    core::MissionMode target_mode;
    std::optional<int> target_id;
    std::optional<core::Position> target_pos;
};

class StatusTextParser {
public:
    StatusTextParser() = default;

    std::optional<ParsedStatusText> parse(const std::string& text);
    void clearBuffer();

private:
    struct BufferEntry {
        std::vector<std::string> parts;
        int total_parts = 0;
        int received_count = 0;
        std::chrono::steady_clock::time_point timestamp;
    };

    // buffer_ is touched from multiple receive threads (pixhawk + gcs).
    // Guard all access with buffer_mutex_.
    std::map<std::string, BufferEntry> buffer_;
    mutable std::mutex buffer_mutex_;

    void cleanupStaleBuffers();

    std::optional<ParsedStatusText> parseRIV(const std::string& text);
    std::optional<ParsedStatusText> parseHSS(const std::string& text);
    std::optional<ParsedStatusText> parseBND(const std::string& text);
    std::optional<ParsedStatusText> parseModeCommand(const std::string& text);

    bool isMultiPart(const std::string& text,
                     std::string& prefix, int& part, int& total,
                     std::string& content);

    static std::vector<std::string> split(const std::string& str, char delim);
    static double toDouble(const std::string& s);
    static int toInt(const std::string& s);
    static void headingSpeedToVelocity(double heading_deg, double speed_ms,
                                       double& vn, double& ve);
};

} // namespace uav::comm
