#include "uav/comm/statustext_parser.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>
#include <regex>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace uav::comm {

std::optional<ParsedStatusText> StatusTextParser::parse(const std::string& text) {
    if (text.empty()) return std::nullopt;

    cleanupStaleBuffers();

    std::string prefix, content;
    int part, total;
    if (isMultiPart(text, prefix, part, total, content)) {
        std::string key = prefix + "_" + std::to_string(total);

        if (buffer_.find(key) == buffer_.end()) {
            BufferEntry entry;
            entry.parts.resize(total);
            entry.total_parts = total;
            entry.received_count = 0;
            entry.timestamp = std::chrono::steady_clock::now();
            buffer_[key] = entry;
        }

        auto& entry = buffer_[key];
        int idx = part - 1;
        if (idx >= 0 && idx < total) {
            if (entry.parts[idx].empty()) {
                entry.parts[idx] = content;
                entry.received_count++;
            }
            entry.timestamp = std::chrono::steady_clock::now();
        }

        if (entry.received_count >= entry.total_parts) {
            std::string combined;
            for (const auto& p : entry.parts) combined += p;
            buffer_.erase(key);
            LOG_DEBUG("Multi-part complete: {} ({} parts)", prefix, total);
            return parse(combined);
        }

        return std::nullopt;
    }

    if (text.find("RIV:") == 0) return parseRIV(text);
    if (text.find("HSS:") == 0) return parseHSS(text);
    if (text.find("BND:") == 0) return parseBND(text);
    if (text.find("LOCK:") == 0) return parseModeCommand(text);
    if (text.find("KAMIKAZE:") == 0) return parseModeCommand(text);
    if (text == "FREE" || text == "ARDUPILOT") return parseModeCommand(text);

    return std::nullopt;
}

void StatusTextParser::clearBuffer() { buffer_.clear(); }

// RIV:ID:LAT:LON:ALT:HDG:SPD:TS
std::optional<ParsedStatusText> StatusTextParser::parseRIV(const std::string& text) {
    auto parts = split(text, ':');
    if (parts.size() < 8) {
        LOG_WARN("Invalid RIV format (need 8 fields): {}", text);
        return std::nullopt;
    }

    try {
        ParsedStatusText result;
        result.type = StatusTextType::RIV;

        core::UAVState opp;
        opp.id = toInt(parts[1]);
        opp.position.latitude = toDouble(parts[2]);
        opp.position.longitude = toDouble(parts[3]);
        opp.position.altitude = toDouble(parts[4]);

        double heading = toDouble(parts[5]);
        double speed = toDouble(parts[6]);

        double vn, ve;
        headingSpeedToVelocity(heading, speed, vn, ve);
        opp.velocity.north = vn;
        opp.velocity.east = ve;
        opp.velocity.down = 0.0;

        opp.heading = heading;
        opp.groundspeed = speed;
        opp.airspeed = speed;
        opp.timestamp_us = time::nowUs();

        result.opponent = opp;

        LOG_INFO("Parsed RIV: id={}, pos=({:.6f},{:.6f},{:.1f}), hdg={:.1f}, spd={:.1f}",
                 opp.id, opp.position.latitude, opp.position.longitude,
                 opp.position.altitude, heading, speed);
        return result;

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse RIV: {} - {}", text, e.what());
        return std::nullopt;
    }
}

// HSS:COUNT:ID1:LAT1:LON1:RAD1:ID2:LAT2:LON2:RAD2:...
std::optional<ParsedStatusText> StatusTextParser::parseHSS(const std::string& text) {
    auto parts = split(text, ':');
    if (parts.size() < 6) {
        LOG_WARN("Invalid HSS format (need >=6 fields): {}", text);
        return std::nullopt;
    }

    try {
        ParsedStatusText result;
        result.type = StatusTextType::HSS;

        int count = toInt(parts[1]);
        result.hss_count = count;

        int idx = 2;
        for (int i = 0; i < count; i++) {
            if (idx + 4 > static_cast<int>(parts.size())) {
                LOG_WARN("HSS: truncated at zone {}", i);
                break;
            }

            core::HSSZone zone;
            zone.id = toInt(parts[idx]);
            zone.center.latitude = toDouble(parts[idx + 1]);
            zone.center.longitude = toDouble(parts[idx + 2]);
            zone.center.altitude = 0.0;
            zone.radius = toDouble(parts[idx + 3]);
            zone.active = true;

            result.hss_zones.push_back(zone);
            idx += 4;

            LOG_DEBUG("Parsed HSS zone: id={}, center=({:.6f},{:.6f}), r={:.1f}m",
                      zone.id, zone.center.latitude, zone.center.longitude, zone.radius);
        }

        LOG_INFO("Parsed HSS: {} zones", result.hss_zones.size());
        return result;

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse HSS: {} - {}", text, e.what());
        return std::nullopt;
    }
}

// BND:COUNT:LAT1:LON1:LAT2:LON2:...
std::optional<ParsedStatusText> StatusTextParser::parseBND(const std::string& text) {
    auto parts = split(text, ':');
    if (parts.size() < 8) {
        LOG_WARN("Invalid BND format (need >=8 fields for 3 vertices): {}", text);
        return std::nullopt;
    }

    try {
        ParsedStatusText result;
        result.type = StatusTextType::BND;

        int count = toInt(parts[1]);
        result.boundary_vertex_count = count;

        core::Boundary boundary;
        boundary.min_altitude = 50.0;
        boundary.max_altitude = 500.0;

        int idx = 2;
        for (int i = 0; i < count; i++) {
            if (idx + 2 > static_cast<int>(parts.size())) {
                LOG_WARN("BND: truncated at vertex {}", i);
                break;
            }

            core::Position pos;
            pos.latitude = toDouble(parts[idx]);
            pos.longitude = toDouble(parts[idx + 1]);
            pos.altitude = 0.0;
            boundary.vertices.push_back(pos);
            idx += 2;
        }

        if (boundary.vertices.size() < 3) {
            LOG_WARN("BND: need >=3 vertices, got {}", boundary.vertices.size());
            return std::nullopt;
        }

        result.boundary = boundary;
        LOG_INFO("Parsed BND: {} vertices", boundary.vertices.size());
        return result;

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse BND: {} - {}", text, e.what());
        return std::nullopt;
    }
}

// FREE, ARDUPILOT, LOCK:TARGET_ID, KAMIKAZE:LAT:LON
std::optional<ParsedStatusText> StatusTextParser::parseModeCommand(const std::string& text) {
    ParsedStatusText result;
    result.type = StatusTextType::MODE_CMD;
    result.mode_command = text;

    try {
        if (text == "FREE") {
            LOG_INFO("Mode command: FREE");
        } else if (text == "ARDUPILOT") {
            LOG_INFO("Mode command: ARDUPILOT");
        } else if (text.find("LOCK:") == 0) {
            auto parts = split(text, ':');
            if (parts.size() >= 2) {
                result.lock_target_id = toInt(parts[1]);
                LOG_INFO("Mode command: LOCK target_id={}", *result.lock_target_id);
            }
        } else if (text.find("KAMIKAZE:") == 0) {
            auto parts = split(text, ':');
            if (parts.size() >= 3) {
                core::Position target;
                target.latitude = toDouble(parts[1]);
                target.longitude = toDouble(parts[2]);
                target.altitude = 0.0;
                result.kamikaze_target = target;
                LOG_INFO("Mode command: KAMIKAZE ({:.6f},{:.6f})", target.latitude, target.longitude);
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse mode command: {} - {}", text, e.what());
    }

    return result;
}

bool StatusTextParser::isMultiPart(const std::string& text,
                                   std::string& prefix, int& part, int& total,
                                   std::string& content) {
    // PREFIX[PART/TOTAL]:content
    static const std::regex pattern(R"(^(HSS|BND|RIV)\[(\d+)/(\d+)\]:(.*)$)");
    std::smatch match;

    if (std::regex_match(text, match, pattern)) {
        prefix = match[1].str();
        part = std::stoi(match[2].str());
        total = std::stoi(match[3].str());
        content = match[4].str();
        return true;
    }
    return false;
}

std::vector<std::string> StatusTextParser::split(const std::string& str, char delim) {
    std::vector<std::string> tokens;
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

double StatusTextParser::toDouble(const std::string& s) { return std::stod(s); }
int StatusTextParser::toInt(const std::string& s) { return std::stoi(s); }

void StatusTextParser::headingSpeedToVelocity(double heading_deg, double speed_ms,
                                              double& vn, double& ve) {
    double rad = heading_deg * M_PI / 180.0;
    vn = speed_ms * std::cos(rad);
    ve = speed_ms * std::sin(rad);
}

void StatusTextParser::cleanupStaleBuffers() {
    auto now = std::chrono::steady_clock::now();
    for (auto it = buffer_.begin(); it != buffer_.end();) {
        if (now - it->second.timestamp > std::chrono::seconds(10)) {
            it = buffer_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace uav::comm
