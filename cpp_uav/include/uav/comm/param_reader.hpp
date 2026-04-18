#pragma once

#include "uav/comm/mavlink_message.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace uav::comm {

class ConnectionManager;

class ParamReader {
public:
    using ReadyCallback = std::function<void()>;

    explicit ParamReader(std::shared_ptr<ConnectionManager> conn,
                         uint8_t target_sys = 1, uint8_t target_comp = 1);

    void requestAll();
    void requestParam(const std::string& param_id);
    void requestCritical();

    void handleParamValue(const mavlink_message_t& msg);

    bool hasParam(const std::string& name) const;
    float getParam(const std::string& name, float default_val = 0.0f) const;
    size_t receivedCount() const;
    int totalCount() const { return total_params_.load(); }
    bool isComplete() const;
    bool areCriticalReady() const;

    void setReadyCallback(ReadyCallback cb) { ready_cb_ = std::move(cb); }

    float getStallSpeed() const   { return getParam("ARSPD_FBW_MIN", 12.0f); }
    float getMinAirspeed() const  { return getParam("ARSPD_FBW_MIN", 12.0f); }
    float getMaxAirspeed() const  { return getParam("ARSPD_FBW_MAX", 27.0f); }
    float getCruiseSpeed() const  { return getParam("TRIM_ARSPD_CM", 1500.0f) / 100.0f; }
    float getMaxBankAngle() const { return getParam("LIM_ROLL_CD", 6000.0f) / 100.0f; }
    float getWPRadius() const     { return getParam("WP_RADIUS", 80.0f); }
    float getFenceAltMax() const  { return getParam("FENCE_ALT_MAX", 400.0f); }
    float getFenceAltMin() const  { return getParam("FENCE_ALT_MIN", 30.0f); }
    float getMinPitch() const     { return getParam("LIM_PITCH_MIN", -25.0f); }
    float getLoiterRadius() const { return getParam("WP_LOITER_RAD", 60.0f); }

    float estimateTurnRadius() const;

    static const std::vector<std::string>& criticalParamNames();

private:
    std::shared_ptr<ConnectionManager> conn_;
    uint8_t target_sys_;
    uint8_t target_comp_;

    mutable std::mutex params_mutex_;
    std::unordered_map<std::string, float> params_;
    std::atomic<int> total_params_{-1};
    std::chrono::steady_clock::time_point request_time_;

    std::unordered_set<std::string> critical_pending_;
    std::atomic<bool> critical_ready_{false};
    std::atomic<bool> all_received_{false};
    ReadyCallback ready_cb_;

    void checkCriticalReady();
};

} // namespace uav::comm
