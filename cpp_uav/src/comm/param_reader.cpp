#include "uav/comm/param_reader.hpp"
#include "uav/comm/connection_manager.hpp"
#include "uav/infra/log_manager.hpp"

#include <cmath>
#include <cstring>

namespace uav::comm {

const std::vector<std::string>& ParamReader::criticalParamNames() {
    static const std::vector<std::string> names = {
        "ARSPD_FBW_MIN",
        "ARSPD_FBW_MAX",
        "TRIM_ARSPD_CM",
        "LIM_ROLL_CD",
        "WP_RADIUS",
        "WP_LOITER_RAD",
        "FENCE_ALT_MAX",
        "FENCE_ALT_MIN",
        "LIM_PITCH_MIN"
    };
    return names;
}

ParamReader::ParamReader(std::shared_ptr<ConnectionManager> conn,
                         uint8_t target_sys, uint8_t target_comp)
    : conn_(std::move(conn)), target_sys_(target_sys), target_comp_(target_comp) {
    for (const auto& name : criticalParamNames()) {
        critical_pending_.insert(name);
    }
}

void ParamReader::requestAll() {
    if (!conn_ || !conn_->isConnected()) return;

    mavlink_message_t msg{};
    mavlink_msg_param_request_list_pack(
        1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
        target_sys_, target_comp_
    );

    MAVLinkMessage wrapped;
    wrapped.raw = msg;
    conn_->sendMessage(wrapped);
    request_time_ = std::chrono::steady_clock::now();
    LOG_INFO("ParamReader: requested all parameters");
}

void ParamReader::requestCritical() {
    if (!conn_ || !conn_->isConnected()) return;

    for (const auto& name : criticalParamNames()) {
        requestParam(name);
    }
    request_time_ = std::chrono::steady_clock::now();
    LOG_INFO("ParamReader: requested {} critical parameters", criticalParamNames().size());
}

void ParamReader::requestParam(const std::string& param_id) {
    if (!conn_ || !conn_->isConnected()) return;

    mavlink_message_t msg{};
    char id[17]{};
    std::strncpy(id, param_id.c_str(), 16);

    mavlink_msg_param_request_read_pack(
        1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
        target_sys_, target_comp_,
        id, -1
    );

    MAVLinkMessage wrapped;
    wrapped.raw = msg;
    conn_->sendMessage(wrapped);
}

void ParamReader::handleParamValue(const mavlink_message_t& msg) {
    mavlink_param_value_t pv;
    mavlink_msg_param_value_decode(&msg, &pv);

    char id[17]{};
    std::memcpy(id, pv.param_id, 16);
    id[16] = '\0';

    std::string name(id);

    {
        std::lock_guard lock(params_mutex_);
        params_[name] = pv.param_value;

        auto it = critical_pending_.find(name);
        if (it != critical_pending_.end()) {
            LOG_INFO("ParamReader: critical param {} = {}", name, pv.param_value);
            critical_pending_.erase(it);
        }
    }

    total_params_ = static_cast<int>(pv.param_count);

    checkCriticalReady();

    if (!all_received_.load() && receivedCount() >= static_cast<size_t>(pv.param_count)) {
        all_received_ = true;
        LOG_INFO("ParamReader: all {} parameters received", pv.param_count);
    }
}

void ParamReader::checkCriticalReady() {
    if (critical_ready_.load()) return;

    bool ready;
    {
        std::lock_guard lock(params_mutex_);
        ready = critical_pending_.empty();
    }

    if (ready) {
        critical_ready_ = true;
        LOG_INFO("ParamReader: all critical parameters received"
                 " | stall={:.1f} cruise={:.1f} max={:.1f}"
                 " | bank={:.1f}deg turnR={:.0f}m"
                 " | WP_R={:.0f} loiter={:.0f}",
                 getStallSpeed(), getCruiseSpeed(), getMaxAirspeed(),
                 getMaxBankAngle(), estimateTurnRadius(),
                 getWPRadius(), getLoiterRadius());

        if (ready_cb_) {
            ready_cb_();
        }
    }
}

bool ParamReader::hasParam(const std::string& name) const {
    std::lock_guard lock(params_mutex_);
    return params_.find(name) != params_.end();
}

float ParamReader::getParam(const std::string& name, float default_val) const {
    std::lock_guard lock(params_mutex_);
    auto it = params_.find(name);
    if (it != params_.end()) return it->second;
    return default_val;
}

size_t ParamReader::receivedCount() const {
    std::lock_guard lock(params_mutex_);
    return params_.size();
}

bool ParamReader::isComplete() const {
    int total = total_params_.load();
    if (total < 0) return false;
    return receivedCount() >= static_cast<size_t>(total);
}

bool ParamReader::areCriticalReady() const {
    return critical_ready_.load();
}

float ParamReader::estimateTurnRadius() const {
    float speed = getCruiseSpeed();
    float bank_deg = getMaxBankAngle();
    float bank_rad = bank_deg * 3.14159f / 180.0f;
    float g = 9.81f;

    if (std::abs(std::tan(bank_rad)) < 0.01f) return 100.0f;
    return (speed * speed) / (g * std::tan(bank_rad));
}

} // namespace uav::comm
