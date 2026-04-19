#include "uav/perception/perception_engine.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

#include <algorithm>
#include <utility>

namespace uav::perception {

const char* toString(PerceptionFrame frame) {
    switch (frame) {
        case PerceptionFrame::BODY:       return "BODY";
        case PerceptionFrame::LOCAL_NED:  return "LOCAL_NED";
        case PerceptionFrame::POLAR:      return "POLAR";
        case PerceptionFrame::VISION_LOS: return "VISION_LOS";
    }
    return "UNKNOWN";
}

PerceptionEngine::PerceptionEngine(std::shared_ptr<infra::EventBus> bus,
                                   std::shared_ptr<world_model::WorldModel> world_model,
                                   Config config)
    : bus_(std::move(bus)),
      world_model_(std::move(world_model)),
      config_(config) {}

PerceptionEngine::~PerceptionEngine() { stop(); }

void PerceptionEngine::registerModule(std::shared_ptr<IPerceptionModule> module) {
    if (!module) return;
    auto id = module->id();
    std::lock_guard lock(active_mutex_);
    modules_[id] = std::move(module);
    LOG_INFO("PerceptionEngine: registered module '{}'", id);
}

bool PerceptionEngine::setActive(const std::string& id) {
    std::shared_ptr<IPerceptionModule> next;
    std::shared_ptr<IPerceptionModule> prev;
    std::string prev_id;
    {
        std::lock_guard lock(active_mutex_);
        if (id.empty()) {
            prev = active_;
            prev_id = active_id_;
            active_.reset();
            active_id_.clear();
        } else {
            auto it = modules_.find(id);
            if (it == modules_.end()) {
                LOG_WARN("PerceptionEngine: setActive('{}') unknown module", id);
                return false;
            }
            if (active_id_ == id) return true;
            prev = active_;
            prev_id = active_id_;
            next = it->second;
            active_ = next;
            active_id_ = id;
        }
    }
    if (prev) {
        try { prev->stop(); }
        catch (const std::exception& e) {
            LOG_ERROR("PerceptionEngine: '{}' stop() threw: {}", prev_id, e.what());
        }
    }
    if (next) {
        try { next->start(); }
        catch (const std::exception& e) {
            LOG_ERROR("PerceptionEngine: '{}' start() threw: {}", id, e.what());
        }
    }
    LOG_INFO("PerceptionEngine: active '{}' -> '{}'",
             prev_id.empty() ? "<none>" : prev_id,
             id.empty() ? "<none>" : id);
    return true;
}

std::string PerceptionEngine::activeId() const {
    std::lock_guard lock(active_mutex_);
    return active_id_;
}

std::vector<std::string> PerceptionEngine::registered() const {
    std::lock_guard lock(active_mutex_);
    std::vector<std::string> out;
    out.reserve(modules_.size());
    for (const auto& [id, _] : modules_) out.push_back(id);
    std::sort(out.begin(), out.end());
    return out;
}

void PerceptionEngine::start() {
    if (running_) return;
    running_ = true;
    last_lock_time_ = time::nowSteady();
    last_was_lock_ = false;
    thread_ = std::thread(&PerceptionEngine::tickLoop, this);
    LOG_INFO("PerceptionEngine: started @ {:.1f} Hz", config_.tick_rate_hz);
}

void PerceptionEngine::stop() {
    if (!running_) return;
    running_ = false;
    if (thread_.joinable()) thread_.join();

    std::shared_ptr<IPerceptionModule> prev;
    {
        std::lock_guard lock(active_mutex_);
        prev = active_;
        active_.reset();
        active_id_.clear();
    }
    if (prev) {
        try { prev->stop(); }
        catch (...) {}
    }
    LOG_INFO("PerceptionEngine: stopped");
}

void PerceptionEngine::tickLoop() {
    const auto period = std::chrono::milliseconds(
        std::max<long long>(20,
            static_cast<long long>(1000.0 / std::max(1.0, config_.tick_rate_hz))));
    auto next_tick = std::chrono::steady_clock::now();

    while (running_) {
        std::shared_ptr<IPerceptionModule> module;
        {
            std::lock_guard lock(active_mutex_);
            module = active_;
        }

        std::optional<PerceptionTarget> result;
        if (module) {
            try {
                auto own = world_model_->getOwnState();
                result = module->poll(own);
            } catch (const std::exception& e) {
                LOG_ERROR("PerceptionEngine: poll() threw: {}", e.what());
                result.reset();
            }
        }

        if (result.has_value() && result->confidence >= config_.min_confidence) {
            PerceptionEvent ev;
            ev.has_lock = true;
            ev.target = std::move(*result);
            if (ev.target.stamp_us == 0) ev.target.stamp_us = time::nowUs();
            last_lock_time_ = time::nowSteady();
            last_was_lock_ = true;
            bus_->publish(infra::EventType::PERCEPTION_UPDATED, ev);
        } else {
            double since_lock_ms = time::durationS(last_lock_time_, time::nowSteady()) * 1000.0;
            bool stale = since_lock_ms >= config_.lock_timeout_ms;
            if (stale && (config_.publish_misses || last_was_lock_)) {
                publishMiss();
                last_was_lock_ = false;
            }
        }

        next_tick += period;
        auto now = std::chrono::steady_clock::now();
        if (next_tick < now) next_tick = now;
        std::this_thread::sleep_until(next_tick);
    }
}

void PerceptionEngine::publishMiss() {
    PerceptionEvent ev;
    ev.has_lock = false;
    bus_->publish(infra::EventType::PERCEPTION_UPDATED, ev);
}

} // namespace uav::perception
