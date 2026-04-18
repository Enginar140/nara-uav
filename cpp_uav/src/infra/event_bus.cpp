#include "uav/infra/event_bus.hpp"
#include "uav/core/time_utils.hpp"

namespace uav::infra {

Event::Event(EventType t, std::any p)
    : type(t), payload(std::move(p)), timestamp_us(uav::time::nowUs()) {}

EventBus::EventBus(size_t max_queue) : max_queue_(max_queue) {}

EventBus::~EventBus() { stop(); }

uint64_t EventBus::subscribe(EventType type, EventHandler handler) {
    std::lock_guard lock(subscribers_mutex_);
    uint64_t id = next_id_++;
    subscribers_[type].push_back({id, type, std::move(handler)});
    return id;
}

void EventBus::unsubscribe(uint64_t id) {
    std::lock_guard lock(subscribers_mutex_);
    for (auto& [type, subs] : subscribers_) {
        subs.erase(
            std::remove_if(subs.begin(), subs.end(),
                           [id](const Subscription& s) { return s.id == id; }),
            subs.end());
    }
}

void EventBus::publish(EventType type, std::any payload) {
    std::lock_guard lock(queue_mutex_);
    if (queue_.size() < max_queue_) {
        queue_.emplace(type, std::move(payload));
        queue_cv_.notify_one();
    }
}

void EventBus::start() {
    if (running_) return;
    running_ = true;
    dispatch_thread_ = std::thread(&EventBus::dispatchLoop, this);
}

void EventBus::stop() {
    if (!running_) return;
    running_ = false;
    queue_cv_.notify_all();
    if (dispatch_thread_.joinable()) {
        dispatch_thread_.join();
    }
}

void EventBus::dispatchLoop() {
    while (running_) {
        Event event(EventType::SYSTEM_STARTED);
        {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait_for(lock, std::chrono::milliseconds(50),
                               [this] { return !queue_.empty() || !running_; });
            if (!running_ && queue_.empty()) break;
            if (queue_.empty()) continue;
            event = std::move(queue_.front());
            queue_.pop();
        }
        dispatchEvent(event);
    }
}

void EventBus::dispatchEvent(const Event& event) {
    std::lock_guard lock(subscribers_mutex_);
    auto it = subscribers_.find(event.type);
    if (it == subscribers_.end()) return;

    for (const auto& sub : it->second) {
        try {
            sub.handler(event);
        } catch (...) {
            // Swallow handler exceptions to protect dispatch loop
        }
    }
}

std::string toString(EventType type) {
    switch (type) {
        case EventType::SAFETY_VIOLATION:     return "SAFETY_VIOLATION";
        case EventType::BOUNDARY_VIOLATION:   return "BOUNDARY_VIOLATION";
        case EventType::HSS_VIOLATION:        return "HSS_VIOLATION";
        case EventType::COLLISION_WARNING:    return "COLLISION_WARNING";
        case EventType::OPPONENT_UPDATED:     return "OPPONENT_UPDATED";
        case EventType::HSS_ZONE_UPDATED:     return "HSS_ZONE_UPDATED";
        case EventType::BOUNDARY_UPDATED:     return "BOUNDARY_UPDATED";
        case EventType::OWN_STATE_UPDATED:    return "OWN_STATE_UPDATED";
        case EventType::MODE_CHANGED:         return "MODE_CHANGED";
        case EventType::MODE_COMMAND_RECEIVED:return "MODE_COMMAND_RECEIVED";
        case EventType::MAVLINK_CONNECTED:    return "MAVLINK_CONNECTED";
        case EventType::MAVLINK_DISCONNECTED: return "MAVLINK_DISCONNECTED";
        case EventType::HEARTBEAT_TIMEOUT:    return "HEARTBEAT_TIMEOUT";
        case EventType::HEARTBEAT_RECOVERED:  return "HEARTBEAT_RECOVERED";
        case EventType::GPS_QUALITY_LOW:      return "GPS_QUALITY_LOW";
        case EventType::GPS_QUALITY_CRITICAL: return "GPS_QUALITY_CRITICAL";
        case EventType::BATTERY_LOW:          return "BATTERY_LOW";
        case EventType::BATTERY_CRITICAL:     return "BATTERY_CRITICAL";
        case EventType::SYSTEM_STARTED:       return "SYSTEM_STARTED";
        case EventType::SYSTEM_STOPPING:      return "SYSTEM_STOPPING";
    }
    return "UNKNOWN";
}

} // namespace uav::infra
