#pragma once

#include <any>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace uav::infra {

enum class EventType {
    SAFETY_VIOLATION,
    BOUNDARY_VIOLATION,
    HSS_VIOLATION,
    COLLISION_WARNING,

    OPPONENT_UPDATED,
    HSS_ZONE_UPDATED,
    BOUNDARY_UPDATED,
    OWN_STATE_UPDATED,

    MODE_CHANGED,
    MODE_COMMAND_RECEIVED,

    MAVLINK_CONNECTED,
    MAVLINK_DISCONNECTED,
    HEARTBEAT_TIMEOUT,
    HEARTBEAT_RECOVERED,
    HEARTBEAT_RECEIVED,

    GPS_QUALITY_LOW,
    GPS_QUALITY_CRITICAL,
    BATTERY_LOW,
    BATTERY_CRITICAL,

    PARAMS_READY,

    SYSTEM_STARTED,
    SYSTEM_STOPPING
};

struct Event {
    EventType type;
    std::any payload;
    uint64_t timestamp_us;

    explicit Event(EventType t, std::any p = {});

    template <typename T>
    T getPayload() const { return std::any_cast<T>(payload); }

    template <typename T>
    bool hasPayload() const { return payload.type() == typeid(T); }
};

using EventHandler = std::function<void(const Event&)>;

class EventBus {
public:
    explicit EventBus(size_t max_queue = 1000);
    ~EventBus();

    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    uint64_t subscribe(EventType type, EventHandler handler);
    void unsubscribe(uint64_t id);
    void publish(EventType type, std::any payload = {});
    void start();
    void stop();
    bool isRunning() const { return running_; }

private:
    struct Subscription {
        uint64_t id;
        EventType type;
        EventHandler handler;
    };

    std::unordered_map<EventType, std::vector<Subscription>> subscribers_;
    mutable std::recursive_mutex subscribers_mutex_;

    std::queue<Event> queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::thread dispatch_thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> next_id_{1};
    size_t max_queue_;

    void dispatchLoop();
    void dispatchEvent(const Event& event);
};

std::string toString(EventType type);

} // namespace uav::infra
