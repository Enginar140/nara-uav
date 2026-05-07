#pragma once

#include "uav/perception/iperception_module.hpp"
#include "uav/perception/perception_types.hpp"
#include "uav/infra/event_bus.hpp"
#include "uav/world_model/world_model.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace uav::perception {

/// Owns a registry of pluggable IPerceptionModule implementations and a single
/// active module. Periodically polls the active module and publishes
/// PerceptionEvent payloads on the EventBus.
///
/// At most one module is active at a time. setActive("") disables perception
/// entirely (engine still ticks but emits has_lock=false events at low rate so
/// downstream consumers can fall back deterministically).
class PerceptionEngine {
public:
    struct Config {
        double tick_rate_hz = 10.0;
        float min_confidence = 0.4f;     // PerceptionTargets below this drop to has_lock=false
        double lock_timeout_ms = 1000.0; // emit has_lock=false if module yields nothing for this long
        bool publish_misses = true;      // also publish has_lock=false on miss (lets modes fall back)
    };

    PerceptionEngine(std::shared_ptr<infra::EventBus> bus,
                     std::shared_ptr<world_model::WorldModel> world_model,
                     Config config = {});
    ~PerceptionEngine();

    PerceptionEngine(const PerceptionEngine&) = delete;
    PerceptionEngine& operator=(const PerceptionEngine&) = delete;

    /// Register a module instance. Engine takes shared ownership. Safe to call
    /// before start().
    void registerModule(std::shared_ptr<IPerceptionModule> module);

    /// Activate a previously registered module by id. Empty string disables.
    /// Returns false if the id is unknown.
    bool setActive(const std::string& id);

    std::string activeId() const;

    void start();
    void stop();
    bool isRunning() const { return running_; }

    /// Returns ids of all registered modules (for diagnostics / yaml validation).
    std::vector<std::string> registered() const;

private:
    void tickLoop();
    void publishMiss();

    std::shared_ptr<infra::EventBus> bus_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    Config config_;

    std::unordered_map<std::string, std::shared_ptr<IPerceptionModule>> modules_;
    std::shared_ptr<IPerceptionModule> active_;
    std::string active_id_;
    mutable std::mutex active_mutex_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    std::chrono::steady_clock::time_point last_lock_time_;
    std::chrono::steady_clock::time_point last_miss_time_;
    bool last_was_lock_ = false;
};

} // namespace uav::perception
