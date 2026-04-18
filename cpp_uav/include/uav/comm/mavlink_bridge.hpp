#pragma once

#include "uav/comm/connection_manager.hpp"
#include "uav/comm/flight_commander.hpp"
#include "uav/comm/param_reader.hpp"
#include "uav/comm/statustext_parser.hpp"
#include "uav/infra/thread_pool.hpp"
#include "uav/infra/event_bus.hpp"
#include "uav/world_model/world_model.hpp"

#include <atomic>
#include <memory>
#include <thread>

namespace uav::comm {

class MAVLinkBridge {
public:
    struct Stats {
        uint64_t messages_received = 0;
        uint64_t messages_sent = 0;
        uint64_t last_heartbeat_us = 0;
        bool pixhawk_connected = false;
        bool gcs_connected = false;
        bool heartbeat_timeout = false;
        bool armed = false;
        std::string flight_mode;
    };

    MAVLinkBridge(const std::string& pixhawk_conn,
                  const std::string& gcs_conn,
                  std::shared_ptr<infra::ThreadPool> thread_pool,
                  std::shared_ptr<world_model::WorldModel> world_model,
                  std::shared_ptr<infra::EventBus> event_bus);
    ~MAVLinkBridge();

    bool start();
    void stop();
    bool isRunning() const { return running_; }

    std::shared_ptr<FlightCommander> getFlightCommander() { return flight_commander_; }
    std::shared_ptr<ParamReader> getParamReader() { return param_reader_; }
    std::shared_ptr<ConnectionManager> getPixhawkConnection() { return pixhawk_; }
    std::shared_ptr<ConnectionManager> getGCSConnection() { return gcs_; }
    Stats getStats() const;

private:
    std::shared_ptr<ConnectionManager> pixhawk_;
    std::shared_ptr<ConnectionManager> gcs_;
    std::shared_ptr<FlightCommander> flight_commander_;
    std::shared_ptr<ParamReader> param_reader_;
    StatusTextParser statustext_parser_;

    std::shared_ptr<infra::ThreadPool> thread_pool_;
    std::shared_ptr<world_model::WorldModel> world_model_;
    std::shared_ptr<infra::EventBus> event_bus_;

    std::atomic<bool> running_{false};
    std::thread heartbeat_thread_;

    static constexpr int HEARTBEAT_TIMEOUT_MS = 5000;
    std::atomic<bool> heartbeat_timeout_{false};

    mutable std::recursive_mutex stats_mutex_;
    Stats stats_;

    void handlePixhawkMessage(const MAVLinkMessage& msg);
    void handleGCSMessage(const MAVLinkMessage& msg);
    void handleHeartbeat(const MAVLinkMessage& msg);
    void handleStatusText(const MAVLinkMessage& msg);
    void handleCommandAck(const MAVLinkMessage& msg);
    void dispatchModeCommand(const ParsedStatusText& parsed);
    void heartbeatLoop();
};

} // namespace uav::comm
