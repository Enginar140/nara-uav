#pragma once

#include "uav/comm/mavlink_message.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace uav::comm {

enum class ConnectionState { DISCONNECTED, CONNECTING, CONNECTED, CONN_ERROR };

struct ConnectionStats {
    uint64_t messages_sent = 0;
    uint64_t messages_received = 0;
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
    uint64_t last_heartbeat_us = 0;
    ConnectionState state = ConnectionState::DISCONNECTED;
};

enum class ProtocolType { UDP_SERVER, UDP_CLIENT, TCP_SERVER, TCP_CLIENT, SERIAL };

class ConnectionManager {
public:
    ConnectionManager(const std::string& connection_string,
                      uint8_t system_id = 1, uint8_t component_id = 1);
    ~ConnectionManager();

    ConnectionManager(const ConnectionManager&) = delete;
    ConnectionManager& operator=(const ConnectionManager&) = delete;

    bool connect();
    void disconnect();
    bool sendMessage(const MAVLinkMessage& msg);
    std::optional<MAVLinkMessage> receiveMessage();
    bool isConnected() const { return state_ == ConnectionState::CONNECTED; }
    ConnectionState getState() const { return state_.load(); }
    ConnectionStats getStats() const;
    bool reconnect();
    bool isHealthy(int timeout_ms = 10000) const;

    void setMessageCallback(std::function<void(const MAVLinkMessage&)> cb) {
        message_callback_ = std::move(cb);
    }

    ProtocolType getProtocolType() const { return protocol_type_; }

private:
    std::string connection_string_;
    uint8_t system_id_, component_id_;

    std::atomic<ConnectionState> state_{ConnectionState::DISCONNECTED};
    ConnectionStats stats_;
    mutable std::recursive_mutex stats_mutex_;

    std::thread receive_thread_;
    std::atomic<bool> running_{false};

    std::function<void(const MAVLinkMessage&)> message_callback_;

    uintptr_t socket_fd_ = 0;
    uintptr_t client_socket_ = 0;

    ProtocolType protocol_type_ = ProtocolType::UDP_SERVER;
    std::string host_;
    int port_ = 0;

    uint8_t remote_addr_storage_[32]{};
    std::atomic<bool> has_remote_addr_{false};
    std::recursive_mutex remote_addr_mutex_;

    std::chrono::steady_clock::time_point last_data_time_;
    mutable std::mutex health_mutex_;

    bool parseConnectionString();
    bool createSocket();
    void closeSocket();
    void receiveLoop();

    bool sendUDP(const uint8_t* data, size_t len);
    bool sendTCP(const uint8_t* data, size_t len);
    int receiveUDP(uint8_t* buffer, size_t max_len);
    int receiveTCP(uint8_t* buffer, size_t max_len);
};

} // namespace uav::comm
