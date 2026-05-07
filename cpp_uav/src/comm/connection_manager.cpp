#include "uav/comm/connection_manager.hpp"
#include "uav/core/time_utils.hpp"
#include "uav/infra/log_manager.hpp"

// =============================================================================
// Platform socket abstraction
// =============================================================================

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>

    using socket_t = SOCKET;
    static constexpr socket_t INVALID_SOCK = INVALID_SOCKET;
    static constexpr int SOCK_ERR = SOCKET_ERROR;

    static bool g_wsa_initialized = false;
    static void platformSocketInit() {
        if (!g_wsa_initialized) {
            WSADATA wsa;
            WSAStartup(MAKEWORD(2, 2), &wsa);
            g_wsa_initialized = true;
        }
    }
    static void platformCloseSocket(socket_t s) { ::closesocket(s); }
    static int platformGetError() { return WSAGetLastError(); }

    static int platformPoll(socket_t fd, int timeout_ms) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        return ::select(0, &readfds, nullptr, nullptr, &tv);
    }

#else
    #include <arpa/inet.h>
    #include <errno.h>
    #include <fcntl.h>
    #include <netinet/in.h>
    #include <poll.h>
    #include <sys/socket.h>
    #include <unistd.h>

    using socket_t = int;
    static constexpr socket_t INVALID_SOCK = -1;
    static constexpr int SOCK_ERR = -1;

    static void platformSocketInit() {}
    static void platformCloseSocket(socket_t s) { ::close(s); }
    static int platformGetError() { return errno; }

    static int platformPoll(socket_t fd, int timeout_ms) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        return ::poll(&pfd, 1, timeout_ms);
    }
#endif

#include <cstring>
#include <sstream>

namespace uav::comm {

ConnectionManager::ConnectionManager(const std::string& connection_string,
                                     uint8_t system_id, uint8_t component_id)
    : connection_string_(connection_string),
      system_id_(system_id), component_id_(component_id) {}

ConnectionManager::~ConnectionManager() { disconnect(); }

bool ConnectionManager::parseConnectionString() {
    std::istringstream ss(connection_string_);
    std::string protocol;
    std::getline(ss, protocol, ':');

    if (protocol == "udpin") {
        protocol_type_ = ProtocolType::UDP_SERVER;
    } else if (protocol == "udpout") {
        protocol_type_ = ProtocolType::UDP_CLIENT;
    } else if (protocol == "tcpin") {
        protocol_type_ = ProtocolType::TCP_SERVER;
    } else if (protocol == "tcpout") {
        protocol_type_ = ProtocolType::TCP_CLIENT;
    } else {
        LOG_ERROR("ConnectionManager: unknown protocol '{}'", protocol);
        return false;
    }

    std::string rest;
    std::getline(ss, rest);
    while (!rest.empty() && rest[0] == ':') rest.erase(0, 1);

    auto colon = rest.rfind(':');
    if (colon == std::string::npos) {
        LOG_ERROR("ConnectionManager: invalid format '{}'", connection_string_);
        return false;
    }

    host_ = rest.substr(0, colon);
    try {
        port_ = std::stoi(rest.substr(colon + 1));
    } catch (...) {
        LOG_ERROR("ConnectionManager: invalid port in '{}'", connection_string_);
        return false;
    }

    return true;
}

bool ConnectionManager::connect() {
    platformSocketInit();

    if (!parseConnectionString()) return false;
    if (!createSocket()) return false;

    state_ = (protocol_type_ == ProtocolType::TCP_SERVER)
                 ? ConnectionState::CONNECTING
                 : ConnectionState::CONNECTED;
    running_ = true;
    last_data_time_ = std::chrono::steady_clock::now();

    receive_thread_ = std::thread(&ConnectionManager::receiveLoop, this);

    LOG_INFO("ConnectionManager: connected to {}", connection_string_);

    {
        std::lock_guard lock(stats_mutex_);
        stats_.state = state_.load();
    }

    return true;
}

void ConnectionManager::disconnect() {
    running_ = false;
    if (receive_thread_.joinable()) receive_thread_.join();
    closeSocket();
    state_ = ConnectionState::DISCONNECTED;

    std::lock_guard lock(stats_mutex_);
    stats_.state = ConnectionState::DISCONNECTED;
}

bool ConnectionManager::isConnected() const {
    if (state_.load() != ConnectionState::CONNECTED) return false;
    if (protocol_type_ == ProtocolType::TCP_SERVER) {
        return client_socket_.load() != 0;
    }
    return true;
}

bool ConnectionManager::createSocket() {
    int sock_type = (protocol_type_ == ProtocolType::UDP_SERVER ||
                     protocol_type_ == ProtocolType::UDP_CLIENT)
                        ? SOCK_DGRAM : SOCK_STREAM;

    auto fd = ::socket(AF_INET, sock_type, 0);
    if (fd == INVALID_SOCK) {
        LOG_ERROR("ConnectionManager: socket creation failed (err={})", platformGetError());
        return false;
    }
    socket_fd_ = static_cast<uintptr_t>(fd);

    int opt = 1;
    setsockopt(static_cast<socket_t>(socket_fd_), SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&opt), sizeof(opt));

    if (protocol_type_ == ProtocolType::UDP_SERVER ||
        protocol_type_ == ProtocolType::TCP_SERVER) {

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));

        if (host_.empty() || host_ == "0.0.0.0" || host_ == "127.0.0.1") {
            addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
        }

        if (::bind(static_cast<socket_t>(socket_fd_),
                    reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == SOCK_ERR) {
            LOG_ERROR("ConnectionManager: bind failed on port {} (err={})", port_, platformGetError());
            closeSocket();
            return false;
        }

        if (protocol_type_ == ProtocolType::TCP_SERVER) {
            if (::listen(static_cast<socket_t>(socket_fd_), 1) == SOCK_ERR) {
                LOG_ERROR("ConnectionManager: listen failed (err={})", platformGetError());
                closeSocket();
                return false;
            }
        }
    }

    if (protocol_type_ == ProtocolType::UDP_CLIENT ||
        protocol_type_ == ProtocolType::TCP_CLIENT) {

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

        if (protocol_type_ == ProtocolType::TCP_CLIENT) {
            if (::connect(static_cast<socket_t>(socket_fd_),
                          reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == SOCK_ERR) {
                LOG_ERROR("ConnectionManager: TCP connect failed to {}:{} (err={})",
                          host_, port_, platformGetError());
                closeSocket();
                return false;
            }
        } else {
            std::lock_guard lock(remote_addr_mutex_);
            std::memcpy(remote_addr_storage_, &addr, sizeof(addr));
            has_remote_addr_ = true;
        }
    }

    return true;
}

void ConnectionManager::closeSocket() {
    if (socket_fd_ != 0) {
        platformCloseSocket(static_cast<socket_t>(socket_fd_));
        socket_fd_ = 0;
    }
    uintptr_t client = client_socket_.exchange(0);
    if (client != 0) {
        platformCloseSocket(static_cast<socket_t>(client));
    }
}

bool ConnectionManager::sendMessage(const MAVLinkMessage& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg.raw);

    bool ok = false;
    if (protocol_type_ == ProtocolType::UDP_SERVER ||
        protocol_type_ == ProtocolType::UDP_CLIENT) {
        ok = sendUDP(buf, len);
    } else {
        ok = sendTCP(buf, len);
    }

    if (ok) {
        std::lock_guard lock(stats_mutex_);
        stats_.messages_sent++;
        stats_.bytes_sent += len;
    }
    return ok;
}

std::optional<MAVLinkMessage> ConnectionManager::receiveMessage() {
    return std::nullopt;
}

bool ConnectionManager::reconnect() {
    disconnect();
    return connect();
}

bool ConnectionManager::isHealthy(int timeout_ms) const {
    std::lock_guard lock(health_mutex_);
    auto elapsed = std::chrono::steady_clock::now() - last_data_time_;
    return elapsed < std::chrono::milliseconds(timeout_ms);
}

ConnectionStats ConnectionManager::getStats() const {
    std::lock_guard lock(stats_mutex_);
    return stats_;
}

// =============================================================================
// Send helpers (cross-platform)
// =============================================================================

bool ConnectionManager::sendUDP(const uint8_t* data, size_t len) {
    struct sockaddr_in addr;
    {
        std::lock_guard lock(remote_addr_mutex_);
        if (!has_remote_addr_) return false;
        std::memcpy(&addr, remote_addr_storage_, sizeof(addr));
    }

    std::lock_guard send_lock(send_mutex_);
    auto sent = ::sendto(static_cast<socket_t>(socket_fd_),
                         reinterpret_cast<const char*>(data),
                         static_cast<int>(len), 0,
                         reinterpret_cast<struct sockaddr*>(&addr),
                         static_cast<socklen_t>(sizeof(addr)));
    return sent > 0;
}

bool ConnectionManager::sendTCP(const uint8_t* data, size_t len) {
    uintptr_t client = client_socket_.load();
    auto fd = (client != 0) ? client : socket_fd_;
    if (fd == 0) return false;
    if (protocol_type_ == ProtocolType::TCP_SERVER && client == 0) return false;

    // Serialize concurrent senders so MAVLink frames are not interleaved.
    std::lock_guard lock(send_mutex_);
    auto sent = ::send(static_cast<socket_t>(fd),
                       reinterpret_cast<const char*>(data),
                       static_cast<int>(len), 0);
    return sent > 0;
}

int ConnectionManager::receiveUDP(uint8_t* buffer, size_t max_len) {
    struct sockaddr_in sender_addr{};
    socklen_t addr_len = sizeof(sender_addr);

    auto n = ::recvfrom(static_cast<socket_t>(socket_fd_),
                        reinterpret_cast<char*>(buffer),
                        static_cast<int>(max_len), 0,
                        reinterpret_cast<struct sockaddr*>(&sender_addr), &addr_len);

    if (n > 0) {
        std::lock_guard lock(remote_addr_mutex_);
        std::memcpy(remote_addr_storage_, &sender_addr, sizeof(sender_addr));
        has_remote_addr_ = true;
    }

    return static_cast<int>(n);
}

int ConnectionManager::receiveTCP(uint8_t* buffer, size_t max_len) {
    if (protocol_type_ == ProtocolType::TCP_SERVER && client_socket_.load() == 0) {
        int poll_result = platformPoll(static_cast<socket_t>(socket_fd_), 100);
        if (poll_result <= 0) return -1;

        struct sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        auto client = ::accept(static_cast<socket_t>(socket_fd_),
                               reinterpret_cast<struct sockaddr*>(&client_addr), &addr_len);
        if (client == INVALID_SOCK) return -1;
        client_socket_ = static_cast<uintptr_t>(client);
        state_ = ConnectionState::CONNECTED;
        {
            std::lock_guard lock(stats_mutex_);
            stats_.state = ConnectionState::CONNECTED;
        }
        LOG_INFO("ConnectionManager: TCP client connected");
    }

    uintptr_t client = client_socket_.load();
    auto fd = (client != 0) ? client : socket_fd_;
    if (protocol_type_ == ProtocolType::TCP_SERVER && client == 0) return -1;

    auto n = ::recv(static_cast<socket_t>(fd),
                    reinterpret_cast<char*>(buffer),
                    static_cast<int>(max_len), 0);

    if (n <= 0 && client != 0) {
        LOG_WARN("ConnectionManager: TCP client disconnected");
        uintptr_t closed = client_socket_.exchange(0);
        if (closed != 0) {
            platformCloseSocket(static_cast<socket_t>(closed));
        }
        if (running_.load() && protocol_type_ == ProtocolType::TCP_SERVER) {
            state_ = ConnectionState::CONNECTING;
            std::lock_guard lock(stats_mutex_);
            stats_.state = ConnectionState::CONNECTING;
        }
    }

    return static_cast<int>(n);
}

// =============================================================================
// Receive loop (cross-platform with poll/select timeout for clean shutdown)
// =============================================================================

void ConnectionManager::receiveLoop() {
    uint8_t buffer[2048];
    mavlink_message_t msg{};
    mavlink_status_t status{};

    while (running_) {
        socket_t poll_fd;
        uintptr_t client = client_socket_.load();
        if (protocol_type_ == ProtocolType::TCP_SERVER && client != 0) {
            poll_fd = static_cast<socket_t>(client);
        } else {
            poll_fd = static_cast<socket_t>(socket_fd_);
        }

        int sel = platformPoll(poll_fd, 100);
        if (sel <= 0) continue;

        int n = 0;
        if (protocol_type_ == ProtocolType::UDP_SERVER ||
            protocol_type_ == ProtocolType::UDP_CLIENT) {
            n = receiveUDP(buffer, sizeof(buffer));
        } else {
            n = receiveTCP(buffer, sizeof(buffer));
        }

        if (n <= 0) continue;

        {
            std::lock_guard lock(health_mutex_);
            last_data_time_ = std::chrono::steady_clock::now();
        }

        // Account bytes once per receive call; messages_received is per parsed frame.
        {
            std::lock_guard lock(stats_mutex_);
            stats_.bytes_received += static_cast<uint64_t>(n);
        }

        for (int i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                MAVLinkMessage wrapped;
                wrapped.raw = msg;
                wrapped.received_at_us = uav::time::nowUs();

                {
                    std::lock_guard lock(stats_mutex_);
                    stats_.messages_received++;
                }

                if (message_callback_) {
                    message_callback_(wrapped);
                }
            }
        }
    }
}

} // namespace uav::comm
