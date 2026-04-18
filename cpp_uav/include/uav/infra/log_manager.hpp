#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>
#include <string>

namespace uav::infra {

class LogManager {
public:
    static LogManager& instance();

    void initialize(const std::string& log_dir = "logs",
                    const std::string& level = "info",
                    int max_size_mb = 10,
                    int backup_count = 5);

    void flush();
    bool isInitialized() const { return initialized_; }

private:
    LogManager() = default;
    bool initialized_ = false;
};

} // namespace uav::infra

#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_INFO(...)  SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARN(...)  SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
