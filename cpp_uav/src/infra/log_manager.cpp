#include "uav/infra/log_manager.hpp"

#include <filesystem>
#include <iostream>

namespace uav::infra {

LogManager& LogManager::instance() {
    static LogManager inst;
    return inst;
}

void LogManager::initialize(const std::string& log_dir,
                            const std::string& level,
                            int max_size_mb,
                            int backup_count) {
    if (initialized_) return;

    try {
        std::filesystem::create_directories(log_dir);

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_dir + "/uav.log",
            static_cast<size_t>(max_size_mb) * 1024 * 1024,
            backup_count);

        auto logger = std::make_shared<spdlog::logger>(
            "uav", spdlog::sinks_init_list{console_sink, file_sink});

        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");

        if (level == "debug")      logger->set_level(spdlog::level::debug);
        else if (level == "warn")  logger->set_level(spdlog::level::warn);
        else if (level == "error") logger->set_level(spdlog::level::err);
        else                       logger->set_level(spdlog::level::info);

        spdlog::set_default_logger(logger);
        spdlog::flush_every(std::chrono::seconds(3));

        initialized_ = true;

    } catch (const spdlog::spdlog_ex& e) {
        std::cerr << "Log initialization failed: " << e.what() << std::endl;
    }
}

void LogManager::flush() {
    spdlog::default_logger()->flush();
}

} // namespace uav::infra
