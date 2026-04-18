#include "uav/app/application.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    std::string config_path;

    if (argc > 1) {
        config_path = argv[1];
    } else {
        auto exe_dir = std::filesystem::path(argv[0]).parent_path();
        auto candidates = {
            exe_dir / "config" / "default.yaml",
            exe_dir / ".." / "config" / "default.yaml",
            std::filesystem::path("config/default.yaml")
        };
        for (const auto& c : candidates) {
            if (std::filesystem::exists(c)) {
                config_path = c.string();
                break;
            }
        }
    }

    if (config_path.empty()) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        std::cerr << "No config file found in default locations.\n";
        return 1;
    }

    uav::app::Application::setupSignalHandlers();
    uav::app::Application app(config_path);

    if (!app.initialize()) {
        std::cerr << "Initialization failed\n";
        return 1;
    }

    if (!app.start()) {
        std::cerr << "Start failed\n";
        return 1;
    }

    app.run();
    app.stop();
    return 0;
}
