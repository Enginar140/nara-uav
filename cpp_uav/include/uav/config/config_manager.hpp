#pragma once

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <string>

namespace uav::config {

class ConfigException : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class ConfigManager {
public:
    explicit ConfigManager(const std::string& config_path);

    std::string getString(const std::string& key, const std::string& def = "") const;
    int getInt(const std::string& key, int def = 0) const;
    double getDouble(const std::string& key, double def = 0.0) const;
    bool getBool(const std::string& key, bool def = false) const;
    bool hasKey(const std::string& key) const;
    YAML::Node getNode(const std::string& key) const;

    std::string getPixhawkConnection() const {
        return getString("mavlink.pixhawk.connection", "udpin:127.0.0.1:14550");
    }
    std::string getGCSConnection() const {
        return getString("mavlink.gcs.connection", "tcpin:127.0.0.1:14552");
    }

private:
    YAML::Node root_;
    std::string path_;

    YAML::Node navigateToKey(const std::string& dotpath) const;
};

} // namespace uav::config
