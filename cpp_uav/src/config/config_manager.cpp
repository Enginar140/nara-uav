#include "uav/config/config_manager.hpp"

#include <fstream>
#include <sstream>

namespace uav::config {

ConfigManager::ConfigManager(const std::string& config_path)
    : path_(config_path) {
    try {
        root_ = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        throw ConfigException("Failed to load config '" + config_path + "': " + e.what());
    }
}

YAML::Node ConfigManager::navigateToKey(const std::string& dotpath) const {
    YAML::Node node = YAML::Clone(root_);
    std::istringstream stream(dotpath);
    std::string segment;

    while (std::getline(stream, segment, '.')) {
        if (!node.IsMap() || !node[segment]) {
            return YAML::Node();
        }
        node = node[segment];
    }
    return node;
}

std::string ConfigManager::getString(const std::string& key,
                                     const std::string& def) const {
    auto node = navigateToKey(key);
    if (node && node.IsScalar()) {
        return node.as<std::string>();
    }
    return def;
}

int ConfigManager::getInt(const std::string& key, int def) const {
    auto node = navigateToKey(key);
    if (node && node.IsScalar()) {
        try { return node.as<int>(); }
        catch (...) { return def; }
    }
    return def;
}

double ConfigManager::getDouble(const std::string& key, double def) const {
    auto node = navigateToKey(key);
    if (node && node.IsScalar()) {
        try { return node.as<double>(); }
        catch (...) { return def; }
    }
    return def;
}

bool ConfigManager::getBool(const std::string& key, bool def) const {
    auto node = navigateToKey(key);
    if (node && node.IsScalar()) {
        try { return node.as<bool>(); }
        catch (...) { return def; }
    }
    return def;
}

bool ConfigManager::hasKey(const std::string& key) const {
    auto node = navigateToKey(key);
    return node && !node.IsNull();
}

YAML::Node ConfigManager::getNode(const std::string& key) const {
    return navigateToKey(key);
}

} // namespace uav::config
