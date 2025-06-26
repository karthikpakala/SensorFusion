#ifndef PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_H

#include "IPlugin.h"
#include <dlfcn.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <json/json.h>

namespace Perception {
namespace PluginSystem {

// Plugin load result
struct PluginLoadResult {
    bool success;
    std::string errorMessage;
    std::shared_ptr<IPlugin> plugin;
};

// Plugin registry entry
struct PluginRegistryEntry {
    PluginMetadata metadata;
    void* libraryHandle;
    std::shared_ptr<IPlugin> pluginInstance;
    CreatePluginFunc createFunc;
    DestroyPluginFunc destroyFunc;
    bool isLoaded;
    bool isActive;
};

// Event system for plugin lifecycle
enum class PluginEvent {
    LOADED,
    UNLOADED,
    STARTED,
    STOPPED,
    ERROR
};

using PluginEventCallback = std::function<void(const std::string& pluginId, PluginEvent event, const std::string& message)>;

class PluginManager {
public:
    static PluginManager& getInstance();
    
    // Plugin discovery and loading
    bool scanForPlugins(const std::string& pluginDirectory);
    PluginLoadResult loadPlugin(const std::string& pluginPath);
    bool unloadPlugin(const std::string& pluginId);
    bool reloadPlugin(const std::string& pluginId);
    
    // Plugin lifecycle management
    bool startPlugin(const std::string& pluginId);
    bool stopPlugin(const std::string& pluginId);
    bool configurePlugin(const std::string& pluginId, const std::map<std::string, std::string>& config);
    
    // Plugin registry access
    std::vector<std::string> getAvailablePlugins() const;
    std::vector<std::string> getLoadedPlugins() const;
    std::vector<std::string> getActivePlugins() const;
    std::vector<std::string> getPluginsByType(PluginType type) const;
    
    // Plugin information
    std::shared_ptr<IPlugin> getPlugin(const std::string& pluginId);
    PluginMetadata getPluginMetadata(const std::string& pluginId) const;
    bool isPluginLoaded(const std::string& pluginId) const;
    bool isPluginActive(const std::string& pluginId) const;
    
    // Dependency management
    bool checkDependencies(const std::string& pluginId) const;
    std::vector<std::string> getMissingDependencies(const std::string& pluginId) const;
    bool resolveDependencies(const std::string& pluginId);
    
    // Configuration management
    bool loadConfiguration(const std::string& configFile);
    bool saveConfiguration(const std::string& configFile) const;
    bool setGlobalConfiguration(const std::map<std::string, std::string>& config);
    std::map<std::string, std::string> getGlobalConfiguration() const;
    
    // Event system
    void registerEventCallback(PluginEventCallback callback);
    void unregisterEventCallback(PluginEventCallback callback);
    
    // Plugin validation
    bool validatePlugin(const std::string& pluginPath, std::string& errorMessage);
    bool verifyPluginSignature(const std::string& pluginPath);
    
    // Performance monitoring
    std::map<std::string, std::map<std::string, double>> getPluginMetrics() const;
    bool startPerformanceMonitoring();
    bool stopPerformanceMonitoring();
    
    // Hot-swapping
    bool swapPlugin(const std::string& oldPluginId, const std::string& newPluginId);
    bool canSwapPlugin(const std::string& pluginId) const;
    
    // Cleanup
    void shutdown();

private:
    PluginManager() = default;
    ~PluginManager();
    
    // Internal methods
    bool loadPluginFromPath(const std::string& pluginPath, PluginRegistryEntry& entry);
    void unloadPluginLibrary(PluginRegistryEntry& entry);
    bool parsePluginManifest(const std::string& manifestPath, PluginMetadata& metadata);
    void notifyEvent(const std::string& pluginId, PluginEvent event, const std::string& message);
    void monitorPluginHealth();
    
    // Member variables
    std::unordered_map<std::string, PluginRegistryEntry> m_plugins;
    std::vector<PluginEventCallback> m_eventCallbacks;
    std::map<std::string, std::string> m_globalConfig;
    
    // Threading
    mutable std::mutex m_pluginsMutex;
    mutable std::mutex m_callbacksMutex;
    std::thread m_monitoringThread;
    std::condition_variable m_monitoringCondition;
    bool m_isMonitoring;
    bool m_shutdown;
    
    // Constants
    static constexpr const char* PLUGIN_MANIFEST_FILENAME = "plugin.json";
    static constexpr const char* PLUGIN_CREATE_FUNCTION_NAME = "createPlugin";
    static constexpr const char* PLUGIN_DESTROY_FUNCTION_NAME = "destroyPlugin";
};

// Helper class for scoped plugin operations
class ScopedPluginOperation {
public:
    ScopedPluginOperation(const std::string& pluginId, const std::string& operation);
    ~ScopedPluginOperation();
    
    bool isSuccessful() const { return m_success; }
    void setSuccess(bool success) { m_success = success; }
    
private:
    std::string m_pluginId;
    std::string m_operation;
    bool m_success;
    std::chrono::steady_clock::time_point m_startTime;
};

// Plugin configuration helper
class PluginConfigBuilder {
public:
    PluginConfigBuilder& addParameter(const std::string& key, const std::string& value);
    PluginConfigBuilder& addParameter(const std::string& key, double value);
    PluginConfigBuilder& addParameter(const std::string& key, int value);
    PluginConfigBuilder& addParameter(const std::string& key, bool value);
    
    std::map<std::string, std::string> build() const;
    
private:
    std::map<std::string, std::string> m_config;
};

} // namespace PluginSystem
} // namespace Perception

#endif // PLUGIN_MANAGER_H
