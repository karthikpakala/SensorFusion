#ifndef PLUGIN_BASED_PERCEPTION_H
#define PLUGIN_BASED_PERCEPTION_H

#include "PluginManager.h"
#include "IPlugin.h"
#include "data_structure.h"
#include <memory>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace Perception {
namespace PluginSystem {

// Data flow pipeline
struct DataFlowNode {
    std::string pluginId;
    PluginType pluginType;
    std::vector<std::string> inputPlugins;
    std::vector<std::string> outputPlugins;
    bool isActive = false;
};

// Processing pipeline configuration
struct PipelineConfiguration {
    std::vector<DataFlowNode> nodes;
    std::map<std::string, std::map<std::string, std::string>> pluginConfigs;
    double targetFrameRate = 30.0;
    bool enableParallelProcessing = true;
    int maxThreads = 4;
};

// Fusion result
struct FusionResult {
    std::vector<DataStructure::BoundingBox> detectedObjects;
    pcl::PointCloud<pcl::PointXYZI> fusedPointCloud;
    cv::Mat annotatedImage;
    double timestamp;
    double confidence;
    std::map<std::string, double> sensorContributions;
};

class PluginBasedPerception {
public:
    PluginBasedPerception();
    ~PluginBasedPerception();
    
    // System lifecycle
    bool initialize(const std::string& configFile);
    bool start();
    bool stop();
    void shutdown();
    
    // Plugin management
    bool loadSensorPlugin(const std::string& pluginPath, const std::string& sensorId);
    bool loadProcessorPlugin(const std::string& pluginPath, const std::string& processorId);
    bool loadFusionPlugin(const std::string& pluginPath, const std::string& fusionId);
    
    bool unloadPlugin(const std::string& pluginId);
    std::vector<std::string> getLoadedPlugins() const;
    
    // Pipeline configuration
    bool configurePipeline(const PipelineConfiguration& config);
    bool loadPipelineFromFile(const std::string& configFile);
    bool savePipelineToFile(const std::string& configFile) const;
    
    // Processing control
    bool startProcessing();
    bool stopProcessing();
    bool pauseProcessing();
    bool resumeProcessing();
    
    // Data access
    bool hasNewResults() const;
    std::shared_ptr<FusionResult> getLatestResult();
    std::vector<std::shared_ptr<FusionResult>> getAllResults();
    void clearResults();
    
    // Real-time streaming
    using ResultCallback = std::function<void(std::shared_ptr<FusionResult>)>;
    void setResultCallback(ResultCallback callback);
    void removeResultCallback();
    
    // Performance monitoring
    struct PerformanceMetrics {
        double averageFrameRate = 0.0;
        double averageLatency = 0.0;
        std::map<std::string, double> pluginPerformance;
        uint64_t totalFramesProcessed = 0;
        uint64_t droppedFrames = 0;
    };
    
    PerformanceMetrics getPerformanceMetrics() const;
    void resetPerformanceMetrics();
    
    // System health
    bool isHealthy() const;
    std::map<std::string, std::string> getSystemStatus() const;
    std::vector<std::string> getActiveWarnings() const;
    std::vector<std::string> getErrors() const;
    
    // Hot-swapping capabilities
    bool swapSensorPlugin(const std::string& oldSensorId, const std::string& newSensorId);
    bool swapProcessorPlugin(const std::string& oldProcessorId, const std::string& newProcessorId);
    bool swapFusionPlugin(const std::string& oldFusionId, const std::string& newFusionId);
    
    // Advanced features
    bool enablePlugin(const std::string& pluginId);
    bool disablePlugin(const std::string& pluginId);
    bool isPluginEnabled(const std::string& pluginId) const;
    
    // Sensor synchronization
    bool enableSensorSync(bool enable);
    bool setSyncTolerance(double toleranceMs);
    double getSyncTolerance() const { return m_syncTolerance; }
    
    // Data recording and playback
    bool startRecording(const std::string& filename);
    bool stopRecording();
    bool startPlayback(const std::string& filename);
    bool stopPlayback();
    bool isRecording() const { return m_isRecording; }
    bool isPlayback() const { return m_isPlayback; }

private:
    // Internal processing methods
    void processingLoop();
    void synchronizeData();
    std::shared_ptr<FusionResult> processFrame();
    
    bool validatePipeline(const PipelineConfiguration& config) const;
    bool buildDataFlowGraph();
    void executePipeline();
    
    // Data management
    void collectSensorData();
    bool waitForSyncData();
    void updatePerformanceMetrics();
    
    // Error handling
    void handlePluginError(const std::string& pluginId, const std::string& error);
    void logWarning(const std::string& message);
    void logError(const std::string& message);
    
    // Member variables
    std::unique_ptr<PluginManager> m_pluginManager;
    PipelineConfiguration m_pipelineConfig;
    
    // Processing state
    std::atomic<bool> m_isInitialized{false};
    std::atomic<bool> m_isProcessing{false};
    std::atomic<bool> m_isPaused{false};
    std::atomic<bool> m_shouldShutdown{false};
    
    // Threading
    std::thread m_processingThread;
    std::mutex m_resultsMutex;
    std::condition_variable m_dataAvailable;
    
    // Data storage
    std::vector<std::shared_ptr<FusionResult>> m_results;
    size_t m_maxResults = 100;
    
    // Sensor data synchronization
    struct SensorDataEntry {
        std::string sensorId;
        std::shared_ptr<void> data;
        double timestamp;
        std::string dataType;
    };
    
    std::vector<SensorDataEntry> m_sensorDataBuffer;
    double m_syncTolerance = 10.0; // milliseconds
    bool m_syncEnabled = true;
    
    // Callbacks
    ResultCallback m_resultCallback;
    
    // Performance tracking
    mutable std::mutex m_metricsMutex;
    PerformanceMetrics m_metrics;
    std::chrono::steady_clock::time_point m_lastFrameTime;
    
    // Recording and playback
    std::atomic<bool> m_isRecording{false};
    std::atomic<bool> m_isPlayback{false};
    std::string m_recordingFile;
    std::string m_playbackFile;
    
    // Health monitoring
    std::vector<std::string> m_warnings;
    std::vector<std::string> m_errors;
    mutable std::mutex m_healthMutex;
    
    // Plugin registry for active plugins
    std::map<std::string, std::shared_ptr<ISensorPlugin>> m_sensorPlugins;
    std::map<std::string, std::shared_ptr<IProcessorPlugin>> m_processorPlugins;
    std::map<std::string, std::shared_ptr<IFusionPlugin>> m_fusionPlugins;
    std::map<std::string, std::shared_ptr<IOutputPlugin>> m_outputPlugins;
};

// Configuration helper class
class PerceptionConfigBuilder {
public:
    PerceptionConfigBuilder& addSensor(const std::string& pluginPath, const std::string& sensorId);
    PerceptionConfigBuilder& addProcessor(const std::string& pluginPath, const std::string& processorId);
    PerceptionConfigBuilder& addFusionAlgorithm(const std::string& pluginPath, const std::string& fusionId);
    PerceptionConfigBuilder& addOutput(const std::string& pluginPath, const std::string& outputId);
    
    PerceptionConfigBuilder& connectPlugins(const std::string& sourceId, const std::string& targetId);
    PerceptionConfigBuilder& setFrameRate(double fps);
    PerceptionConfigBuilder& setParallelProcessing(bool enabled, int maxThreads = 4);
    
    PerceptionConfigBuilder& configurePlugin(const std::string& pluginId, 
                                           const std::map<std::string, std::string>& config);
    
    PipelineConfiguration build() const;
    bool saveToFile(const std::string& filename) const;
    
private:
    PipelineConfiguration m_config;
    std::map<std::string, std::string> m_pluginPaths;
};

} // namespace PluginSystem
} // namespace Perception

#endif // PLUGIN_BASED_PERCEPTION_H
