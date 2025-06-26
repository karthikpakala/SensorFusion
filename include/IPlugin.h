#ifndef IPLUGIN_H
#define IPLUGIN_H

#include <string>
#include <memory>
#include <map>
#include <vector>
#include "data_structure.h"

namespace Perception {
namespace PluginSystem {

// Plugin types
enum class PluginType {
    SENSOR,           // Camera, LiDAR, Radar, IMU
    PROCESSOR,        // Object detection, tracking, filtering
    FUSION_ALGORITHM, // Different fusion strategies
    OUTPUT           // Visualization, logging, storage
};

// Plugin capabilities
struct PluginCapabilities {
    std::vector<std::string> supportedDataTypes;
    std::vector<std::string> requiredDependencies;
    std::string version;
    std::string author;
    std::string description;
    bool supportsRealTime;
    bool supportsGPU;
};

// Plugin metadata
struct PluginMetadata {
    std::string name;
    std::string id;
    PluginType type;
    PluginCapabilities capabilities;
    std::string filePath;
};

// Base plugin interface
class IPlugin {
public:
    virtual ~IPlugin() = default;
    
    // Core plugin lifecycle
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual void shutdown() = 0;
    
    // Plugin information
    virtual PluginMetadata getMetadata() const = 0;
    virtual std::string getVersion() const = 0;
    virtual std::string getName() const = 0;
    virtual PluginType getType() const = 0;
    
    // Status and health
    virtual bool isActive() const = 0;
    virtual std::string getStatus() const = 0;
    virtual std::map<std::string, std::string> getHealthMetrics() const = 0;
    
    // Configuration
    virtual bool configure(const std::map<std::string, std::string>& config) = 0;
    virtual std::map<std::string, std::string> getConfiguration() const = 0;
};

// Sensor plugin interface
class ISensorPlugin : public IPlugin {
public:
    virtual ~ISensorPlugin() = default;
    
    // Data acquisition
    virtual bool startDataAcquisition() = 0;
    virtual bool stopDataAcquisition() = 0;
    virtual bool hasNewData() const = 0;
    
    // Data retrieval - generic data container
    virtual std::shared_ptr<void> getData() = 0;
    virtual std::string getDataType() const = 0;
    
    // Sensor-specific
    virtual bool calibrate() = 0;
    virtual bool isCalibrated() const = 0;
    virtual double getDataRate() const = 0; // Hz
    virtual void setDataRate(double rate) = 0;
    
    PluginType getType() const override { return PluginType::SENSOR; }
};

// Processor plugin interface
class IProcessorPlugin : public IPlugin {
public:
    virtual ~IProcessorPlugin() = default;
    
    // Data processing
    virtual std::shared_ptr<void> process(std::shared_ptr<void> inputData, 
                                        const std::string& inputType) = 0;
    virtual std::vector<std::string> getSupportedInputTypes() const = 0;
    virtual std::string getOutputType() const = 0;
    
    // Processing parameters
    virtual bool setProcessingParameters(const std::map<std::string, double>& params) = 0;
    virtual std::map<std::string, double> getProcessingParameters() const = 0;
    
    // Performance metrics
    virtual double getProcessingTime() const = 0; // milliseconds
    virtual double getThroughput() const = 0; // data/second
    
    PluginType getType() const override { return PluginType::PROCESSOR; }
};

// Fusion algorithm plugin interface
class IFusionPlugin : public IPlugin {
public:
    virtual ~IFusionPlugin() = default;
    
    // Multi-sensor fusion
    virtual std::shared_ptr<void> fuse(const std::vector<std::shared_ptr<void>>& sensorData,
                                     const std::vector<std::string>& dataTypes,
                                     double timestamp) = 0;
    
    // Sensor registration
    virtual bool registerSensorType(const std::string& sensorType, double weight) = 0;
    virtual bool unregisterSensorType(const std::string& sensorType) = 0;
    virtual std::vector<std::string> getRegisteredSensorTypes() const = 0;
    
    // Fusion parameters
    virtual bool setFusionParameters(const std::map<std::string, double>& params) = 0;
    virtual std::map<std::string, double> getFusionParameters() const = 0;
    
    // Quality metrics
    virtual double getFusionQuality() const = 0;
    virtual std::map<std::string, double> getSensorContributions() const = 0;
    
    PluginType getType() const override { return PluginType::FUSION_ALGORITHM; }
};

// Output plugin interface
class IOutputPlugin : public IPlugin {
public:
    virtual ~IOutputPlugin() = default;
    
    // Data output
    virtual bool output(std::shared_ptr<void> data, const std::string& dataType) = 0;
    virtual bool flush() = 0;
    
    // Output configuration
    virtual bool setOutputFormat(const std::string& format) = 0;
    virtual std::string getOutputFormat() const = 0;
    virtual bool setOutputDestination(const std::string& destination) = 0;
    virtual std::string getOutputDestination() const = 0;
    
    PluginType getType() const override { return PluginType::OUTPUT; }
};

} // namespace PluginSystem
} // namespace Perception

// Plugin factory function type
extern "C" {
    typedef Perception::PluginSystem::IPlugin* (*CreatePluginFunc)();
    typedef void (*DestroyPluginFunc)(Perception::PluginSystem::IPlugin*);
}

// Macros for plugin implementation
#define DECLARE_PLUGIN(ClassName) \
    extern "C" { \
        Perception::PluginSystem::IPlugin* createPlugin() { \
            return new ClassName(); \
        } \
        void destroyPlugin(Perception::PluginSystem::IPlugin* plugin) { \
            delete plugin; \
        } \
    }

#endif // IPLUGIN_H
