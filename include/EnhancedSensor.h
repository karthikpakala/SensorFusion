#ifndef ENHANCED_SENSOR_H
#define ENHANCED_SENSOR_H

#include "IPlugin.h"
#include "data_structure.h"
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

namespace Perception {
namespace PluginSystem {

// Sensor data wrapper
template<typename T>
class SensorData {
public:
    SensorData(const T& data, double timestamp, const std::string& sensorId)
        : m_data(data), m_timestamp(timestamp), m_sensorId(sensorId), 
          m_sequenceNumber(s_sequenceCounter++) {}
    
    const T& getData() const { return m_data; }
    double getTimestamp() const { return m_timestamp; }
    const std::string& getSensorId() const { return m_sensorId; }
    uint64_t getSequenceNumber() const { return m_sequenceNumber; }
    
    bool isValid() const { return m_timestamp > 0 && !m_sensorId.empty(); }
    
private:
    T m_data;
    double m_timestamp;
    std::string m_sensorId;
    uint64_t m_sequenceNumber;
    static std::atomic<uint64_t> s_sequenceCounter;
};

template<typename T>
std::atomic<uint64_t> SensorData<T>::s_sequenceCounter{0};

// Enhanced sensor base class
class EnhancedSensor : public ISensorPlugin {
public:
    EnhancedSensor(const std::string& sensorId, const std::string& sensorType);
    virtual ~EnhancedSensor();
    
    // ISensorPlugin implementation
    bool initialize(const std::map<std::string, std::string>& config) override;
    bool start() override;
    bool stop() override;
    void shutdown() override;
    
    bool startDataAcquisition() override;
    bool stopDataAcquisition() override;
    bool hasNewData() const override;
    std::shared_ptr<void> getData() override;
    
    bool isActive() const override { return m_isActive; }
    std::string getStatus() const override;
    std::string getName() const override { return m_metadata.name; }
    
    // Enhanced sensor features
    virtual bool selfTest() = 0;
    virtual bool performCalibration() = 0;
    virtual std::map<std::string, double> getDiagnostics() const = 0;
    
    // Data queue management
    void setQueueSize(size_t size) { m_maxQueueSize = size; }
    size_t getQueueSize() const { return m_dataQueue.size(); }
    void clearQueue();
    
    // Synchronization
    void enableTimestampSync(bool enable) { m_timestampSyncEnabled = enable; }
    void setMasterClock(std::shared_ptr<std::chrono::steady_clock> clock) { m_masterClock = clock; }
    
    // Statistics
    struct Statistics {
        uint64_t totalFramesProcessed = 0;
        uint64_t droppedFrames = 0;
        double averageProcessingTime = 0.0;
        double dataRate = 0.0;
        std::chrono::steady_clock::time_point lastUpdateTime;
    };
    
    Statistics getStatistics() const { return m_statistics; }
    void resetStatistics();

protected:
    // Template method pattern for sensor-specific implementation
    virtual bool doInitialize(const std::map<std::string, std::string>& config) = 0;
    virtual bool doStart() = 0;
    virtual bool doStop() = 0;
    virtual void doShutdown() = 0;
    
    // Data acquisition methods to be implemented by derived classes
    virtual bool doStartDataAcquisition() = 0;
    virtual bool doStopDataAcquisition() = 0;
    virtual std::shared_ptr<void> doGetData() = 0;
    
    // Utility methods for derived classes
    void updateStatistics();
    double getCurrentTimestamp() const;
    void pushToQueue(std::shared_ptr<void> data);
    
    // Configuration helpers
    template<typename T>
    T getConfigValue(const std::map<std::string, std::string>& config, 
                    const std::string& key, const T& defaultValue) const;
    
    // Member variables
    std::string m_sensorId;
    std::string m_sensorType;
    PluginMetadata m_metadata;
    std::atomic<bool> m_isActive{false};
    std::atomic<bool> m_isAcquiring{false};
    
    // Data management
    std::queue<std::shared_ptr<void>> m_dataQueue;
    mutable std::mutex m_queueMutex;
    std::condition_variable m_dataAvailable;
    size_t m_maxQueueSize = 100;
    
    // Timing and synchronization
    bool m_timestampSyncEnabled = false;
    std::shared_ptr<std::chrono::steady_clock> m_masterClock;
    std::chrono::steady_clock::time_point m_startTime;
    
    // Statistics
    mutable std::mutex m_statsMutex;
    Statistics m_statistics;
    
    // Configuration
    std::map<std::string, std::string> m_config;
};

// Camera sensor plugin
class CameraSensorPlugin : public EnhancedSensor {
public:
    CameraSensorPlugin();
    ~CameraSensorPlugin() override = default;
    
    // Plugin metadata
    PluginMetadata getMetadata() const override;
    std::string getVersion() const override { return "1.0.0"; }
    std::string getDataType() const override { return "CameraImageStruct"; }
    
    // Sensor-specific methods
    bool selfTest() override;
    bool performCalibration() override;
    std::map<std::string, double> getDiagnostics() const override;
    
    // Camera-specific functionality
    bool setResolution(int width, int height);
    bool setFrameRate(double fps);
    bool setExposure(double exposure);
    bool setGain(double gain);
    
    std::pair<int, int> getResolution() const { return {m_width, m_height}; }
    double getFrameRate() const { return m_frameRate; }

protected:
    bool doInitialize(const std::map<std::string, std::string>& config) override;
    bool doStart() override;
    bool doStop() override;
    void doShutdown() override;
    
    bool doStartDataAcquisition() override;
    bool doStopDataAcquisition() override;
    std::shared_ptr<void> doGetData() override;

private:
    // Camera-specific members
    int m_cameraIndex = 0;
    int m_width = 640;
    int m_height = 480;
    double m_frameRate = 30.0;
    double m_exposure = -1; // Auto exposure
    double m_gain = -1;     // Auto gain
    
    // OpenCV camera capture
    std::unique_ptr<cv::VideoCapture> m_capture;
    
    // Threading for continuous capture
    std::thread m_captureThread;
    std::atomic<bool> m_shouldCapture{false};
    
    void captureLoop();
};

// LiDAR sensor plugin
class LidarSensorPlugin : public EnhancedSensor {
public:
    LidarSensorPlugin();
    ~LidarSensorPlugin() override = default;
    
    // Plugin metadata
    PluginMetadata getMetadata() const override;
    std::string getVersion() const override { return "1.0.0"; }
    std::string getDataType() const override { return "PointCloud"; }
    
    // Sensor-specific methods
    bool selfTest() override;
    bool performCalibration() override;
    std::map<std::string, double> getDiagnostics() const override;
    
    // LiDAR-specific functionality
    bool setVerticalFOV(double minAngle, double maxAngle);
    bool setHorizontalFOV(double minAngle, double maxAngle);
    bool setRangeFilter(double minRange, double maxRange);
    bool setIntensityFilter(double minIntensity, double maxIntensity);

protected:
    bool doInitialize(const std::map<std::string, std::string>& config) override;
    bool doStart() override;
    bool doStop() override;
    void doShutdown() override;
    
    bool doStartDataAcquisition() override;
    bool doStopDataAcquisition() override;
    std::shared_ptr<void> doGetData() override;

private:
    // LiDAR-specific configuration
    double m_verticalFOVMin = -15.0;
    double m_verticalFOVMax = 15.0;
    double m_horizontalFOVMin = -180.0;
    double m_horizontalFOVMax = 180.0;
    double m_minRange = 0.1;
    double m_maxRange = 100.0;
    double m_minIntensity = 0.0;
    double m_maxIntensity = 255.0;
    
    // Data source (file-based for simulation)
    std::string m_dataPath;
    std::vector<std::string> m_pointCloudFiles;
    size_t m_currentFileIndex = 0;
    
    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(const std::string& filename);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};

} // namespace PluginSystem
} // namespace Perception

// Plugin declarations
DECLARE_PLUGIN(Perception::PluginSystem::CameraSensorPlugin)
DECLARE_PLUGIN(Perception::PluginSystem::LidarSensorPlugin)

#endif // ENHANCED_SENSOR_H
