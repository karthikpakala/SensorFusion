#include "PluginBasedPerception.h"
#include "EnhancedSensor.h"
#include <iostream>
#include <memory>

// Example plugin implementation for object detection
class YOLODetectorPlugin : public Perception::PluginSystem::IProcessorPlugin {
public:
    YOLODetectorPlugin() {
        m_metadata.name = "YOLO Object Detector";
        m_metadata.id = "yolo_detector";
        m_metadata.type = Perception::PluginSystem::PluginType::PROCESSOR;
        m_metadata.capabilities.version = "1.0.0";
        m_metadata.capabilities.author = "SensorFusion Team";
        m_metadata.capabilities.description = "YOLO-based object detection plugin";
        m_metadata.capabilities.supportedDataTypes = {"CameraImageStruct"};
        m_metadata.capabilities.supportsRealTime = true;
        m_metadata.capabilities.supportsGPU = true;
    }
    
    bool initialize(const std::map<std::string, std::string>& config) override {
        // Load YOLO model configuration
        auto modelPath = config.find("model_path");
        auto weightsPath = config.find("weights_path");
        auto configPath = config.find("config_path");
        
        if (modelPath != config.end() && weightsPath != config.end() && configPath != config.end()) {
            // Initialize YOLO network
            m_net = cv::dnn::readNetFromDarknet(configPath->second, weightsPath->second);
            
            // Enable GPU if available
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            }
            
            m_isInitialized = true;
            return true;
        }
        return false;
    }
    
    std::shared_ptr<void> process(std::shared_ptr<void> inputData, 
                                const std::string& inputType) override {
        if (!m_isInitialized || inputType != "CameraImageStruct") {
            return nullptr;
        }
        
        auto imageStruct = std::static_pointer_cast<Perception::DataStructure::CameraImageStruct>(inputData);
        if (!imageStruct) {
            return nullptr;
        }
        
        // Perform YOLO detection
        auto detectedObjects = detectObjects(imageStruct->image);
        
        // Create result structure
        auto result = std::make_shared<Perception::DataStructure::CameraImageStruct>(*imageStruct);
        result->boundingBoxes = detectedObjects;
        
        return result;
    }
    
    // ... implement other required methods
    
private:
    cv::dnn::Net m_net;
    bool m_isInitialized = false;
    
    std::vector<Perception::DataStructure::BoundingBox> detectObjects(const cv::Mat& image) {
        std::vector<Perception::DataStructure::BoundingBox> detections;
        
        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
        m_net.setInput(blob);
        
        // Run inference
        std::vector<cv::Mat> outs;
        m_net.forward(outs, m_net.getUnconnectedOutLayersNames());
        
        // Process detections
        float confThreshold = 0.5;
        float nmsThreshold = 0.4;
        
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        
        for (size_t i = 0; i < outs.size(); ++i) {
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                
                if (confidence > confThreshold) {
                    int centerX = (int)(data[0] * image.cols);
                    int centerY = (int)(data[1] * image.rows);
                    int width = (int)(data[2] * image.cols);
                    int height = (int)(data[3] * image.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }
        
        // Apply NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        
        // Convert to BoundingBox format
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            Perception::DataStructure::BoundingBox box;
            box.boxID = static_cast<int>(i);
            box.roi = boxes[idx];
            box.classID = classIds[idx];
            box.confidence = confidences[idx];
            detections.push_back(box);
        }
        
        return detections;
    }
};

// Example usage demonstration
void demonstratePluginArchitecture() {
    using namespace Perception::PluginSystem;
    
    std::cout << "=== Plugin Architecture Demonstration ===" << std::endl;
    
    // 1. Create the plugin-based perception system
    auto perception = std::make_unique<PluginBasedPerception>();
    
    // 2. Build configuration using the builder pattern
    auto config = PerceptionConfigBuilder()
        .addSensor("./plugins/camera_sensor.so", "camera_0")
        .addSensor("./plugins/lidar_sensor.so", "lidar_0")
        .addProcessor("./plugins/yolo_detector.so", "object_detector")
        .addFusionAlgorithm("./plugins/kalman_fusion.so", "kalman_filter")
        .addOutput("./plugins/visualization.so", "visualizer")
        .connectPlugins("camera_0", "object_detector")
        .connectPlugins("lidar_0", "kalman_filter")
        .connectPlugins("object_detector", "kalman_filter")
        .connectPlugins("kalman_filter", "visualizer")
        .setFrameRate(30.0)
        .setParallelProcessing(true, 4)
        .configurePlugin("camera_0", {
            {"device_id", "0"},
            {"width", "1920"},
            {"height", "1080"},
            {"fps", "30"}
        })
        .configurePlugin("lidar_0", {
            {"data_path", "/path/to/lidar/data"},
            {"min_range", "0.1"},
            {"max_range", "100.0"}
        })
        .configurePlugin("object_detector", {
            {"model_path", "./model/yolo/yolov3.weights"},
            {"config_path", "./model/yolo/yolov3.cfg"},
            {"weights_path", "./model/yolo/yolov3.weights"},
            {"confidence_threshold", "0.5"}
        })
        .build();
    
    // 3. Initialize and configure the system
    if (!perception->initialize("config.json")) {
        std::cerr << "Failed to initialize perception system" << std::endl;
        return;
    }
    
    if (!perception->configurePipeline(config)) {
        std::cerr << "Failed to configure pipeline" << std::endl;
        return;
    }
    
    // 4. Set up result callback for real-time processing
    perception->setResultCallback([](std::shared_ptr<FusionResult> result) {
        std::cout << "New fusion result: " << result->detectedObjects.size() 
                  << " objects detected at timestamp " << result->timestamp << std::endl;
        
        // Process the result (visualization, logging, etc.)
        for (const auto& obj : result->detectedObjects) {
            std::cout << "  Object " << obj.boxID << ": class=" << obj.classID 
                      << ", confidence=" << obj.confidence << std::endl;
        }
    });
    
    // 5. Start processing
    if (!perception->start()) {
        std::cerr << "Failed to start perception system" << std::endl;
        return;
    }
    
    if (!perception->startProcessing()) {
        std::cerr << "Failed to start processing" << std::endl;
        return;
    }
    
    std::cout << "Perception system started successfully!" << std::endl;
    std::cout << "Active plugins: ";
    for (const auto& plugin : perception->getLoadedPlugins()) {
        std::cout << plugin << " ";
    }
    std::cout << std::endl;
    
    // 6. Monitor performance
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    auto metrics = perception->getPerformanceMetrics();
    std::cout << "\nPerformance Metrics:" << std::endl;
    std::cout << "  Average frame rate: " << metrics.averageFrameRate << " fps" << std::endl;
    std::cout << "  Average latency: " << metrics.averageLatency << " ms" << std::endl;
    std::cout << "  Total frames processed: " << metrics.totalFramesProcessed << std::endl;
    std::cout << "  Dropped frames: " << metrics.droppedFrames << std::endl;
    
    // 7. Demonstrate hot-swapping
    std::cout << "\nDemonstrating hot-swapping..." << std::endl;
    if (perception->swapProcessorPlugin("object_detector", "new_object_detector")) {
        std::cout << "Successfully swapped object detector plugin" << std::endl;
    }
    
    // 8. System health check
    if (perception->isHealthy()) {
        std::cout << "System is healthy" << std::endl;
    } else {
        std::cout << "System health issues detected:" << std::endl;
        for (const auto& warning : perception->getActiveWarnings()) {
            std::cout << "  WARNING: " << warning << std::endl;
        }
        for (const auto& error : perception->getErrors()) {
            std::cout << "  ERROR: " << error << std::endl;
        }
    }
    
    // 9. Cleanup
    std::cout << "\nShutting down..." << std::endl;
    perception->stopProcessing();
    perception->stop();
    perception->shutdown();
    
    std::cout << "Plugin architecture demonstration completed." << std::endl;
}

int main() {
    try {
        demonstratePluginArchitecture();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
