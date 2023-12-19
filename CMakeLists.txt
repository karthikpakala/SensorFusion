cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall -pthread") ## -w disable all diagnostics
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

project(SensorFusion CXX)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MACOS)
    set(PCL_DIR /opt/homebrew/opt/pcl/include)
    set(OpenCV_DIR /opt/homebrew/opt/opencv/include)
elseif(LINUX)
    find_package(CUDA REQUIRED)
    include_directories($(CUDA_PTX_COMPILATION ON))
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


add_executable (SensorFusion src/Camera.cpp src/Lidar.cpp src/Radar.cpp src/KalmanFilter.cpp src/Tools.cpp src/Calibration.cpp include/Camera.h include/Lidar.h include/Radar.h include/KalmanFilter.h src/main.cpp include/Tools.h include/Calibration.h)

target_link_libraries(SensorFusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} z ssl)
