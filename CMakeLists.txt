cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(SensorFusion)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

##add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/ransac.cpp src/cluster_3d.cpp src/kdtree.h)
add_executable (SensorFusion src/main.cpp src/Camera.cpp src/Lidar.cpp src/Radar.cpp src/KalmanFilter.cpp include/Camera.h include/Lidar.h include/Radar.h include/KalmanFilter.h)

target_link_libraries(SensorFusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} z ssl uv uWS)
