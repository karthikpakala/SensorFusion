cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

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

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

##add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/ransac.cpp src/cluster_3d.cpp src/kdtree.h)
add_executable (SensorFusion src/Camera.cpp src/Lidar.cpp src/Radar.cpp src/KalmanFilter.cpp src/Tools.cpp src/Calibration.cpp include/Camera.h include/Lidar.h include/Radar.h include/KalmanFilter.h src/main.cpp include/Tools.h include/Calibration.h )

target_link_libraries(SensorFusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} z ssl uv uWS)
