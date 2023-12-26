cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

add_definitions(-std=c++17)

##set(CXX_FLAGS "-Wall -lpthread") ## -w disable all diagnostics
##set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)


project(SensorFusion CXX)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MACOS)
    set(PCL_DIR /opt/homebrew/opt/pcl/include)
    set(OpenCV_DIR /opt/homebrew/opt/opencv/include)
elseif(LINUX)
    find_package(CUDA REQUIRED)
    set(PCL_DIR /usr/local/share/pcl-1.13/PCLConfig.cmake)
    set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.13/pcl)
    set(PCL_LIBRARIES /usr/local/lib)
    include_directories($(CUDA_PTX_COMPILATION ON))
endif()

find_package(PCL 1.3 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


add_executable (SensorFusion src/Camera.cpp src/Lidar.cpp src/Radar.cpp src/KalmanFilter.cpp src/Tools.cpp src/Calibration.cpp include/Camera.h include/Lidar.h include/Radar.h include/KalmanFilter.h src/main.cpp src/main.cu include/Tools.h include/Calibration.h)

target_link_libraries(SensorFusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} z ssl)
