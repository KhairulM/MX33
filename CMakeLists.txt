cmake_minimum_required(VERSION 3.10)
project(MX33)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
# Intel realsense SDK
find_package(realsense2 REQUIRED)

# ZeroMQ (cppzmq)
find_package(cppzmq REQUIRED)

# PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
#find_package(OpenCV REQUIRED)

# Add executables
add_executable(camera_publisher camera_publisher.cpp)
add_executable(camera_subscriber camera_subscriber.cpp)

# Link libraries
target_link_libraries(
    camera_publisher
    realsense2
    cppzmq
    #${OpenCV_LIBS}
)

target_link_libraries(
    camera_subscriber
    realsense2
    cppzmq
    #${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)