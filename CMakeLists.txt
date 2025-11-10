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

# NLohhmann JSON
# find_package(nlohmann_json 3.12.0 REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/msgs
    ${CMAKE_CURRENT_SOURCE_DIR}/srvs
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/msgpack/include
)

# Add executables
option(BUILD_EXAMPLES "Build example applications" ON)
if(BUILD_EXAMPLES)
    add_executable(camera_publisher demo/camera_publisher.cpp)
    add_executable(camera_subscriber demo/camera_subscriber.cpp)
    add_executable(demo_service_server demo/demo_service_server.cpp)
    add_executable(demo_service_client demo/demo_service_client.cpp)
    
    # Link libraries
    target_link_libraries(
        camera_publisher
        realsense2
        cppzmq
    )
    target_link_libraries(
        camera_subscriber
        cppzmq
        ${PCL_LIBRARIES}
    )

    target_link_libraries(
        demo_service_server
        cppzmq
    )

    target_link_libraries(
        demo_service_client
        cppzmq
    )

endif()

add_executable(broker broker.cpp)
target_link_libraries(
    broker
    cppzmq
)

add_executable(map_server map_server.cpp)
target_link_libraries(
    map_server
    cppzmq
    ${PCL_LIBRARIES}
)

add_executable(robot robot.cpp)
target_link_libraries(
    robot
    cppzmq
    ${PCL_LIBRARIES}
)