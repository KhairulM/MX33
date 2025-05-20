#include <iostream>
#include <chrono>
#include <csignal>
#include <mutex>
#include <unistd.h>
#include <atomic>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>

#include "publisher.hpp"
#include "msgs/pointcloud.hpp"

using namespace std::chrono_literals;

std::mutex mutex;
std::atomic<bool> is_stopped(false);

void signalHandler(int signum) {
    is_stopped = true;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler); // Register signal handler for Ctrl+C

    if (argc == 1) {
        argv[1] = "pointcloud";
        argv[2] = "localhost";
        argv[3] = "5555";
    }
    
    if (argc == 2) {
        argv[2] = "localhost";
        argv[3] = "5555";
    }

    if (argc == 3) {
        argv[3] = "5555";
    }

    // Configure connection constants
    char pub_hostname[1024];
    gethostname(pub_hostname, 1024);
    std::cout << "Hostname: " << pub_hostname << std::endl;

    const char* server_host = argv[2]; // Server hostname
    const int server_port = atoi(argv[3]); // Server port

    // Create a publisher
    Publisher<Pointcloud> pointcloud_publisher(
        "PointcloudPublisher", 
        "tcp://" + std::string(server_host) + ":" + std::to_string(server_port),
        std::string(argv[1])
    );

    // Configure RealSense camera
    const int resolution_width = 640; // Camera width
    const int resolution_height = 480; // Camera height
    const int decimation_magnitude = 8; // Decimation filter magnitude
    const int camera_fps = 15; // Camera frames per second

    rs2::pipeline camera_pipeline;
    rs2::config config;
    rs2::decimation_filter decimation_filter(decimation_magnitude);
    config.enable_stream(RS2_STREAM_DEPTH, resolution_width, resolution_height, RS2_FORMAT_Z16, camera_fps); // Enable depth stream
    
    std::cout << "Initializing camera with resolution: " << resolution_width << "x" << resolution_height << std::endl;
    std::cout << "Sending pointcloud data of size: " <<  (resolution_width/decimation_magnitude) << "x" << (resolution_height/decimation_magnitude) << ", " << (resolution_width/decimation_magnitude) * (resolution_height/decimation_magnitude) * 3 * sizeof(float) << " bytes" << std::endl;
    std::cout << "Server address: tcp://" << server_host << ":" << server_port << std::endl;
    
    // Statistics
    const auto statistic_interval = 3s;
    int total_frames_sent = 0;
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    // Realsense callback function
    auto cameraCallback = [&](const rs2::frame& frame) {
        // Check if stopped
        if (is_stopped) return;
        
        // Check if it's a synchronized frame or not
        rs2::frameset frameset = frame.as<rs2::frameset>();
        if (!frameset) return;
        
        // Check if depth frame is available
        rs2::depth_frame depth_frame = frameset.get_depth_frame();
        if (!depth_frame) return;
        
        // Decimate the depth frame
        rs2::frame decimated_depth = decimation_filter.process(depth_frame);
        depth_frame = decimated_depth.as<rs2::depth_frame>();
        
        int width = depth_frame.get_width();
        int height = depth_frame.get_height();
        int size = width * height;
        
        // Get the pointcloud data
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth_frame);
        
        // Flatten the pointcloud data
        const rs2::vertex* vertices = points.get_vertices();
        std::vector<float> pointcloud_data(size * 3);
        memcpy(pointcloud_data.data(), vertices, size * 3 * sizeof(float));
        
        // Serialize the message with msgpack
        Pointcloud message;
        message.hostname = pub_hostname;
        message.width = width;
        message.height = height;
        message.pointcloud_data = pointcloud_data;

        // Publish the messages
        pointcloud_publisher.publish(message);

        // Update statistics
        total_frames_sent++;
        frame_count++;
    };
    
    camera_pipeline.start(config, cameraCallback);

    while (!is_stopped) {
        // Update statistics
        auto now = std::chrono::steady_clock::now();
        if (now - start_time >= statistic_interval) {
            std::lock_guard<std::mutex> lock(mutex);
            
            float fps = (float)frame_count / std::chrono::duration<float>(now - start_time).count();
            std::cout << "FPS: " << fps << std::endl;

            frame_count = 0;
            start_time = now;
        }
    }

    camera_pipeline.stop();

    std::cout << "\nTotal frames sent: " << total_frames_sent << std::endl;

    return 0;
}