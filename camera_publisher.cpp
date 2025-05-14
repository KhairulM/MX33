#include <iostream>
#include <chrono>
#include <csignal>
#include <mutex>
#include <unistd.h>

#include <atomic>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>

using namespace std::chrono_literals;

std::mutex mutex;
std::atomic<bool> is_stopped(false);

void signalHandler(int signum) {
    is_stopped = true;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler); // Register signal handler for Ctrl+C

    if (argc != 3) {
        argv[1] = "localhost";
        argv[2] = "5555";
    }

    // Configure connection constants
    char pub_hostname[1024];
    gethostname(pub_hostname, 1024);
    std::cout << "Hostname: " << pub_hostname << std::endl;

    const char* server_host = argv[1]; // Server hostname
    const int server_port = atoi(argv[2]); // Server port

    // Initialize ZMQ context and socket
    const int zmq_hwm = 45;
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);
    socket.setsockopt(ZMQ_SNDHWM, zmq_hwm);
    socket.connect("tcp://" + std::string(server_host) + ":" + std::to_string(server_port));

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
        
        // Send a multipart message of pub_hostname, frame_width, frame_height, and depth data
        zmq::message_t hostname_message(pub_hostname, strlen(pub_hostname));
        zmq::message_t width_message(&width, sizeof(width));
        zmq::message_t height_message(&height, sizeof(height));
        zmq::message_t pointcloud_message(pointcloud_data.data(), pointcloud_data.size() * sizeof(float));
        
        // Send the messages as multipart messages
        auto send_flags = zmq::send_flags::sndmore | zmq::send_flags::dontwait;
        socket.send(zmq::str_buffer("pointcloud"), send_flags);
        socket.send(hostname_message, send_flags);
        socket.send(width_message, send_flags);
        socket.send(height_message, send_flags);
        socket.send(pointcloud_message, zmq::send_flags::dontwait);

        // Update statistics
        total_frames_sent++;

        mutex.lock();
        frame_count++;
        mutex.unlock();
    };
    
    camera_pipeline.start(config, cameraCallback);

    while (!is_stopped) {
        // Update statistics
        auto now = std::chrono::steady_clock::now();
        if (now - start_time >= statistic_interval) {
            float fps = frame_count / std::chrono::duration<float>(now - start_time).count();
            std::cout << "FPS: " << fps << std::endl;

            mutex.lock();
            frame_count = 0;
            start_time = now;
            mutex.unlock();
        }
    }

    camera_pipeline.stop();
    socket.close();

    std::cout << "\nTotal frames sent: " << total_frames_sent << std::endl;

    return 0;
}