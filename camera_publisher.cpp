#include <iostream>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>

using namespace std::chrono_literals;

auto STATISTIC_INTERVAL = 3s;
bool is_stopped = false;

void signalHandler(int signum) {
    is_stopped = true;
}

int main() {
    signal(SIGINT, signalHandler); // Register signal handler for Ctrl+C

    // Configure connection constants
    char pub_hostname[1024];
    gethostname(pub_hostname, 1024);
    std::cout << "Hostname: " << pub_hostname << std::endl;

    const char* server_host = "localhost"; // Server hostname
    const int server_port = 5555; // Server port

    // Initialize ZMQ context and socket
    const int zmq_hwm = 10;
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_DEALER);
    socket.setsockopt(ZMQ_SNDHWM, zmq_hwm);
    socket.connect("tcp://" + std::string(server_host) + ":" + std::to_string(server_port));

    // Initialize RealSense camera
    const int resolution_width = 640; // Camera width
    const int resolution_height = 480; // Camera height
    const int decimation_magnitude = 8; // Decimation filter magnitude
    const int camera_fps = 15; // Camera frames per second

    rs2::pipeline camera_pipeline;
    rs2::config config;
    rs2::decimation_filter decimation_filter(decimation_magnitude);
    config.enable_stream(RS2_STREAM_DEPTH, resolution_width, resolution_height, RS2_FORMAT_Z16, camera_fps); // Enable depth stream
    camera_pipeline.start(config);

    std::cout << "Initializing camera with resolution: " << resolution_width << "x" << resolution_height << std::endl;
    std::cout << "Sending pointcloud data of size: " << (resolution_width/decimation_magnitude) * (resolution_height/decimation_magnitude) * 3 * sizeof(float) << " bytes" << std::endl;
    std::cout << "Server address: tcp://" << server_host << ":" << server_port << std::endl;

    // Statistics
    int total_frames_sent = 0;
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!is_stopped)
    {
        // Wait for a new frame
        rs2::frameset frames = camera_pipeline.wait_for_frames(); // Wait for a set of frames from multiple streams (video, motion, pose, etc)
        rs2::depth_frame depth = frames.get_depth_frame(); // Get the depth frame from the frameset

        // Decimate the depth frame
        rs2::frame decimated_depth = decimation_filter.process(depth);
        depth = decimated_depth.as<rs2::depth_frame>();

        int width = depth.get_width(); // Get the width of the depth frame
        int height = depth.get_height(); // Get the height of the depth frame
        int size = width * height;

        // Get the pointcloud data
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);

        const rs2::vertex* vertices = points.get_vertices();
        std::vector<float> pointcloud_data(size * 3);
        memcpy(pointcloud_data.data(), vertices, size * 3 * sizeof(float));

        // Send a multipart message of pub_hostname, frame_width, frame_height, and depth data
        zmq::message_t hostname_message(pub_hostname, strlen(pub_hostname));
        zmq::message_t width_message(&width, sizeof(width));
        zmq::message_t height_message(&height, sizeof(height));
        zmq::message_t pointcloud_message(pointcloud_data.data(), pointcloud_data.size() * sizeof(float));

        // Send the messages as multipart messages
        try {
            socket.send(hostname_message, zmq::send_flags::sndmore);
            socket.send(width_message, zmq::send_flags::sndmore);
            socket.send(height_message, zmq::send_flags::sndmore);
            socket.send(pointcloud_message, zmq::send_flags::none);
        } catch (const zmq::error_t& e) {
            break;
        }

        // Update statistics
        total_frames_sent++;
        frame_count++;
        if (auto now = std::chrono::steady_clock::now(); now - start_time >= STATISTIC_INTERVAL) {
            float fps = frame_count / std::chrono::duration<float>(now - start_time).count();
            frame_count = 0;
            start_time = now;
            std::cout << "FPS: " << fps << std::endl;
        }
    }

    std::cout << "\nTotal frames sent: " << total_frames_sent << std::endl;

    return 0;
}