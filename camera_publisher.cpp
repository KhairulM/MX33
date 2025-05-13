#include <iostream>
#include <chrono>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>
// #include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

int main() {
    // Configure connection constants
    char pub_hostname[1024];
    gethostname(pub_hostname, 1024);
    std::cout << "Hostname: " << pub_hostname << std::endl;

    const char* server_host = "192.168.0.7"; // Server hostname
    const int server_port = 5555; // Server port

    // Initialize ZMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_DEALER);
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
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
        // Displaying the depth frame
        // cv::Mat depth_frame(height, width, CV_16U, const_cast<void*>(depth.get_data()));
        // cv::Mat depth_frame_8u;
        // depth_frame.convertTo(depth_frame_8u, CV_8U, 255.0 / 3000); // assuming max depth of 3 meters
        // cv::Mat depth_colormap;
        // cv::applyColorMap(depth_frame_8u, depth_colormap, cv::COLORMAP_JET);
        // cv::imshow("Published Depth Frame", depth_colormap); // Display the depth frame
        // cv::waitKey(1); // Wait for a short period to allow the window to update

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
        socket.send(hostname_message, zmq::send_flags::sndmore);
        socket.send(width_message, zmq::send_flags::sndmore);
        socket.send(height_message, zmq::send_flags::sndmore);
        socket.send(pointcloud_message, zmq::send_flags::none);

        // Update statistics
        frame_count++;
        if (std::chrono::steady_clock::now() - start_time >= 5s) {
            float fps = frame_count / 5.0;
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
            std::cout << "FPS over 5s: " << fps << std::endl;
        }
    }

    return 0;
}