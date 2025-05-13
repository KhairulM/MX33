#include <iostream>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>
// #include <opencv2/opencv.hpp>

int main() {
    // Configure connection constants
    char pub_hostname[1024];
    gethostname(pub_hostname, 1024);
    std::cout << "Hostname: " << pub_hostname << std::endl;

    const char* server_hostname = "localhost"; // Server hostname
    const int server_port = 5555; // Server port

    // Initialize ZMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_DEALER);
    socket.connect("tcp://" + std::string(server_hostname) + ":" + std::to_string(server_port));

    // Initialize RealSense camera
    rs2::pipeline camera_pipeline;
    camera_pipeline.start();

    rs2::frameset frames = camera_pipeline.wait_for_frames(); // Wait for a set of frames from multiple streams (video, motion, pose, etc)
    rs2::depth_frame depth = frames.get_depth_frame(); // Get the depth frame from the frameset

    const int width = depth.get_width(); // Get the width of the depth frame
    const int height = depth.get_height(); // Get the height of the depth frame
    const int size = width * height;

    std::cout << "Initializing camera with resolution: " << width << "x" << height << std::endl;
    std::cout << "Sending pointcloud data of size: " << size * 3 * sizeof(float) << " bytes" << std::endl;
    std::cout << "Server address: tcp://" << server_hostname << ":" << server_port << std::endl;

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

        // Get the pointcloud data
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);

        const rs2::vertex* vertices = points.get_vertices();
        std::vector<float> pointcloud_data(size * 3);
        for (int i = 0; i < size; ++i) {
            pointcloud_data[i * 3] = vertices[i].x;
            pointcloud_data[i * 3 + 1] = vertices[i].y;
            pointcloud_data[i * 3 + 2] = vertices[i].z;
        }

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

        // Wait for the next frame
        frames = camera_pipeline.wait_for_frames();
        depth = frames.get_depth_frame();
    }

    return 0;
}