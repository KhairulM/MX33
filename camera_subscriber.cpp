#include <iostream>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <zmq.hpp>

int main() {
    // Initialize ZMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_ROUTER);
    socket.bind("tcp://*:5555");

    std::cout << "Subscriber started, waiting for messages..." << std::endl;

    while (true) {
        zmq::message_t routing_id, hostname_message, width_message, height_message, depth_message;

        // Receive multipart message
        socket.recv(&routing_id); // Receive the routing ID first
        socket.recv(&hostname_message);
        socket.recv(&width_message);
        socket.recv(&height_message);
        socket.recv(&depth_message);

        // Get the hostname
        std::string hostname((char*)hostname_message.data(), hostname_message.size());

        // Get the width and height
        int width = *(int*)width_message.data();
        int height = *(int*)height_message.data();

        // Deserialize the depth_message
        const uint16_t* depth_data = (const uint16_t*)depth_message.data();
        size_t size = depth_message.size();

        std::cout << "Routing ID: " << routing_id.to_string() << std::endl;
        std::cout << "Received data from: " << hostname << ", size: " << size << " bytes" << std::endl;

        // Displaying the depth frame
        cv::Mat depth_frame(height, width, CV_16U, (void*)depth_data);        
        cv::Mat depth_frame_8u;
        depth_frame.convertTo(depth_frame_8u, CV_8U, 255.0 / 3000); // assuming max depth of 3 meters
        cv::Mat depth_colormap;
        cv::applyColorMap(depth_frame_8u, depth_colormap, cv::COLORMAP_JET);

        cv::imshow("Received Depth Frame: " + hostname, depth_colormap); // Display the depth frame
        cv::waitKey(1); // Wait for a short period to allow the window to update
    }

    return 0;
}