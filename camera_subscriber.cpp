#include <iostream>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <opencv2/opencv.hpp>

int main() {
    // Initialize ZMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_ROUTER);
    socket.bind("tcp://*:5555");

    std::cout << "Subscriber started, waiting for messages..." << std::endl;

    pcl::visualization::CloudViewer viewer("Received PointCloud");

    while (!viewer.wasStopped()) {
        zmq::message_t routing_id, hostname_message, width_message, height_message, pointcloud_message;

        // Receive multipart message
        socket.recv(&routing_id); // Receive the routing ID first
        socket.recv(&hostname_message);
        socket.recv(&width_message);
        socket.recv(&height_message);
        socket.recv(&pointcloud_message);
        
        // Get the hostname
        std::string hostname((char*)hostname_message.data(), hostname_message.size());
        
        // Get the width and height
        int width = *(int*)width_message.data();
        int height = *(int*)height_message.data();
        
        // Deserialize the pointcloud data
        float* pointcloud_data = (float*)pointcloud_message.data();
        size_t pointcloud_size = pointcloud_message.size();
        size_t n_floats = pointcloud_size / sizeof(float);

        if (n_floats % 3 != 0) {
            std::cerr << "Invalid pointcloud data size: " << n_floats << " floats" << std::endl;
            continue;
        }

        std::cout << "Received data from: " << hostname << ", size: " << pointcloud_size 
        << " bytes" << " width: " << width << " height: " << height << std::endl;

        // Create a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        size_t n_points = n_floats / 3;
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;
        cloud->points.resize(n_points);

        for (size_t i = 0; i < n_points; ++i) {
            cloud->points[i].x = pointcloud_data[i * 3];
            cloud->points[i].y = pointcloud_data[i * 3 + 1];
            cloud->points[i].z = pointcloud_data[i * 3 + 2];
        }

        // Display the point cloud
        viewer.showCloud(cloud);

        // Displaying the depth frame
        // cv::Mat depth_frame(height, width, CV_16U, (void*)pointcloud_data);        
        // cv::Mat depth_frame_8u;
        // depth_frame.convertTo(depth_frame_8u, CV_8U, 255.0 / 3000); // assuming max depth of 3 meters
        // cv::Mat depth_colormap;
        // cv::applyColorMap(depth_frame_8u, depth_colormap, cv::COLORMAP_JET);

        // cv::imshow("Received Depth Frame: " + hostname, depth_colormap); // Display the depth frame
        // cv::waitKey(1); // Wait for a short period to allow the window to update
    }

    return 0;
}