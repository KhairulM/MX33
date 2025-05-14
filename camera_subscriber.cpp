#include <iostream>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <zmq.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

auto STATISTIC_INTERVAL = 3s;
bool is_stopped = false;

class PointCloudSubscriber {
    public:
        std::string hostname;
        int width;
        int height;

        // pcl::visualization::CloudViewer::Ptr viewer;

        int total_frames_received;
        int frame_count;
        std::chrono::steady_clock::time_point start_time;

        // Default constructor
        PointCloudSubscriber() : hostname(""), width(0), height(0) {
            // viewer = nullptr;
            total_frames_received = 0;
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
        }

        // Constructor
        PointCloudSubscriber(std::string host, int w, int h): hostname(host), width(w), height(h) {
            // viewer = pcl::visualization::CloudViewer::Ptr(new pcl::visualization::CloudViewer(hostname + " Point Cloud Viewer"));
            total_frames_received = 0;
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
        }

        // Function to display the point cloud
        void onPointCloudMessage(zmq::message_t& pointcloud_message) {
            // Deserialize the pointcloud data
            float* pointcloud_data = (float*)pointcloud_message.data();
            size_t pointcloud_size = pointcloud_message.size();
            size_t n_floats = pointcloud_size / sizeof(float);

            if (n_floats % 3 != 0) {
                std::cerr << "Invalid pointcloud data size: " << n_floats << " floats" << std::endl;
                return;
            }

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
            // if (!viewer->wasStopped()) {
            //     viewer->showCloud(cloud);
            // }

            // Update statistics
            total_frames_received++;
            frame_count++;
            auto now = std::chrono::steady_clock::now();
            if (now - start_time >= STATISTIC_INTERVAL) {
                float fps = frame_count / std::chrono::duration<float>(now - start_time).count();
                frame_count = 0;
                start_time = now;

                std::cout << "Received data from: " << hostname << ", size: " << pointcloud_size  << " bytes, " << "FPS average: " << fps << std::endl;
            }
        }
};

void signalHandler(int signum) {
    is_stopped = true;
}


int main() {
    signal(SIGINT, signalHandler); // Register signal handler for Ctrl+C

    // Initialize ZMQ context and socket
    const int zmq_hwm = 10;
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_ROUTER);
    socket.setsockopt(ZMQ_RCVHWM, zmq_hwm);
    socket.bind("tcp://*:5555");

    // Store map of routing id to PointCloudSubscriber
    std::map<std::string, PointCloudSubscriber> subscribers;
    
    std::cout << "Server started, waiting for messages..." << std::endl;

    while (!is_stopped) {
        zmq::message_t routing_id, hostname_message, width_message, height_message, pointcloud_message;

        // Receive multipart message
        try {
            socket.recv(&routing_id); // Receive the routing ID first
            socket.recv(&hostname_message);
            socket.recv(&width_message);
            socket.recv(&height_message);
            socket.recv(&pointcloud_message);
        } catch (const zmq::error_t& e) {
            break;
        }

        // Convert routing ID to string
        std::string routing_id_str(static_cast<char*>(routing_id.data()), routing_id.size());

        if (subscribers.find(routing_id_str) == subscribers.end()) {
            // New publisher, create a new PointCloudSubscriber
            std::string hostname((char*)hostname_message.data(), hostname_message.size());
            int width = *(int*)width_message.data();
            int height = *(int*)height_message.data();

            subscribers.emplace(routing_id_str, PointCloudSubscriber(hostname, width, height));
        }

        subscribers[routing_id_str].onPointCloudMessage(pointcloud_message);
    }

    for (auto& subscriber : subscribers) {
        std::cout << "\nSubscriber Hostname: " << subscriber.second.hostname << ", Total frames received: " << subscriber.second.total_frames_received << std::endl;
    }

    return 0;
}