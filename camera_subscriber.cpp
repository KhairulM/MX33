#include <string>
#include <iostream>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include <zmq.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "subscriber.hpp"
#include "msgs/pointcloud.hpp"

using namespace std::chrono_literals;

bool stop = false;

void signalHandler(int signum) {
    stop = true;
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        argv[1] = "pointcloud";
        argv[2] = "localhost";
        argv[3] = "5556";
    }

    if (argc == 2) {
        argv[2] = "localhost";
        argv[3] = "5556";
    }

    if (argc == 3) {
        argv[3] = "5556";
    }

    signal(SIGINT, signalHandler); // Register signal handler for Ctrl+C

    // PCL viewer
    // pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

    // Create a subscriber
    Subscriber<Pointcloud> pointcloud_subscriber(
        "PointCloudSubscriber", 
        "tcp://" + std::string(argv[2]) + ":" + std::string(argv[3]),
        std::string(argv[1])
        // "/home/control/Work/MX33/curve/broker_pub.key"
    );

    // Statistics
    const auto statistic_interval = 3s;
    int total_frames_received = 0;
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    std::cout << "Connected to broker at tcp://" + std::string(argv[2]) + ":" + std::string(argv[3]) << std::endl;
    std::cout << "Server started, waiting for messages..." << std::endl;

    while (!stop) {
        std::unique_ptr<Pointcloud> message = pointcloud_subscriber.getMessageObject();
        if (message) {
            std::vector<float> pointcloud_data = message->pointcloud_data;
            size_t pointcloud_size = pointcloud_data.size();

            if (pointcloud_size % 3 != 0) {
                std::cerr << "Invalid pointcloud data size: " << pointcloud_size << " floats" << std::endl;
                break;
            }

            // Create a PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            size_t n_points = pointcloud_size / 3;
            cloud->width = message->width;
            cloud->height = message->height;
            cloud->is_dense = false;
            cloud->points.resize(n_points);

            for (size_t i = 0; i < n_points; ++i) {
                cloud->points[i].x = pointcloud_data[i * 3];
                cloud->points[i].y = pointcloud_data[i * 3 + 1];
                cloud->points[i].z = pointcloud_data[i * 3 + 2];
            }

            // Display the point cloud
            // if (!viewer.wasStopped()) {
            //     viewer.showCloud(cloud);
            // }

            // Update statistics
            total_frames_received++;
            frame_count++;
        }

        auto now = std::chrono::steady_clock::now();
        if (now - start_time >= statistic_interval) {            
            float fps = (float)frame_count / std::chrono::duration<float>(now - start_time).count();
            std::cout << "FPS: " << fps << std::endl;

            frame_count = 0;
            start_time = now;
        }
    }

    std::cout << "Total frames received: " << total_frames_received << std::endl;

    return 0;
}