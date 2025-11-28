#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <csignal>
#include <map>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

// Communication libraries
#include "server.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

// Msgs
#include "msgs/pointcloud.hpp"
#include "msgs/transforms.hpp"
#include "msgs/pointcloud_tf.hpp"

// Srvs
#include "srvs/register_robot.hpp"

std::atomic<bool> is_stopped(false);

void signalHandler(int signum) {
    std::cout << "\n[MapServer] Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    is_stopped = true;
}

class Robot {
    public:
        std::string id;
        std::string ip_address;
        Transform global_to_odom_transform;

        bool connected = false;
};

class MapServer {
    std::string _transformation_file_path;

    std::map<std::string, Robot> robots;
    std::map<std::string, PointcloudTF> robot_pointclouds; // Individual scans for each robot
    std::map<std::string, Transform> global_to_odom_tf; // Global to odom transformations for each robot

    // pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
    octomap::OcTree global_map{0.1f}; // OctoMap with 0.1m resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_pcl; // PCL version for visualization

    Server<RegisterRobot::Request, RegisterRobot::Response> register_robot_server;
    Subscriber<PointcloudTF> pointcloud_subscriber;

    std::mutex robots_mutex;
    std::mutex pointclouds_mutex;
    std::mutex global_map_mutex;

    std::shared_ptr<rclcpp::Node> ros2_node;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;

    public:
        // Constructor
        MapServer(
            std::string broker_ip_address,
            std::string transformation_file_path = "",
            std::string register_robot_service_name = "register_robot",
            std::string pointcloud_topic = "pointcloud_tf",
            std::string map_save_path = "global_map.pcd",
            std::string broker_public_key_path = ""
        )
        : register_robot_server("MapServer_RegisterRobot", broker_ip_address, register_robot_service_name)
        , pointcloud_subscriber("MapServer_PointcloudSubscriber", broker_ip_address, pointcloud_topic, broker_public_key_path, 120)
        {
            _transformation_file_path = transformation_file_path;
            global_map_pcl = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            
            // Initialize ROS 2
            rclcpp::init(0, nullptr);
            ros2_node = std::make_shared<rclcpp::Node>("map_server_node");
            pointcloud_publisher = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_map", 10);
            std::cout << "[MapServer] ROS 2 publisher initialized on topic /global_map" << std::endl;
        }

        void run() {
            // Read the global to local transformation file path
            std::ifstream infile(_transformation_file_path);
            if (!infile.is_open()) {
                std::cout << "[MapServer] No existing transformation file found. Using identity transform for all robots." << std::endl;
            } else {
                // Expected format per line: robot_id    x y z qx qy qz qw
                // Where (x, y, z) is translation and (qx, qy, qz, qw) is rotation in quaternion, and all is in floating point
                // This represents the transform from the robot's local frame to the global frame
                std::string robot_id;
                Transform transform;
                while (infile >> robot_id >> transform.x >> transform.y >> transform.z
                       >> transform.qx >> transform.qy >> transform.qz >> transform.qw) {
                    global_to_odom_tf[robot_id] = transform;
                    std::cout << "[MapServer] Loaded global to odom transformation for robot " << robot_id << " from " << _transformation_file_path << std::endl;
                }
                infile.close();
            }

            // Start the threads
            std::thread register_robot_thread(&MapServer::registerRobotThread, this);
            std::thread process_pointclouds_thread(&MapServer::processPointcloudsThread, this);
            std::thread construct_global_map_thread(&MapServer::constructGlobalMapThread, this);
            std::thread ros2_spin_thread(&MapServer::ros2SpinThread, this);

            // Join the threads
            if (register_robot_thread.joinable()) {
                register_robot_thread.join();
            }
            if (process_pointclouds_thread.joinable()) {
                process_pointclouds_thread.join();
            }
            if (construct_global_map_thread.joinable()) {
                construct_global_map_thread.join();
            }
            if (ros2_spin_thread.joinable()) {
                ros2_spin_thread.join();
            }
            
            rclcpp::shutdown();
        }

        void registerRobotThread() {
            register_robot_server.registerService();
            register_robot_server.onRequestObject([this](const RegisterRobot::Request& req) -> RegisterRobot::Response {
                RegisterRobot::Response res;
                // if (robots.find(robot_info.id) != robots.end()) {
                //     res.success = false;
                //     res.message = "Robot with ID " + robot_info.id + " is already registered.";
                //     return res;
                // }

                Robot new_robot;
                new_robot.id = req.id;
                new_robot.ip_address = req.ip_address;
                new_robot.connected = true; // Mark as connected upon registration
                new_robot.global_to_odom_transform = Transform{0, 0, 0, 0, 0, 0, 1}; // Identity transform

                // If a global to odom transformation exists for this robot, use it
                if (global_to_odom_tf.find(req.id) != global_to_odom_tf.end()) {
                    new_robot.global_to_odom_transform = global_to_odom_tf[req.id];
                }

                {
                    std::lock_guard<std::mutex> lock(robots_mutex);
                    robots[req.id] = new_robot;
                }

                res.success = true;
                res.message = "[MapServer] Robot with ID " + req.id + " registered successfully.";
                std::cout << res.message << std::endl;
                return res;
            });

            register_robot_server.run(&is_stopped);
        }

        void processPointcloudsThread() {
            while (!is_stopped) {
                auto msg = pointcloud_subscriber.getMessageObjectPtr();
                if (!msg) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                // Extract robot_id from the message
                std::string robot_id = msg->robot_id;

                {
                    std::lock_guard<std::mutex> lk(pointclouds_mutex);
                    robot_pointclouds[robot_id] = *msg;
                }
            }
            pointcloud_subscriber.stop();
            std::cout << "[MapServer] Pointcloud processing thread stopped." << std::endl;
        }

        void constructGlobalMapThread() {
            // Combine all robots' pointclouds into a global map
            while (!is_stopped) {
                // Snapshot pointclouds to minimize lock holding
                std::map<std::string, PointcloudTF> snapshot;
                {
                    std::lock_guard<std::mutex> lk(pointclouds_mutex);
                    snapshot = robot_pointclouds;
                }

                // For each robot, transform its pointcloud to the global frame and merge
                for (auto it = snapshot.begin(); it != snapshot.end(); ++it) {
                    const std::string& robot_id = it->first;
                    const PointcloudTF& pc_tf = it->second;


                    const Pointcloud& pointcloud = pc_tf.pointcloud;
                    const Transform& odom_to_base_link = pc_tf.odom_to_base_link_transform;

                    int pointcloud_width = pointcloud.width;
                    int pointcloud_height = pointcloud.height;
                    const std::vector<float>& pointcloud_data = pointcloud.pointcloud_data;
                    
                    // Get the robot's global to odom transformation
                    Transform global_to_odom;
                    {
                        std::lock_guard<std::mutex> lk(robots_mutex);
                        if (robots.find(robot_id) == robots.end()) {
                            std::cout << "[MapServer] Robot ID " << robot_id << " not found in registered robots." << std::endl;
                            continue;
                        }
                        global_to_odom = robots[robot_id].global_to_odom_transform;
                    }

                    // Calculate the transformation from base_link frame to global frame
                    // Points in the pointcloud are in the base_link frame, we need to transform them to global frame
                    // Transformation chain: base_link → odom → global
                    // We have: global_to_odom (global→odom) and odom_to_base_link (odom→base_link)
                    // We need: base_link_to_odom (inverse of odom_to_base_link) and odom_to_global (inverse of global_to_odom)
                    Transform base_link_to_odom = odom_to_base_link.inverse();
                    Transform odom_to_global = global_to_odom.inverse();
                    Transform base_link_to_global = odom_to_global * base_link_to_odom;

                    // Transform the pointcloud data using the combined transformation
                    octomap::Pointcloud octo_pointcloud;
                    if (pointcloud_data.size() != pointcloud_width * pointcloud_height * 3) {
                        std::cout << "[MapServer] Pointcloud size mismatch for robot " << robot_id << std::endl;
                        continue;
                    }

                    for (size_t i = 0; i < pointcloud_data.size(); i += 3) {
                        std::array<float,3> p{ pointcloud_data[i], pointcloud_data[i+1], pointcloud_data[i+2] };
                        auto tp = base_link_to_global.transformPoint(p);
                        octo_pointcloud.push_back(tp[0], tp[1], tp[2]);
                    }

                    // Update the octomap with the transformed pointcloud
                    octomap::point3d sensor_origin(0.0f, 0.0f, 0.0f); // global origin
                    global_map.insertPointCloud(octo_pointcloud, sensor_origin, -1, true, true);
                }

                // Update inner nodes of the octomap
                global_map.updateInnerOccupancy();

                // Convert octomap to PCL point cloud and publish to ROS 2
                updatePCLPointCloud();
                publishPointCloud2();

                // Sleep for a short duration before next update
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Save the global map to a file
            std::string octomap_filename = "global_map.ot";
            std::cout << "[MapServer] Saving global map to " << octomap_filename << "..." << std::endl;
            if (global_map.write(octomap_filename)) {
                std::cout << "[MapServer] Global map saved successfully." << std::endl;
            } else {
                std::cerr << "[MapServer] Failed to save global map." << std::endl;
            }

            std::cout << "[MapServer] Global map construction thread stopped." << std::endl;
        }

        void updatePCLPointCloud() {
            // Convert OctoMap to PCL point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            
            for (octomap::OcTree::leaf_iterator it = global_map.begin_leafs(), end = global_map.end_leafs(); it != end; ++it) {
                if (global_map.isNodeOccupied(*it)) {
                    pcl::PointXYZRGB point;
                    point.x = it.getX();
                    point.y = it.getY();
                    point.z = it.getZ();
                    
                    // Color by height
                    float z_normalized = (point.z + 2.0f) / 4.0f; // Normalize z to [0, 1]
                    z_normalized = std::max(0.0f, std::min(1.0f, z_normalized));
                    
                    // Create rainbow color gradient based on height
                    if (z_normalized < 0.25f) {
                        point.r = 0;
                        point.g = static_cast<uint8_t>(255 * z_normalized * 4);
                        point.b = 255;
                    } else if (z_normalized < 0.5f) {
                        point.r = 0;
                        point.g = 255;
                        point.b = static_cast<uint8_t>(255 * (0.5f - z_normalized) * 4);
                    } else if (z_normalized < 0.75f) {
                        point.r = static_cast<uint8_t>(255 * (z_normalized - 0.5f) * 4);
                        point.g = 255;
                        point.b = 0;
                    } else {
                        point.r = 255;
                        point.g = static_cast<uint8_t>(255 * (1.0f - z_normalized) * 4);
                        point.b = 0;
                    }
                    
                    temp_cloud->points.push_back(point);
                }
            }
            
            temp_cloud->width = temp_cloud->points.size();
            temp_cloud->height = 1;
            temp_cloud->is_dense = false;
            
            // Thread-safe update
            {
                std::lock_guard<std::mutex> lock(global_map_mutex);
                global_map_pcl = temp_cloud;
            }
        }

        void ros2SpinThread() {
            std::cout << "[MapServer] Starting ROS 2 spin thread..." << std::endl;
            while (!is_stopped && rclcpp::ok()) {
                rclcpp::spin_some(ros2_node);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "[MapServer] ROS 2 spin thread stopped." << std::endl;
        }

        void publishPointCloud2() {
            if (!pointcloud_publisher) {
                return;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_copy;
            {
                std::lock_guard<std::mutex> lock(global_map_mutex);
                if (!global_map_pcl || global_map_pcl->empty()) {
                    return;
                }
                cloud_copy = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*global_map_pcl));
            }

            // Create PointCloud2 message
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            // Set header
            cloud_msg->header.stamp = ros2_node->now();
            cloud_msg->header.frame_id = "map";
            
            // Set dimensions
            cloud_msg->height = 1;
            cloud_msg->width = cloud_copy->points.size();
            cloud_msg->is_bigendian = false;
            cloud_msg->is_dense = false;
            
            // Set fields
            sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(cloud_copy->points.size());
            
            // Fill data
            sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
            
            for (const auto& point : cloud_copy->points) {
                *iter_x = point.x;
                *iter_y = point.y;
                *iter_z = point.z;
                *iter_r = point.r;
                *iter_g = point.g;
                *iter_b = point.b;
                
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }
            
            // Publish
            pointcloud_publisher->publish(*cloud_msg);
        }
};

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Default parameters
    std::string broker_ip_address = "localhost";
    std::string transformation_file_path = "global_to_odom_tf.txt";
    std::string register_robot_service_name = "register_robot";
    std::string pointcloud_topic = "pointcloud_tf";
    std::string map_save_path = "global_map.pcd";
    std::string broker_public_key_path = "";

    // Parse command line arguments if needed
    if (argc > 1) {
        broker_ip_address = argv[1];
    }
    if (argc > 2) {
        transformation_file_path = argv[2];
    }
    if (argc > 3) {
        broker_public_key_path = argv[3];
    }

    std::cout << "[MapServer] Starting Map Server..." << std::endl;
    std::cout << "[MapServer] Broker address: " << broker_ip_address << std::endl;
    std::cout << "[MapServer] Transformation file: " << transformation_file_path << std::endl;
    std::cout << "[MapServer] ROS 2 Publishing: enabled" << std::endl;

    MapServer map_server(
        broker_ip_address,
        transformation_file_path,
        register_robot_service_name,
        pointcloud_topic,
        map_save_path,
        broker_public_key_path
    );

    map_server.run();

    std::cout << "[MapServer] Map Server stopped gracefully." << std::endl;
    return 0;
}