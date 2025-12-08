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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

// Communication libraries
#include "server.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

// Msgs
#include "msgs/pointcloud.hpp"
#include "msgs/transforms.hpp"
#include "msgs/pointcloud.hpp"
#include "msgs/pointcloud_tf.hpp"

// Srvs
#include "srvs/register_robot.hpp"
#include "srvs/get_frontiers.hpp"

std::atomic<bool> is_stopped(false);

void signalHandler(int signum) {
    std::cout << "\n[MapServer] Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    is_stopped = true;
}

class Robot {
    public:
        std::string id;
        std::string ip_address;
        Transform global_to_base_link_tf;
        Pointcloud pointcloud;

        bool connected = false;
};

class MapServer {
    std::string gto_file_path;

    // List of robots
    std::map<std::string, Robot> robots;
    std::map<std::string, Transform> robot_gtos; // Global to odom transformation defined in an external file
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
    octomap::OcTree global_map{0.1f}; // OctoMap resolution in meters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_pcl; // PCL version for visualization

    Server<RegisterRobot::Request, RegisterRobot::Response> register_robot_server;
    Server<GetFrontiers::Request, GetFrontiers::Response> get_frontiers_server;
    
    Subscriber<PointcloudTF> pointcloud_subscriber;

    std::mutex robots_mutex;
    std::mutex global_map_mutex;

    std::shared_ptr<rclcpp::Node> ros2_node;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> robot_pose_publishers;

    public:
        // Constructor
        MapServer(
            std::string broker_ip_address,
            std::string transformation_file_path = "",
            std::string pointcloud_topic = "pointcloud_tf",
            std::string map_save_path = "global_map.pcd",
            std::string broker_public_key_path = ""
        )
        : register_robot_server("MapServer_RegisterRobot", broker_ip_address, "register_robot"),
        get_frontiers_server("MapServer_GetFrontiers", broker_ip_address, "get_frontiers"),
        pointcloud_subscriber("MapServer_PointcloudSubscriber", broker_ip_address, pointcloud_topic, broker_public_key_path, 120)
        {
            gto_file_path = transformation_file_path;
            global_map_pcl = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            
            // Initialize ROS 2
            rclcpp::init(0, nullptr);
            ros2_node = std::make_shared<rclcpp::Node>("map_server_node");
            pointcloud_publisher = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_map", rclcpp::QoS(1).best_effort().durability_volatile());
            
            std::cout << "[MapServer] ROS 2 publisher initialized on topic /global_map" << std::endl;
        }

        void run() {
            // Read the global to local transformation file path
            std::ifstream infile(gto_file_path);
            if (!infile.is_open()) {
                std::cout << "[MapServer] No existing transformation from global to robot's odom file found" << std::endl;
            } else {
                // Expected format per line: robot_id    x y z qx qy qz qw
                // Where (x, y, z) is translation and (qx, qy, qz, qw) is rotation in quaternion, and all is in floating point
                // This represents the transform from the robot's local frame to the global frame
                std::string robot_id;
                Transform transform;
                while (infile >> robot_id >> transform.x >> transform.y >> transform.z
                       >> transform.qx >> transform.qy >> transform.qz >> transform.qw) {
                    robot_gtos[robot_id] = transform;
                    std::cout << "[MapServer] Loaded global to odom transformation for " << robot_id << " from " << gto_file_path << std::endl;
                }
                infile.close();
            }

            // Start the threads
            std::thread register_robot_thread(&MapServer::registerRobotThread, this);
            std::thread get_frontiers_thread(&MapServer::getFrontiersThread, this);
            std::thread process_pointclouds_thread(&MapServer::processPointcloudsThread, this);
            std::thread construct_global_map_thread(&MapServer::constructGlobalMapThread, this);
            std::thread ros2_spin_thread(&MapServer::ros2SpinThread, this);

            // Join the threads
            if (register_robot_thread.joinable()) {
                register_robot_thread.join();
            }
            if (get_frontiers_thread.joinable()) {
                get_frontiers_thread.join();
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

        void ros2SpinThread() {
            std::cout << "[MapServer] Starting ROS 2 spin thread..." << std::endl;
            while (!is_stopped && rclcpp::ok()) {
                rclcpp::spin_some(ros2_node);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "[MapServer] ROS 2 spin thread stopped." << std::endl;
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
                new_robot.connected = true;

                std::cout << "[MapServer] New robot registration received" << std::endl;
                std::cout << "[MapServer] Robot id: " << req.id << std::endl;
                std::cout << "[MapServer] Robot ip address: " << req.ip_address << std::endl;
                
                // If a global to odom transformation exists for this robot, use it as the initial global to base link transformation
                if (robot_gtos.find(req.id) != robot_gtos.end()) {
                    new_robot.global_to_base_link_tf = robot_gtos[req.id];
                    std::cout << "[MapServer] Assigned existing global to odom transformation for robot " << req.id << std::endl;
                }

                {
                    std::lock_guard<std::mutex> lock(robots_mutex);
                    robots[req.id] = new_robot;
                }

                robot_pose_publishers[req.id] = ros2_node->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "/" + req.id + "/pose", rclcpp::QoS(1).best_effort().durability_volatile());

                res.success = true;
                res.message = "[MapServer] Robot with ID " + req.id + " registered successfully.";
                
                std::cout << res.message << std::endl;
                return res;
            });

            register_robot_server.run(&is_stopped);
        }

        void getFrontiersThread() {
            get_frontiers_server.registerService();
            get_frontiers_server.onRequestObject([this](const GetFrontiers::Request& req) -> GetFrontiers::Response {
                GetFrontiers::Response res;
                
                std::cout << "[MapServer] Received get frontiers request for bounds: " 
                         << "\nx[" << req.x_min << "," << req.x_max << "] "
                         << "\ny[" << req.y_min << "," << req.y_max << "] "
                         << "\nz[" << req.z_min << "," << req.z_max << "]" 
                         << "\nmin_radius_from_robot: " << req.min_radius_from_robot
                         << "\noccupied_neighbors_cell_dist: " << req.occupied_neighbors_cell_dist 
                         << std::endl;
                
                // Find frontiers which satisfies the parameters
                res.frontiers = findFrontiers(
                    req.x_min, req.x_max, 
                    req.y_min, req.y_max, 
                    req.z_min, req.z_max,
                    req.min_radius_from_robot,
                    req.occupied_neighbors_cell_dist
                );
                
                std::cout << "[MapServer] Found " << res.frontiers.size() << " frontier points" << std::endl;
                return res;
            });

            get_frontiers_server.run(&is_stopped);
        }
        
        void processPointcloudsThread() {
            while (!is_stopped) {
                auto msg = pointcloud_subscriber.getMessageObjectPtr();
                if (!msg) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                // Update the robot global to base link transformation
                std::string robot_id = msg->robot_id;

                // Check if the robot is registered
                if (robots.find(robot_id) == robots.end()) {
                    std::cout << "[MapServer] Robot ID: " << robot_id << " is not registered in the map server" << std::endl;
                    continue;
                }


                // Check if there is a global to odom tf assigned for this robot, if not, we use an identity transformation
                Transform global_to_odom_tf;
                global_to_odom_tf.qw = 1.0;
                if (robot_gtos.find(robot_id) != robot_gtos.end()) {
                    global_to_odom_tf = robot_gtos[robot_id];
                }

                {
                    std::lock_guard<std::mutex> lock(robots_mutex);
                    // Calculate the transformation from global to base link and store it in the robots map
                    robots[robot_id].global_to_base_link_tf = global_to_odom_tf * msg->odom_to_base_link_transform;
                    robots[robot_id].pointcloud = msg->pointcloud;
                }


                // float x, y, z;
                // float qx, qy, qz, qw;

                // x = msg->odom_to_base_link_transform.x;
                // y = msg->odom_to_base_link_transform.y;
                // z = msg->odom_to_base_link_transform.z;
                // qx = msg->odom_to_base_link_transform.qx;
                // qy = msg->odom_to_base_link_transform.qy;
                // qz = msg->odom_to_base_link_transform.qz;
                // qw = msg->odom_to_base_link_transform.qw;

                // std::cout << "Robot Pose x:" << x << " y:" << y << " z:" << z << std::endl;
                // std::cout << "Robot Orientation qx:" << qx << " qy:" << qy << " qz:" << qz << " w:" << qw << std::endl; 
            }

            pointcloud_subscriber.stop();
            std::cout << "[MapServer] Pointcloud processing thread stopped." << std::endl;
        }

        void constructGlobalMapThread() {
            // Combine all robots' pointclouds into a global map
            while (!is_stopped) {            
                // Snapshot robot's to minimize lock holding
                std::map<std::string, Robot> snapshot;
                {
                    std::lock_guard<std::mutex> lk(robots_mutex);
                    snapshot = robots;
                }

                // For each robot, transform its pointcloud to the global frame and merge
                for (auto it = snapshot.begin(); it != snapshot.end(); ++it) {
                    const std::string& robot_id = it->first;
                    const Robot& robot = it->second;

                    const Pointcloud& pointcloud = robot.pointcloud;
                    const Transform& global_to_base_link_tf = robot.global_to_base_link_tf;

                    int pointcloud_width = pointcloud.width;
                    int pointcloud_height = pointcloud.height;
                    const std::vector<float>& pointcloud_data = pointcloud.pointcloud_data;                

                    // Transform the pointcloud data using the combined transformation
                    octomap::Pointcloud octo_pointcloud;
                    if (pointcloud_data.size() != pointcloud_width * pointcloud_height * 3) {
                        std::cout << "[MapServer] Pointcloud size mismatch for robot " << robot_id << std::endl;
                        continue;
                    }

                    for (size_t i = 0; i < pointcloud_data.size(); i += 3) {
                        std::array<float,3> p{ pointcloud_data[i], pointcloud_data[i+1], pointcloud_data[i+2] };
                        auto tp = global_to_base_link_tf.transformPoint(p);
                        octo_pointcloud.push_back(tp[0], tp[1], tp[2]);
                    }

                    // Update the octomap with the transformed pointcloud
                    octomap::point3d sensor_origin(
                        global_to_base_link_tf.x, 
                        global_to_base_link_tf.y,
                        global_to_base_link_tf.z
                    );
                    global_map.insertPointCloud(octo_pointcloud, sensor_origin, -1, false, true);
                    // global_map.insertPointCloud(octo_pointcloud, sensor_origin);
                    
                    // Publish the robot's pose in the global frame
                    publishRobotPose(robot_id, global_to_base_link_tf);
                }

                // Update inner nodes of the octomap
                // global_map.updateInnerOccupancy();

                // Convert octomap to PCL point cloud and publish to ROS 2
                updatePCLPointCloud();
                publishPointCloud2();

                // Sleep for a short duration before next update
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    private:
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
                    float z_normalized = (point.z + 2.5f) / 6.0f; // Normalize z to [0, 1]
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

        void publishRobotPose(const std::string& robot_id, const Transform& base_link_to_global) {
            auto pose_publisher_it = robot_pose_publishers.find(robot_id);
            if (pose_publisher_it == robot_pose_publishers.end()) {
                std::cout << "[MapServer] No pose publisher found for robot " << robot_id << std::endl;
                return;
            }

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = ros2_node->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = base_link_to_global.x;
            pose_msg.pose.position.y = base_link_to_global.y;
            pose_msg.pose.position.z = base_link_to_global.z;
            pose_msg.pose.orientation.x = base_link_to_global.qx;
            pose_msg.pose.orientation.y = base_link_to_global.qy;
            pose_msg.pose.orientation.z = base_link_to_global.qz;
            pose_msg.pose.orientation.w = base_link_to_global.qw;

            pose_publisher_it->second->publish(pose_msg);
        }

        std::vector<std::array<float, 3>> findFrontiers(
            float x_min, float x_max, 
            float y_min, float y_max, 
            float z_min, float z_max,
            float min_radius_from_robot,
            int occupied_neighbors_cell_dist
        ) {
            std::vector<std::array<float, 3>> frontiers;
            
            // Get robot positions for distance checking
            std::vector<std::array<float, 3>> robot_positions;
            {
                std::lock_guard<std::mutex> lock(robots_mutex);
                for (const auto& pair : robots) {
                    robot_positions.push_back({
                        pair.second.global_to_base_link_tf.x,
                        pair.second.global_to_base_link_tf.y,
                        pair.second.global_to_base_link_tf.z
                    });
                }
            }
            
            // Lock the global map to ensure thread safety
            std::lock_guard<std::mutex> lock(global_map_mutex);
            
            // Get the resolution of the octomap
            double resolution = global_map.getResolution();
            
            // Iterate through all leaf nodes in the specified bounding box
            for (octomap::OcTree::leaf_bbx_iterator it = global_map.begin_leafs_bbx(
                    octomap::point3d(x_min, y_min, z_min), 
                    octomap::point3d(x_max, y_max, z_max)), 
                 end = global_map.end_leafs_bbx(); it != end; ++it) {
                
                // Check if the current cell is free (not occupied)
                if (global_map.isNodeOccupied(*it)) continue;

                octomap::point3d center = it.getCoordinate();
                
                // Check if this point is too close to any robot
                bool too_close_to_robot = false;
                for (const auto& robot_pos : robot_positions) {
                    float dx = center.x() - robot_pos[0];
                    float dy = center.y() - robot_pos[1];
                    float dz = center.z() - robot_pos[2];
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (dist < min_radius_from_robot) {
                        too_close_to_robot = true;
                        break;
                    }
                }
                if (too_close_to_robot) continue;
                
                // Check if this free cell is a frontier (has unknown neighbors and no nearby obstacles)
                if (isFrontierCell(center, resolution, occupied_neighbors_cell_dist)) {
                    frontiers.push_back({
                        static_cast<float>(center.x()),
                        static_cast<float>(center.y()),
                        static_cast<float>(center.z())
                    });
                }
            }
            
            return frontiers;
        }
        
        bool isFrontierCell(const octomap::point3d& center, double resolution, int occupied_neighbors_cell_dist) {
            bool has_unknown_neighbor = false;
            
            // First check: Must have at least one unknown neighbor (6-connectivity for speed)
            const std::vector<std::array<int, 3>> face_neighbors = {
                {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}
            };
            
            for (const auto& offset : face_neighbors) {
                octomap::point3d neighbor(
                    center.x() + offset[0] * resolution,
                    center.y() + offset[1] * resolution,
                    center.z() + offset[2] * resolution
                );
                
                octomap::OcTreeNode* neighbor_node = global_map.search(neighbor);
                if (neighbor_node == nullptr) {
                    has_unknown_neighbor = true;
                    break;
                }
            }
            
            // If no unknown neighbors, this is not a frontier
            if (!has_unknown_neighbor) return false;
            
            // Second check: No occupied cells within occupied_neighbors_cell_dist (using 6-connectivity)
            if (occupied_neighbors_cell_dist > 0) {
                for (int dist = 1; dist <= occupied_neighbors_cell_dist; dist++) {
                    for (const auto& offset : face_neighbors) {
                        octomap::point3d neighbor(
                            center.x() + offset[0] * dist * resolution,
                            center.y() + offset[1] * dist * resolution,
                            center.z() + offset[2] * dist * resolution
                        );
                        
                        octomap::OcTreeNode* neighbor_node = global_map.search(neighbor);
                        // If there's an occupied cell nearby, reject this frontier
                        if (neighbor_node != nullptr && global_map.isNodeOccupied(neighbor_node)) {
                            return false;
                        }
                    }
                }
            }
            
            return true;
        }
};

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Default parameters
    std::string broker_ip_address = "localhost";
    std::string transformation_file_path = "global_to_odom_tf.txt";
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
        pointcloud_topic,
        map_save_path,
        broker_public_key_path
    );

    map_server.run();

    std::cout << "[MapServer] Map Server stopped gracefully." << std::endl;
    return 0;
}