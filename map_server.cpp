#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <map>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Communication libraries
#include "robot.hpp"
#include "server.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

// Msgs
#include "msgs/pointcloud.hpp"
#include "msgs/transforms.hpp"
#include "msgs/pointcloud_tf.hpp"

// Srvs
#include "srvs/register_robot.hpp"

class MapServer {
    std::string _transformation_file_path;

    std::map<std::string, Robot> robots;
    std::map<std::string, PointcloudTF> robot_pointclouds; // Individual scans for each robot
    std::map<std::string, Transform> global_to_odom_tf; // Map of global to odom transformations for each robot

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;

    Server<RegisterRobot::Request, RegisterRobot::Response> register_robot_server;
    Subscriber<PointcloudTF> pointcloud_subscriber;

    std::mutex robots_mutex;
    std::mutex pointclouds_mutex;
    std::mutex global_map_mutex;

    // Optional: viewer removed to avoid dependency on PCL visualization

    public:
        // Constructor
        MapServer(
            std::string broker_address,
            std::string transformation_file_path = "",
            std::string register_robot_service_name = "register_robot",
            std::string pointcloud_topic = "pointcloud_tf",
            std::string map_save_path = "global_map.pcd",
            std::string broker_public_key_path = ""
        )
        : register_robot_server("MapServer_RegisterRobot", broker_address, register_robot_service_name, "", "0", broker_public_key_path)
        , pointcloud_subscriber("MapServer_PointcloudSubscriber", broker_address, pointcloud_topic, broker_public_key_path)
        {
            _transformation_file_path = transformation_file_path;
            global_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
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

            // Join the threads
            register_robot_thread.join();
            process_pointclouds_thread.join();
            construct_global_map_thread.join();
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

            register_robot_server.run();
        }

        void processPointcloudsThread() {
            while (true) {
                auto msg = pointcloud_subscriber.getMessageObjectPtr();
                if (!msg) continue;

                // Extract robot_id from the message
                std::string robot_id = msg->robot_id;

                {
                    std::lock_guard<std::mutex> lk(pointclouds_mutex);
                    robot_pointclouds[robot_id] = *msg;
                }
            }
        }

        void constructGlobalMapThread() {
            // Combine all robots' pointclouds into a global map
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(global_map_mutex);
                    global_map->clear();
                }

                pcl::PointCloud<pcl::PointXYZ>::Ptr new_global(new pcl::PointCloud<pcl::PointXYZ>());

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
                    Pointcloud pointcloud = pc_tf.pointcloud;
                    Transform odom_to_base_link = pc_tf.odom_to_base_link_transform;

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
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    transformed_cloud->width = pointcloud_width;
                    transformed_cloud->height = pointcloud_height;
                    transformed_cloud->is_dense = false;
                    transformed_cloud->points.resize(pointcloud_width * pointcloud_height);

                    if (pointcloud_data.size() != static_cast<size_t>(pointcloud_width) * static_cast<size_t>(pointcloud_height) * 3) {
                        std::cout << "[MapServer] Pointcloud size mismatch for robot " << robot_id << std::endl;
                        continue;
                    }

                    for (size_t i = 0; i < pointcloud_data.size(); i += 3) {
                        std::array<float,3> p{ pointcloud_data[i], pointcloud_data[i+1], pointcloud_data[i+2] };
                        auto tp = base_link_to_global.transformPoint(p);
                        auto& dst = transformed_cloud->points[i/3];
                        dst.x = tp[0]; dst.y = tp[1]; dst.z = tp[2];
                    }

                    // Accumulate into new global cloud
                    *new_global += *transformed_cloud;
                }

                // Swap into global_map under lock
                {
                    std::lock_guard<std::mutex> lk(global_map_mutex);
                    *global_map = *new_global;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5 Hz
            }
        }
};

int main(int argc, char* argv[]) {
    // Default parameters
    std::string broker_address = "tcp://localhost:5555";
    std::string transformation_file_path = "global_to_local_tf.txt";
    std::string register_robot_service_name = "register_robot";
    std::string pointcloud_topic = "pointcloud_tf";
    std::string map_save_path = "global_map.pcd";
    std::string broker_public_key_path = "";

    // Parse command line arguments if needed
    if (argc > 1) {
        broker_address = argv[1];
    }
    if (argc > 2) {
        transformation_file_path = argv[2];
    }
    if (argc > 3) {
        broker_public_key_path = argv[3];
    }

    std::cout << "[MapServer] Starting Map Server..." << std::endl;
    std::cout << "[MapServer] Broker address: " << broker_address << std::endl;
    std::cout << "[MapServer] Transformation file: " << transformation_file_path << std::endl;

    MapServer map_server(
        broker_address,
        transformation_file_path,
        register_robot_service_name,
        pointcloud_topic,
        map_save_path,
        broker_public_key_path
    );

    map_server.run();

    return 0;
}