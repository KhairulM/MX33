#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Communication libraries
#include "client.hpp"
#include "publisher.hpp"

// Msgs
#include "msgs/pointcloud.hpp"
#include "msgs/transforms.hpp"
#include "msgs/pointcloud_tf.hpp"

// Srvs
#include "srvs/register_robot.hpp"

// Utils
#include "utils/get_local_ip.hpp"

class RobotClient {
    std::string robot_id;
    std::string broker_address;
    std::string broker_public_key_path;
    
    std::unique_ptr<Client<RegisterRobot::Request, RegisterRobot::Response>> register_client;
    std::unique_ptr<Publisher<PointcloudTF>> pointcloud_publisher;

public:
    RobotClient(
        std::string id,
        std::string broker_addr,
        std::string pointcloud_topic = "pointcloud_tf",
        std::string register_service_name = "register_robot",
        std::string broker_key_path = ""
    )
    : robot_id(id)
    , broker_address(broker_addr)
    , broker_public_key_path(broker_key_path)
    {
        // Create client for registration
        register_client = std::make_unique<Client<RegisterRobot::Request, RegisterRobot::Response>>(
            broker_address, register_service_name, broker_public_key_path
        );

        // Create publisher for pointcloud data
        pointcloud_publisher = std::make_unique<Publisher<PointcloudTF>>(
            robot_id + "_PointcloudPublisher", broker_address, pointcloud_topic, broker_public_key_path
        );
    }

    bool registerToMapServer() {
        try {
            RegisterRobot::Request req;
            req.id = robot_id;
            req.ip_address = get_local_ip();
            
            if (req.ip_address.empty()) {
                req.ip_address = "unknown";
            }

            std::cout << "Registering robot " << robot_id << " with IP " << req.ip_address << std::endl;

            RegisterRobot::Response resp = register_client->call(req);

            std::cout << "Registration response: " << resp.message << std::endl;
            return resp.success;

        } catch (const std::exception& e) {
            std::cerr << "Failed to register robot: " << e.what() << std::endl;
            return false;
        }
    }

    void publishPointcloud() {
        // This is a demo function that generates dummy pointcloud data
        // In a real application, this would read from a camera/sensor
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

        while (true) {
            PointcloudTF msg;
            msg.robot_id = robot_id;

            // Create dummy pointcloud (100x100 points)
            int width = 100;
            int height = 100;
            msg.pointcloud.width = width;
            msg.pointcloud.height = height;
            msg.pointcloud.pointcloud_data.resize(width * height * 3);

            // Fill with random points
            for (int i = 0; i < width * height * 3; i++) {
                msg.pointcloud.pointcloud_data[i] = dis(gen);
            }

            // Set transform from local (robot base) to camera frame
            // For demo: camera is 0.5m forward, 0.2m up from robot base, no rotation
            msg.local_to_camera_transform.x = 0.5f;
            msg.local_to_camera_transform.y = 0.0f;
            msg.local_to_camera_transform.z = 0.2f;
            msg.local_to_camera_transform.qx = 0.0f;
            msg.local_to_camera_transform.qy = 0.0f;
            msg.local_to_camera_transform.qz = 0.0f;
            msg.local_to_camera_transform.qw = 1.0f; // Identity rotation

            // Publish the pointcloud
            pointcloud_publisher->publish(msg);
            std::cout << "Published pointcloud from robot " << robot_id << std::endl;

            // Publish at 1 Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void run() {
        // First, register to the map server
        if (!registerToMapServer()) {
            std::cerr << "Failed to register robot. Exiting." << std::endl;
            return;
        }

        std::cout << "Robot " << robot_id << " registered successfully. Starting pointcloud publishing..." << std::endl;

        // Start publishing pointclouds
        publishPointcloud();
    }
};

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string robot_id = "robot_1";
    std::string broker_address = "tcp://localhost:5555";
    std::string broker_public_key_path = "";

    if (argc > 1) {
        robot_id = argv[1];
    }
    if (argc > 2) {
        broker_address = argv[2];
    }
    if (argc > 3) {
        broker_public_key_path = argv[3];
    }

    std::cout << "Starting Robot Client..." << std::endl;
    std::cout << "Robot ID: " << robot_id << std::endl;
    std::cout << "Broker address: " << broker_address << std::endl;

    RobotClient robot(robot_id, broker_address, "pointcloud_tf", "register_robot", broker_public_key_path);
    robot.run();

    return 0;
}