#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <csignal>
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#ifdef ZED_AVAILABLE
#include <sl/Camera.hpp>
#endif

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

std::atomic<bool> is_stopped(false);

void signalHandler(int signum) {
    is_stopped = true;
}

enum CameraType {
    REALSENSE,
    RANDOM
#ifdef ZED_AVAILABLE
    , ZED
#endif
};

class RobotClient {
    std::string robot_id;
    std::string broker_ip_address;
    std::string broker_public_key_path;
    
    Client<RegisterRobot::Request, RegisterRobot::Response> register_client;
    Publisher<PointcloudTF> pointcloud_publisher;

    CameraType camera_type;

    // Configure RealSense camera
    const int resolution_width = 640; // Camera width
    const int resolution_height = 480; // Camera height
    const int decimation_magnitude = 8; // Decimation filter magnitude
    const int camera_fps = 15; // Camera frames per second

    PointcloudTF msg;

    // Random pointcloud generation
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> pos_dist;

    // RealSense specific
    rs2::pipeline camera_pipeline;
    rs2::config rs_config;
    rs2::decimation_filter decimation_filter;

#ifdef ZED_AVAILABLE
    // ZED specific
    sl::Camera zed;
    sl::InitParameters zed_init_params;
    sl::RuntimeParameters zed_runtime_params;
    sl::Mat zed_point_cloud;
    sl::Pose zed_pose;
#endif

public:
    RobotClient(
        std::string id,
        std::string broker_ip_address,
        std::string pointcloud_topic = "pointcloud_tf",
        std::string register_service_name = "register_robot",
        std::string broker_key_path = "",
        CameraType cam_type = REALSENSE
    ): register_client("robot_register_client", broker_ip_address, register_service_name, broker_public_key_path), 
       pointcloud_publisher(robot_id + "_PointcloudPublisher", broker_ip_address, pointcloud_topic, broker_public_key_path),
       camera_type(cam_type), gen(rd()), pos_dist(-2.0f, 2.0f)
    {
        this->robot_id = id;
        this->broker_ip_address = broker_ip_address;
        this->broker_public_key_path = broker_key_path;

        msg.robot_id = robot_id;

        if (camera_type == REALSENSE) {
            // Initialize RealSense
            decimation_filter = rs2::decimation_filter(decimation_magnitude);
            rs_config.enable_stream(RS2_STREAM_DEPTH, resolution_width, resolution_height, RS2_FORMAT_Z16, camera_fps);
        } else if (camera_type == RANDOM) {
            std::cout << "[Robot " << robot_id << "] Using random pointcloud generation for testing" << std::endl;
        }
#ifdef ZED_AVAILABLE
        else if (camera_type == ZED) {
            // Initialize ZED
            zed_init_params.camera_resolution = sl::RESOLUTION::HD720;
            zed_init_params.camera_fps = camera_fps;
            zed_init_params.coordinate_units = sl::UNIT::METER;
            zed_init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
            zed_init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
            
            // Enable positional tracking for pose
            sl::PositionalTrackingParameters tracking_params;
            tracking_params.enable_area_memory = false;
            
            sl::ERROR_CODE err = zed.open(zed_init_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                std::cerr << "[Robot " << robot_id << "] Failed to open ZED camera: " << err << std::endl;
                throw std::runtime_error("Failed to open ZED camera");
            }
            
            err = zed.enablePositionalTracking(tracking_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                std::cerr << "[Robot " << robot_id << "] Failed to enable positional tracking: " << err << std::endl;
            }
            
            std::cout << "[Robot " << robot_id << "] ZED camera initialized successfully" << std::endl;
        }
#endif
    }

    void registerToMapServer() {
        try {
            RegisterRobot::Request req;
            req.id = robot_id;
            req.ip_address = get_local_ip();
            
            if (req.ip_address.empty()) {
                throw std::runtime_error("[Robot " + robot_id + "] Failed to obtain local IP address.");
            }

            std::cout << "[Robot " << robot_id << "] Registering robot " << robot_id << " with IP " << req.ip_address << std::endl;

            RegisterRobot::Response resp = register_client.call(req);
            if (resp.success) {
                std::cout << "[Robot " << robot_id << "] Registration successful: " << resp.message << std::endl;
            } else {
                std::cerr << "[Robot " << robot_id << "] Registration failed: " << resp.message << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[Robot " << robot_id << "] Failed to register robot: " << e.what() << std::endl;
        }
    }

    void realsenseCameraCallback(const rs2::frame& frame) {
        // Check if stopped
        if (is_stopped) return;
        
        // Check if it's a synchronized frame or not
        rs2::frameset frameset = frame.as<rs2::frameset>();
        if (!frameset) return;
        
        // Check if depth frame is available
        rs2::depth_frame depth_frame = frameset.get_depth_frame();
        if (!depth_frame) return;
        
        // Decimate the depth frame
        rs2::frame decimated_depth = decimation_filter.process(depth_frame);
        depth_frame = decimated_depth.as<rs2::depth_frame>();
        
        int width = depth_frame.get_width();
        int height = depth_frame.get_height();
        int size = width * height;
        
        // Get the pointcloud data
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth_frame);
        
        // Flatten the pointcloud data
        const rs2::vertex* vertices = points.get_vertices();
        std::vector<float> pointcloud_data(size * 3);
        memcpy(pointcloud_data.data(), vertices, size * 3 * sizeof(float));

        // Copy pointcloud data to msg
        msg.pointcloud.width = resolution_width / decimation_magnitude;
        msg.pointcloud.height = resolution_height / decimation_magnitude;
        msg.pointcloud.pointcloud_data = pointcloud_data;
    }

    void generateRandomPointcloud() {
        if (camera_type != RANDOM) return;
        
        // Generate random pointcloud with 1000-5000 points
        std::uniform_int_distribution<int> point_count_dist(1000, 5000);
        int num_points = point_count_dist(gen);
        
        std::vector<float> pointcloud_data;
        pointcloud_data.reserve(num_points * 3);
        
        for (int i = 0; i < num_points; i++) {
            pointcloud_data.push_back(pos_dist(gen)); // x
            pointcloud_data.push_back(pos_dist(gen)); // y
            pointcloud_data.push_back(pos_dist(gen)); // z
        }
        
        // Update message
        msg.pointcloud.width = num_points;
        msg.pointcloud.height = 1;
        msg.pointcloud.pointcloud_data = pointcloud_data;
        
        // Use identity transform for random pointcloud
        msg.odom_to_base_link_transform.x = 0.0f;
        msg.odom_to_base_link_transform.y = 0.0f;
        msg.odom_to_base_link_transform.z = 0.0f;
        msg.odom_to_base_link_transform.qx = 0.0f;
        msg.odom_to_base_link_transform.qy = 0.0f;
        msg.odom_to_base_link_transform.qz = 0.0f;
        msg.odom_to_base_link_transform.qw = 1.0f;
    }

    void captureZEDFrame() {
#ifdef ZED_AVAILABLE
        if (camera_type != ZED) return;
        
        if (zed.grab(zed_runtime_params) == sl::ERROR_CODE::SUCCESS) {
            // Retrieve point cloud
            zed.retrieveMeasure(zed_point_cloud, sl::MEASURE::XYZRGBA);
            
            // Get camera pose
            sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(zed_pose);
            
            // Extract point cloud data
            int width = zed_point_cloud.getWidth();
            int height = zed_point_cloud.getHeight();
            
            sl::float4* point_cloud_data = zed_point_cloud.getPtr<sl::float4>();
            
            std::vector<float> pointcloud_data;
            pointcloud_data.reserve(width * height * 3);
            
            for (int i = 0; i < width * height; i++) {
                sl::float4 point = point_cloud_data[i];
                // Check for valid points (ZED uses NaN for invalid points)
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    pointcloud_data.push_back(point.x);
                    pointcloud_data.push_back(point.y);
                    pointcloud_data.push_back(point.z);
                }
            }
            
            // Update message
            msg.pointcloud.width = pointcloud_data.size() / 3;
            msg.pointcloud.height = 1;
            msg.pointcloud.pointcloud_data = pointcloud_data;
            
            // Extract pose from ZED and set as odom_to_base_link transform
            if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
                sl::Translation translation = zed_pose.getTranslation();
                sl::Orientation orientation = zed_pose.getOrientation();
                
                msg.odom_to_base_link_transform.x = translation.x;
                msg.odom_to_base_link_transform.y = translation.y;
                msg.odom_to_base_link_transform.z = translation.z;
                msg.odom_to_base_link_transform.qx = orientation.x;
                msg.odom_to_base_link_transform.qy = orientation.y;
                msg.odom_to_base_link_transform.qz = orientation.z;
                msg.odom_to_base_link_transform.qw = orientation.w;
            } else {
                // If tracking is not available, use identity transform
                msg.odom_to_base_link_transform.x = 0.0f;
                msg.odom_to_base_link_transform.y = 0.0f;
                msg.odom_to_base_link_transform.z = 0.0f;
                msg.odom_to_base_link_transform.qx = 0.0f;
                msg.odom_to_base_link_transform.qy = 0.0f;
                msg.odom_to_base_link_transform.qz = 0.0f;
                msg.odom_to_base_link_transform.qw = 1.0f;
            }
        }
#endif
    }

    void publishPointcloud() {
        if (camera_type == RANDOM) {
            generateRandomPointcloud();
        }
#ifdef ZED_AVAILABLE
        else if (camera_type == ZED) {
            captureZEDFrame();
        }
#endif
        else {
            // For RealSense, use identity transform (no built-in pose tracking)
            msg.odom_to_base_link_transform.x = 0.0f;
            msg.odom_to_base_link_transform.y = 0.0f;
            msg.odom_to_base_link_transform.z = 0.0f;
            msg.odom_to_base_link_transform.qx = 0.0f;
            msg.odom_to_base_link_transform.qy = 0.0f;
            msg.odom_to_base_link_transform.qz = 0.0f;
            msg.odom_to_base_link_transform.qw = 1.0f;
        }

        // Publish the pointcloud
        pointcloud_publisher.publish(msg);
        std::cout << "[Robot " << robot_id << "] Published pointcloud (" 
                  << msg.pointcloud.pointcloud_data.size() / 3 << " points) from robot " 
                  << robot_id << std::endl;
    }
    
    void run() {
        // First, register to the map server
        registerToMapServer();

        if (camera_type == REALSENSE) {
            // Start the Realsense camera pipeline with lambda callback
            camera_pipeline.start(rs_config, [this](const rs2::frame& frame) {
                this->realsenseCameraCallback(frame);
            });
        } else if (camera_type == RANDOM) {
            std::cout << "[Robot " << robot_id << "] Starting random pointcloud generation..." << std::endl;
        }
                
        // Start publishing pointclouds
        while (!is_stopped) {
            publishPointcloud();
            // Publish at 1 Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        // Stop the camera
        if (camera_type == REALSENSE) {
            camera_pipeline.stop();
        } else if (camera_type == RANDOM) {
            std::cout << "[Robot " << robot_id << "] Stopping random pointcloud generation..." << std::endl;
        }
#ifdef ZED_AVAILABLE
        else if (camera_type == ZED) {
            zed.close();
        }
#endif
        
        std::cout << "[Robot " << robot_id << "] Stopped gracefully." << std::endl;
    }
};

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse command line arguments
    std::string robot_id = "1";
    std::string broker_ip_address = "localhost";
    std::string broker_public_key_path = "";
    std::string camera_type_str = "realsense"; // Default to RealSense

    if (argc > 1) {
        robot_id = argv[1];
    }
    if (argc > 2) {
        broker_ip_address = argv[2];
    }
    if (argc > 3) {
        broker_public_key_path = argv[3];
    }
    if (argc > 4) {
        camera_type_str = argv[4];
    }

    // Determine camera type
    CameraType camera_type = REALSENSE;
    if (camera_type_str == "random" || camera_type_str == "RANDOM") {
        camera_type = RANDOM;
    }
#ifdef ZED_AVAILABLE
    else if (camera_type_str == "zed" || camera_type_str == "ZED") {
        camera_type = ZED;
    }
#else
    else if (camera_type_str == "zed" || camera_type_str == "ZED") {
        std::cerr << "ZED camera requested but ZED SDK not available. Falling back to RealSense." << std::endl;
    }
#endif

    std::cout << "Starting Robot Client..." << std::endl;
    std::cout << "Robot ID: " << robot_id << std::endl;
    std::cout << "Broker IP address: " << broker_ip_address << std::endl;
    std::cout << "Camera type: " << (
        camera_type == RANDOM ? "Random" :
#ifdef ZED_AVAILABLE
        camera_type == ZED ? "ZED" : 
#endif
        "RealSense") << std::endl;

    RobotClient robot(robot_id, broker_ip_address, "pointcloud_tf", "register_robot", broker_public_key_path, camera_type);
    robot.run();

    return 0;
}