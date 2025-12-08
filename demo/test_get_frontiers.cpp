#include <iostream>
#include <iomanip>
#include "client.hpp"
#include "../srvs/get_frontiers.hpp"

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string broker_address = "localhost";
    float x_min = -10.0f, x_max = 10.0f;
    float y_min = -10.0f, y_max = 10.0f;
    float z_min = -1.0f, z_max = 3.0f;
    float min_radius_from_robot = 0.5f;
    int occupied_neighbors_cell_dist = 1;

    if (argc > 1) {
        broker_address = argv[1];
    }
    if (argc > 7) {
        x_min = std::stof(argv[2]);
        x_max = std::stof(argv[3]);
        y_min = std::stof(argv[4]);
        y_max = std::stof(argv[5]);
        z_min = std::stof(argv[6]);
        z_max = std::stof(argv[7]);
    }
    if (argc > 8) {
        min_radius_from_robot = std::stof(argv[8]);
    }
    if (argc > 9) {
        occupied_neighbors_cell_dist = std::stoi(argv[9]);
    }

    std::cout << "=== Get Frontiers Service Test ===" << std::endl;
    std::cout << "Broker address: " << broker_address << std::endl;
    std::cout << "Bounding box:" << std::endl;
    std::cout << "  X: [" << x_min << ", " << x_max << "]" << std::endl;
    std::cout << "  Y: [" << y_min << ", " << y_max << "]" << std::endl;
    std::cout << "  Z: [" << z_min << ", " << z_max << "]" << std::endl;
    std::cout << "Min radius from robot: " << min_radius_from_robot << std::endl;
    std::cout << "Occupied neighbors cell dist: " << occupied_neighbors_cell_dist << std::endl;
    std::cout << std::endl;

    // Create client
    Client<GetFrontiers::Request, GetFrontiers::Response> client(
        "test_get_frontiers_client",
        broker_address,
        "get_frontiers"
    );

    // Prepare request
    GetFrontiers::Request req;
    req.x_min = x_min;
    req.x_max = x_max;
    req.y_min = y_min;
    req.y_max = y_max;
    req.z_min = z_min;
    req.z_max = z_max;
    req.min_radius_from_robot = min_radius_from_robot;
    req.occupied_neighbors_cell_dist = occupied_neighbors_cell_dist;

    try {
        std::cout << "Calling get_frontiers service..." << std::endl;
        GetFrontiers::Response res = client.call(req);
        
        std::cout << "\n=== Response Received ===" << std::endl;
        std::cout << "Number of frontier points: " << res.frontiers.size() << std::endl;
        
        if (res.frontiers.empty()) {
            std::cout << "No frontiers found in the specified region." << std::endl;
        } else {
            std::cout << "\nFrontier points (x, y, z):" << std::endl;
            std::cout << std::fixed << std::setprecision(3);
            
            // Display first 20 frontiers
            int display_count = std::min(static_cast<int>(res.frontiers.size()), 20);
            for (int i = 0; i < display_count; i++) {
                const auto& point = res.frontiers[i];
                std::cout << "  [" << std::setw(3) << i << "] "
                         << "(" << std::setw(7) << point[0] 
                         << ", " << std::setw(7) << point[1]
                         << ", " << std::setw(7) << point[2] << ")" << std::endl;
            }
            
            if (res.frontiers.size() > 20) {
                std::cout << "  ... and " << (res.frontiers.size() - 20) 
                         << " more frontier points." << std::endl;
            }
            
            // Calculate bounding box of frontiers
            if (!res.frontiers.empty()) {
                float min_x = res.frontiers[0][0], max_x = res.frontiers[0][0];
                float min_y = res.frontiers[0][1], max_y = res.frontiers[0][1];
                float min_z = res.frontiers[0][2], max_z = res.frontiers[0][2];
                
                for (const auto& point : res.frontiers) {
                    min_x = std::min(min_x, point[0]);
                    max_x = std::max(max_x, point[0]);
                    min_y = std::min(min_y, point[1]);
                    max_y = std::max(max_y, point[1]);
                    min_z = std::min(min_z, point[2]);
                    max_z = std::max(max_z, point[2]);
                }
                
                std::cout << "\nFrontier statistics:" << std::endl;
                std::cout << "  X range: [" << min_x << ", " << max_x << "]" << std::endl;
                std::cout << "  Y range: [" << min_y << ", " << max_y << "]" << std::endl;
                std::cout << "  Z range: [" << min_z << ", " << max_z << "]" << std::endl;
            }
        }
        
        std::cout << "\n=== Test Completed Successfully ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\n=== Error ===" << std::endl;
        std::cerr << "Failed to call get_frontiers service: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
