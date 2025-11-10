#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include "../msgs/transforms.hpp"

class Robot {
    public:
        std::string id;
        std::string ip_address;
        Transform global_to_odom_transform;

        bool connected = false;

};

#endif // ROBOT_HPP