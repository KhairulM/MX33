#pragma once
#include <string>
#include <msgpack.hpp>

#include "pointcloud.hpp"
#include "transforms.hpp"

class PointcloudTF {
    public:
        std::string robot_id;
        Pointcloud pointcloud; // assume in base_link frame
        Transform odom_to_base_link_transform;
        MSGPACK_DEFINE(robot_id, pointcloud, odom_to_base_link_transform);
};