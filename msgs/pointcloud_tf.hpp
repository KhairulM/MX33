#pragma once
#include <string>
#include <msgpack.hpp>

#include "pointcloud.hpp"
#include "transforms.hpp"

class PointcloudTF {
    public:
        std::string robot_id;
        Pointcloud pointcloud;
        Transform local_to_camera_transform;
        MSGPACK_DEFINE(robot_id, pointcloud, local_to_camera_transform);
};