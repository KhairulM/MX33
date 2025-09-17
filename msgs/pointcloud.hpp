#pragma once
#include <string>
#include <msgpack.hpp>

class Pointcloud {
    public:
        int width;
        int height;
        std::vector<float> pointcloud_data;
        MSGPACK_DEFINE(width, height, pointcloud_data);
};