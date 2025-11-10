#pragma once
#include <string>
#include <msgpack.hpp>

#include "transforms.hpp"

class PoseStamped {
    public:
        std::string timestamp; // ISO 8601 format 2025-11-10T14:23:45.123Z
        Transform odom_to_base_link;
        MSGPACK_DEFINE(timestamp, odom_to_base_link);
};