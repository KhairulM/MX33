#include <string>
#include <msgpack.hpp>

class Pointcloud {
    public:
        std::string hostname;
        int width;
        int height;
        std::vector<float> pointcloud_data;
        MSGPACK_DEFINE(hostname, width, height, pointcloud_data);
};