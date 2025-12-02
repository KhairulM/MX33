#include <msgpack.hpp>

namespace GetFrontiers {
    class Request {
        public:
            float x_min, x_max;
            float y_min, y_max;
            float z_min, z_max;

        MSGPACK_DEFINE(x_min, x_max, y_min, y_max, z_min, z_max);
    };

    class Response {
        public:
            std::vector<std::array<float, 3>> frontiers; // List of frontier points (x, y, z)

            MSGPACK_DEFINE(frontiers);
    };
}