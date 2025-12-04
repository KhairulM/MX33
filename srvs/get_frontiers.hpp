#include <msgpack.hpp>

namespace GetFrontiers {
    class Request {
        public:
            float x_min, x_max; // x, y, z bounds for searching
            float y_min, y_max;
            float z_min, z_max;
            float min_radius_from_robot; // frontier points must be a radius away from the robots
            int occupied_neighbors_cell_dist; // there should be no occupied cells within cell_dist of the frontier points

        MSGPACK_DEFINE(x_min, x_max, y_min, y_max, z_min, z_max, min_radius_from_robot, occupied_neighbors_cell_dist);
    };

    class Response {
        public:
            std::vector<std::array<float, 3>> frontiers; // List of frontier points (x, y, z)

            MSGPACK_DEFINE(frontiers);
    };
}