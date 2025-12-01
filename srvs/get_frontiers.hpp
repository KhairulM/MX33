namespace GetFrontiers {
    class Request {
        public:
            std::string robot_id = "";
            float radius = 0.0f;

        MSGPACK_DEFINE(robot_id, radius);
    };

    class Response {
        public:
            std::vector<std::array<float, 3>> frontiers; // List of frontier points (x, y, z)

            MSGPACK_DEFINE(frontiers);
    };
}