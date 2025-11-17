#include <string>
#include <msgpack.hpp>

namespace RegisterRobot {
    class Request {
        public:
            std::string id;
            std::string ip_address;
            std::string set_goal_topic;

        MSGPACK_DEFINE(id, ip_address, set_goal_topic);
    };

    class Response {
        public:
            bool success = false;
            std::string message;

            MSGPACK_DEFINE(success, message);
    };
}