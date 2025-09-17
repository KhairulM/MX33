#include <string>
#include <msgpack.hpp>

namespace RegisterRobot {
    class Request {
        public:
            std::string id;
            std::string ip_address;

        MSGPACK_DEFINE(id, ip_address);
    };

    class Response {
        public:
            bool success = false;
            std::string message;

            MSGPACK_DEFINE(success, message);
    };
}