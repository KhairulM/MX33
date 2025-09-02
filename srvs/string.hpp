#include <string>
#include <msgpack.hpp>

namespace String {
    class Request {
        public:
            std::string hostname;
            std::string data;

            MSGPACK_DEFINE(hostname, data);
    };

    class Response {
        public:
            std::string hostname;
            std::string data;
            bool success = false;

            MSGPACK_DEFINE(hostname, data, success);
    };
}