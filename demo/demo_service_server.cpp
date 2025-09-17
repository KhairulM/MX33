#include <csignal>

#include "server.hpp"
#include "../srvs/string.hpp"

int main() {
    Server<String::Request, String::Response> server(
        "DemoServiceServer", 
        "tcp://localhost:5558", 
        "string_service",
        "143.248.159.7",
        "0"
        // "path/to/broker_public_key"
    );

    server.registerService();
    server.onRequestObject([](const String::Request& req) -> String::Response {
        String::Response res;
        res.hostname = req.hostname;
        res.data = "Hello, " + req.data + "!";
        res.success = true;
        return res;
    });
    server.run();

    return 0;
}