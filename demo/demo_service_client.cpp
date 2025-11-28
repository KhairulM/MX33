#include <iostream>
#include "client.hpp" // relative include path; adjust if needed
#include "../srvs/string.hpp"

int main() {
    // NOTE: set this to your broker's service-name-lookup REP endpoint
    const std::string broker_lookup_address = "localhost"; // <- adjust to your broker's lookup address

    Client<String::Request, String::Response> client("demo_client", broker_lookup_address, "string_service");

    String::Request req;
    req.hostname = "client_host";
    req.data = "World";

    try {
        String::Response res = client.call(req);
        std::cout << "Response: success=" << res.success << ", hostname=" << res.hostname << ", data='" << res.data << "'" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "RPC error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
