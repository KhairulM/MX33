#include <iostream>
#include "client.hpp" // relative include path; adjust if needed
#include "../srvs/string.hpp"

int main() {
    // NOTE: set this to your broker's service-name-lookup REP endpoint
    const std::string broker_lookup_address = "tcp://localhost:5557"; // <- adjust to your broker's lookup address

    Client<String::Request, String::Response> client(broker_lookup_address, "string_service" /*, "path/to/broker_pubkey"*/);

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
