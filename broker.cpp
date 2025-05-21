#include "broker.hpp"

int main(int argc, char* argv[]) {
    char* frontend_port;
    char* backend_port;

    if (argc == 3) {
        frontend_port = argv[1];
        backend_port = argv[2];
    } else if (argc == 2) {
        frontend_port = argv[1];
        backend_port = "5556"; // Default backend port
    } else {
        frontend_port = "5555"; // Default frontend port
        backend_port = "5556"; // Default backend port
    }

    std::string frontend_address = "tcp://*:" + std::string(frontend_port);
    std::string backend_address = "tcp://*:" + std::string(backend_port);

    // Create a new Broker instance
    Broker broker(
        frontend_address, 
        backend_address, 
        "/home/control/Work/MX33/curve/broker_secret.key"
    );

    // Run the broker
    broker.run();
    
    return 0;
}