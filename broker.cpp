#include "broker.hpp"

int main(int argc, char* argv[]) {
    char* frontend_port = "5555"; // Default frontend port
    char* backend_port = "5556"; // Default backend port
    char* service_registry_lookup_port = "5557"; // Default service registry lookup port
    char* service_registry_add_port = "5558"; // Default service registry add port

    std::string frontend_address = "tcp://*:" + std::string(frontend_port);
    std::string backend_address = "tcp://*:" + std::string(backend_port);
    std::string service_registry_lookup_address = "tcp://*:" + std::string(service_registry_lookup_port);
    std::string service_registry_add_address = "tcp://*:" + std::string(service_registry_add_port);

    // Create a new Broker instance
    Broker broker(
        frontend_address, 
        backend_address, 
        service_registry_lookup_address,
        service_registry_add_address
        // "/home/control/Work/MX33/curve/broker_secret.key"
    );

    // Run the broker
    broker.run();
    
    return 0;
}