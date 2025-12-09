#include "broker.hpp"
#include <csignal>
#include <atomic>
#include <yaml-cpp/yaml.h>

static Broker* gBrokerInstance = nullptr;

void handleSigInt(int){
    if (gBrokerInstance) {
        std::cout << "\nSIGINT received, shutting down broker..." << std::endl;
        gBrokerInstance->stop();
    }
}

int main(int argc, char* argv[]) {
    // Default config file path
    std::string config_file = "config.yaml";
    
    // Allow override via command line
    if (argc > 1) {
        config_file = argv[1];
    }

    // Load configuration
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (const std::exception& e) {
        std::cerr << "[Broker] Failed to load config file: " << e.what() << std::endl;
        std::cerr << "[Broker] Using default values" << std::endl;
    }

    // Read broker configuration with defaults
    int frontend_port = config["broker"]["frontend_port"].as<int>(5555);
    int backend_port = config["broker"]["backend_port"].as<int>(5556);
    int service_registry_lookup_port = config["broker"]["service_registry_lookup_port"].as<int>(5557);
    int service_registry_add_port = config["broker"]["service_registry_add_port"].as<int>(5558);

    std::string frontend_address = "tcp://*:" + std::to_string(frontend_port);
    std::string backend_address = "tcp://*:" + std::to_string(backend_port);
    std::string service_registry_lookup_address = "tcp://*:" + std::to_string(service_registry_lookup_port);
    std::string service_registry_add_address = "tcp://*:" + std::to_string(service_registry_add_port);

    std::cout << "[Broker] Configuration loaded from: " << config_file << std::endl;
    std::cout << "[Broker] Frontend port: " << frontend_port << std::endl;
    std::cout << "[Broker] Backend port: " << backend_port << std::endl;

    // Create a new Broker instance
    Broker broker(
        frontend_address, 
        backend_address, 
        service_registry_lookup_address,
        service_registry_add_address
    );

    gBrokerInstance = &broker;
    std::signal(SIGINT, handleSigInt);

    broker.run();
    
    return 0;
}