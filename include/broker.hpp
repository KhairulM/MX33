#ifndef BROKER_HPP
#define BROKER_HPP

#include <zmq.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <future>
#include <thread>


class Broker {
    std::string mFrontendAddress, mBackendAddress;
    std::string mServiceNameRegistryLookupAddress, mServiceNameRegistryAddAddress;
    std::map<std::string, std::string> mServiceNameRegistry;

    char private_key[41];

    public:
        Broker(
            std::string frontend_address, 
            std::string backend_address, 
            std::string service_name_registry_lookup_address,
            std::string service_name_registry_add_address,
            std::string secret_key_path = ""
        ) {
            mFrontendAddress = frontend_address;
            mBackendAddress = backend_address;
            mServiceNameRegistryLookupAddress = service_name_registry_lookup_address;
            mServiceNameRegistryAddAddress = service_name_registry_add_address;

            if (secret_key_path.empty()) {
                return;
            }

            std::ifstream secret_file(secret_key_path);

            if (secret_file.is_open()) {
                secret_file >> private_key;
            } else {
                std::cerr << "Error reading key file.\n";
            }
        }


        void run() {
            // Create a ZMQ context
            zmq::context_t context(1);

            std::cout << "Broker:" << std::endl;
            std::cout << "  Frontend address: " << mFrontendAddress.c_str() << std::endl;
            std::cout << "  Backend address: " << mBackendAddress.c_str() << std::endl;
            std::cout << "  Service name registry lookup address: " << mServiceNameRegistryLookupAddress.c_str() << std::endl;
            std::cout << "  Service name registry add address: " << mServiceNameRegistryAddAddress.c_str() << std::endl;

            // Run the forwarder and the service name registry in separate threads
            auto thread1 = std::async(std::launch::async, &Broker::run_forwarder, this, std::ref(context));
            auto thread2 = std::async(std::launch::async, &Broker::run_service_name_registry_lookup, this, std::ref(context));
            auto thread3 = std::async(std::launch::async, &Broker::run_service_name_registry_add, this, std::ref(context));

            thread1.wait();
            thread2.wait();
            thread3.wait();

            context.shutdown();
            context.close();
        }

        void run_forwarder(zmq::context_t& context) {
            // Create a ZMQ socket for the frontend (The one that will receive messages from subscribers)
            zmq::socket_t frontend(context, ZMQ_XSUB);
            if (private_key[0] != '\0') {
                frontend.set(zmq::sockopt::curve_server, 1);
                frontend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            frontend.bind(mFrontendAddress);

            // Create a ZMQ socket for the backend (The one that will send messages to publishers)
            zmq::socket_t backend(context, ZMQ_XPUB);
            if (private_key[0] != '\0') {
                backend.set(zmq::sockopt::curve_server, 1);
                backend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            backend.bind(mBackendAddress);
            
            std::cout << "Broker started, XSUB: " << mFrontendAddress << ", XPUB:  " << mBackendAddress << std::endl;

            // Start the proxy
            zmq::proxy(frontend, backend);

            // Close the sockets
            frontend.close();
            backend.close();
        }

        void run_service_name_registry_lookup(zmq::context_t& context) {
            // Create a REP socket to accepts requests for service names
            zmq::socket_t service_registry_socket(context, ZMQ_REP);
            service_registry_socket.bind(mServiceNameRegistryLookupAddress);

            while (true) {
                zmq::message_t request;
                if (!service_registry_socket.recv(request, zmq::recv_flags::dontwait)) {
                    // If no message is received, continue the loop
                    continue;
                }
                std::string service_name(static_cast<char*>(request.data()), request.size());

                // Look up the service name in the registry
                auto it = mServiceNameRegistry.find(service_name);
                if (it != mServiceNameRegistry.end()) {
                    // If found, send the address back to the requester
                    zmq::message_t reply(it->second.data(), it->second.size());
                    service_registry_socket.send(reply, zmq::send_flags::none);
                } else {
                    // If not found, send an empty reply
                    zmq::message_t reply;
                    service_registry_socket.send(reply, zmq::send_flags::none);
                }
            }

            // Close the socket
            service_registry_socket.close();
        }

        void run_service_name_registry_add(zmq::context_t& context) {
            // Create a REP socket to accepts requests for adding service names
            zmq::socket_t service_registry_socket(context, ZMQ_REP);
            service_registry_socket.bind(mServiceNameRegistryAddAddress);

            while (true) {
                zmq::message_t registration;
                if (!service_registry_socket.recv(registration, zmq::recv_flags::dontwait)) {
                    // If no message is received, continue the loop
                    continue;
                }

                std::string registration_str(static_cast<char*>(registration.data()), registration.size());

                // Split the registration string by whitespace
                // Service name is the 2nd part
                // Service address is the 3rd part
                std::istringstream iss(registration_str);
                std::string part;
                std::vector<std::string> parts;
                while (iss >> part) {
                    parts.push_back(part);
                }
                std::string service_name_str = parts.size() > 0 ? parts[0] : "";
                std::string address_str = parts.size() > 1 ? parts[1] : "";

                // Add the service name to the registry
                mServiceNameRegistry[service_name_str] = address_str;

                // Send an empty reply
                zmq::message_t reply;
                service_registry_socket.send(reply, zmq::send_flags::none);
            }

            // Close the socket
            service_registry_socket.close();
        }
};

#endif // BROKER_HPP