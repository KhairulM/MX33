#ifndef BROKER_HPP
#define BROKER_HPP

#include <zmq.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>


class Broker {
    std::string mFrontendAddress, mBackendAddress;
    std::string mServiceNameRegistryLookupAddress, mServiceNameRegistryAddAddress;
    std::map<std::string, std::string> mServiceNameRegistry;
    std::mutex mRegistryMutex; // protect mServiceNameRegistry

    std::atomic<bool> mRunning{true};

    char private_key[41] = {0};

    public:
        Broker(
            std::string frontend_address,
            std::string backend_address,
            std::string service_name_registry_lookup_address,
            std::string service_name_registry_add_address,
            std::string secret_key_path = ""
        ) :
            mFrontendAddress(std::move(frontend_address)),
            mBackendAddress(std::move(backend_address)),
            mServiceNameRegistryLookupAddress(std::move(service_name_registry_lookup_address)),
            mServiceNameRegistryAddAddress(std::move(service_name_registry_add_address))
        {
            if (!secret_key_path.empty()) {
                std::ifstream secret_file(secret_key_path);
                if (secret_file.is_open()) {
                    secret_file >> private_key;
                } else {
                    std::cerr << "Error reading key file.\n";
                }
            }
        }

        void stop() { mRunning = false; }
        bool isRunning() const { return mRunning.load(); }

        void run() {
            zmq::context_t context(1);

            std::cout << "Broker:" << std::endl;
            std::cout << "  Frontend address: " << mFrontendAddress << std::endl;
            std::cout << "  Backend address: " << mBackendAddress << std::endl;
            std::cout << "  Service name registry lookup address: " << mServiceNameRegistryLookupAddress << std::endl;
            std::cout << "  Service name registry modify address: " << mServiceNameRegistryAddAddress << std::endl;

            std::thread tForwarder(&Broker::runForwarder, this, std::ref(context));
            std::thread tLookup(&Broker::runServiceNameRegistryLookup, this, std::ref(context));
            std::thread tModify(&Broker::runServiceNameRegistryModify, this, std::ref(context));

            // Wait for the threads to finish (they end when stop() is called)
            tForwarder.join();
            tLookup.join();
            tModify.join();

            context.shutdown();
            context.close();
        }

    private:
        void forwardMultipart(zmq::socket_t &from, zmq::socket_t &to) {
            while (true) {
                zmq::message_t msg;
                if (!from.recv(msg, zmq::recv_flags::none)) {
                    return; // interrupted/failed
                }
                bool more = from.get(zmq::sockopt::rcvmore);
                to.send(msg, more ? zmq::send_flags::sndmore : zmq::send_flags::none);
                if (!more) break;
            }
        }

        void runForwarder(zmq::context_t& context) {
            zmq::socket_t frontend(context, ZMQ_XSUB);
            if (private_key[0] != '\0') {
                frontend.set(zmq::sockopt::curve_server, 1);
                frontend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            frontend.bind(mFrontendAddress);

            zmq::socket_t backend(context, ZMQ_XPUB);
            if (private_key[0] != '\0') {
                backend.set(zmq::sockopt::curve_server, 1);
                backend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            backend.bind(mBackendAddress);

            // Manual proxy loop so we can terminate gracefully
            while (mRunning.load()) {
                zmq::pollitem_t items[] = {
                    { static_cast<void*>(frontend), 0, ZMQ_POLLIN, 0 },
                    { static_cast<void*>(backend), 0, ZMQ_POLLIN, 0 }
                };
                // poll with timeout to check mRunning periodically
                try {
                    zmq::poll(items, 2, std::chrono::milliseconds(100));
                } catch (const zmq::error_t &e) {
                    if (!mRunning.load()) break; // likely interrupted
                    std::cerr << "poll error: " << e.what() << std::endl;
                }

                if (items[0].revents & ZMQ_POLLIN) {
                    forwardMultipart(frontend, backend);
                }
                if (items[1].revents & ZMQ_POLLIN) {
                    forwardMultipart(backend, frontend);
                }
            }

            frontend.close();
            backend.close();
            std::cout << "Forwarder thread exiting" << std::endl;
        }

        void runServiceNameRegistryLookup(zmq::context_t& context) {
            zmq::socket_t serviceRegistrySocket(context, ZMQ_REP);
            serviceRegistrySocket.bind(mServiceNameRegistryLookupAddress);

            while (mRunning.load()) {
                zmq::message_t request;
                if (!serviceRegistrySocket.recv(request, zmq::recv_flags::dontwait)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                std::string service_name(static_cast<char*>(request.data()), request.size());

                std::string address;
                {
                    std::lock_guard<std::mutex> lock(mRegistryMutex);
                    auto it = mServiceNameRegistry.find(service_name);
                    if (it != mServiceNameRegistry.end()) address = it->second;
                }
                if (!address.empty()) {
                    zmq::message_t reply(address.data(), address.size());
                    serviceRegistrySocket.send(reply, zmq::send_flags::none);
                } else {
                    zmq::message_t reply; // empty
                    serviceRegistrySocket.send(reply, zmq::send_flags::none);
                }
            }
            serviceRegistrySocket.close();
            std::cout << "Lookup thread exiting" << std::endl;
        }

        void runServiceNameRegistryModify(zmq::context_t& context) {
            zmq::socket_t serviceRegistrySocket(context, ZMQ_REP);
            serviceRegistrySocket.bind(mServiceNameRegistryAddAddress);

            while (mRunning.load()) {
                zmq::message_t registration;
                if (!serviceRegistrySocket.recv(registration, zmq::recv_flags::dontwait)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                std::string registration_str(static_cast<char*>(registration.data()), registration.size());
                std::istringstream iss(registration_str);
                std::string cmd;
                iss >> cmd;

                if (cmd == "REGISTER") {
                    std::string service_name_str;
                    std::string address_str;
                    iss >> service_name_str >> address_str;
                    if (!service_name_str.empty() && !address_str.empty()) {
                        {
                            std::lock_guard<std::mutex> lock(mRegistryMutex);
                            mServiceNameRegistry[service_name_str] = address_str;
                        }
                        std::cout << "Registered service: " << service_name_str << " at " << address_str << std::endl;
                    } else {
                        std::cerr << "Malformed REGISTER message: '" << registration_str << "'" << std::endl;
                    }
                } else if (cmd == "UNREGISTER") {
                    std::string service_name_str; iss >> service_name_str;
                    if (!service_name_str.empty()) {
                        bool erased = false;
                        {
                            std::lock_guard<std::mutex> lock(mRegistryMutex);
                            auto it = mServiceNameRegistry.find(service_name_str);
                            if (it != mServiceNameRegistry.end()) { mServiceNameRegistry.erase(it); erased = true; }
                        }
                        if (erased) {
                            std::cout << "Unregistered service: " << service_name_str << std::endl;
                        } else {
                            std::cout << "Service to unregister not found: " << service_name_str << std::endl;
                        }
                    } else {
                        std::cerr << "Malformed UNREGISTER message: '" << registration_str << "'" << std::endl;
                    }
                } else {
                    std::cerr << "Unknown service name registry command: '" << registration_str << "'" << std::endl;
                }

                zmq::message_t reply; // empty ack
                serviceRegistrySocket.send(reply, zmq::send_flags::none);
            }
            serviceRegistrySocket.close();
            std::cout << "Modify thread exiting" << std::endl;
        }
};

#endif // BROKER_HPP