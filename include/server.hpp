#ifndef SERVER_HPP
#define SERVER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <cerrno>
#include <functional>
#include <thread>

#include <zmq.hpp>
#include <msgpack.hpp>

// Utils (relative to this header's directory)
#include "../utils/get_local_ip.hpp"

// Change: Server now takes Request and Response types
template<typename Request, typename Response>
class Server {
    std::string name;
    std::string service_registry_address;
    std::string service_name;
    std::string service_ip_address;
    std::string service_port;

    // store user-provided handler
    std::function<Response(const Request&)> request_handler;
    
    bool is_registered = false;

    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the service server
    char secret_key[41]; // Secret key for the service server

    zmq::context_t context;
    zmq::socket_t service_reply_socket, service_registry_socket;

    public:
        // Constructor
        Server(
            std::string name, 
            std::string broker_ip_address, 
            std::string service_name,
            std::string service_ip_address = "", // if its empty, it will get the machine local IP
            std::string service_port = "0", // use ephemeral port by default 
            std::string broker_public_key_path = "") 
        {
            this->name = name;
            this->service_registry_address = "tcp://" + broker_ip_address + ":5558";
            
            if (service_ip_address.empty() || service_ip_address.length() == 0) {
                this->service_ip_address = get_local_ip();
            } else {
                this->service_ip_address = service_ip_address;
            }
            
            this->service_port = service_port;
            this->service_name = service_name;

            context = zmq::context_t(1);
            service_reply_socket = zmq::socket_t(context, ZMQ_REP);
            service_registry_socket = zmq::socket_t(context, ZMQ_REQ);

            if (!broker_public_key_path.empty()) {
                std::ifstream pub_file(broker_public_key_path);
                if (pub_file.is_open()) {
                    pub_file >> broker_public_key;
                    zmq_curve_keypair(public_key, secret_key);

                    service_registry_socket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                    service_registry_socket.set(zmq::sockopt::curve_publickey, public_key);                    
                    service_registry_socket.set(zmq::sockopt::curve_secretkey, secret_key);
                } else {
                    std::cerr << "[" << name << "] Error reading broker public key file.\n";
                }
            }
        }

        // Destructor
        ~Server() {
            // Attempt to unregister service
            unregisterService();

            service_reply_socket.close();
            service_registry_socket.close();
            context.shutdown();
            context.close();
        }


        void registerService() {
            service_registry_socket.connect(service_registry_address);

            // Bind the reply socket, if the service port is 0, zeromq will assign an ephemeral port
            service_reply_socket.bind("tcp://" + this->service_ip_address + ":" + this->service_port);

            // Get the final address
            std::string service_address = service_reply_socket.get(zmq::sockopt::last_endpoint);

            std::cout << "[" << name << "] Service " << service_name << " bound at address: " << service_address << std::endl;

            // Update the current IP address and the assigned port (if its an ephemeral port)
            this->service_ip_address = service_address.substr(service_address.find("://") + 3, service_address.rfind(":") - (service_address.find("://") + 3));
            this->service_port = service_address.substr(service_address.rfind(":") + 1);

            std::cout << "[" << name << "] Registering service " << service_name << " at " << service_address << std::endl;

            // Register the service name with the broker (use REGISTER command)
            std::string reg_msg = "REGISTER " + service_name + " " + service_address;
            service_registry_socket.send(zmq::buffer(reg_msg), zmq::send_flags::none);

            zmq::message_t reply;
            service_registry_socket.recv(reply, zmq::recv_flags::none);

            std::cout << "[" << name << "] Registered service " << service_name << " at " << service_address << std::endl;
            is_registered = true;
        }

        void unregisterService() {
            if (service_name.empty() || service_registry_address.empty() || !is_registered) {
                return; // nothing to unregister
            }

            // Ensure connected to broker (connect is idempotent)
            service_registry_socket.connect(service_registry_address);

            std::string msg = "UNREGISTER " + service_name;
            service_registry_socket.send(zmq::buffer(msg), zmq::send_flags::none);

            zmq::message_t reply;
            service_registry_socket.recv(reply, zmq::recv_flags::none);
            std::cout << "[" << name << "] Unregistered service " << service_name << std::endl;
            is_registered = false;
        }

        // Store user-defined handler that maps Request -> Response
        void onRequestObject(std::function<Response(const Request&)> handler) {
            request_handler = std::move(handler);
        }

        // Blocking run: loop checking for requests; on errors throw back to caller.
        void run(std::atomic<bool>* stop_flag = nullptr) {
            if (!request_handler) {
                throw std::runtime_error("["  + name + "] No message handler registered");
            }

            while (true) {
                // Check stop flag for graceful shutdown
                if (stop_flag && stop_flag->load()) {
                    std::cout << "[" << name << "] Stop signal received, shutting down..." << std::endl;
                    break;
                }

                zmq::message_t request;

                auto received = service_reply_socket.recv(request, zmq::recv_flags::dontwait);
                if (!received) {
                    std::this_thread::yield();
                    continue;
                }

                // Deserialize Request
                Request req_obj;
                {
                    msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(request.data()), request.size());
                    oh.get().convert(req_obj);
                }

                // Execute message handler 
                Response resp_obj = request_handler(req_obj);

                // Serialize Response
                msgpack::sbuffer sbuf;
                msgpack::pack(sbuf, resp_obj);

                // Send response non-blocking; on failure, throw an error to caller
                auto sent = service_reply_socket.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::dontwait);
                if (!sent) {
                    // Couldn't send now (would block) — propagate as a ZMQ error
                    unregisterService();
                    throw zmq::error_t(EAGAIN);
                }
            }

            // requested to stop: unregister before returning
            std::cout << "[" << name << "] Unregistering service before exit..." << std::endl;
            unregisterService();
        }
};

#endif // SERVER_HPP