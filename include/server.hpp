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
    std::string nName;
    std::string mRegistryAddress;
    std::string mServiceName;
    std::string mServiceAddress;
    std::string mServiceIpAddress;
    std::string mServicePort;

    // store user-provided handler
    std::function<Response(const Request&)> mHandler;

    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the service server
    char secret_key[41]; // Secret key for the service server

    zmq::context_t mContext;
    zmq::socket_t mRepSocket, mRegisterServiceSocket;

    public:
        // Constructor
        Server(
            std::string name, 
            std::string proxy_address, 
            std::string service_name,
            std::string server_ip_address = "",
            std::string service_port = "0", // use ephemeral port by default 
            std::string broker_public_key_path = "") {
            nName = name;
            mRegistryAddress = proxy_address;
            mServiceName = service_name;
            mServiceIpAddress = get_local_ip();
            mServicePort = service_port; // use ephemeral port by default

            mContext = zmq::context_t(1);
            mRepSocket = zmq::socket_t(mContext, ZMQ_REP);
            mRegisterServiceSocket = zmq::socket_t(mContext, ZMQ_REQ);

            if (!broker_public_key_path.empty()) {
                std::ifstream pub_file(broker_public_key_path);
                if (pub_file.is_open()) {
                    pub_file >> broker_public_key;
                    zmq_curve_keypair(public_key, secret_key);

                    mRegisterServiceSocket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                    mRegisterServiceSocket.set(zmq::sockopt::curve_publickey, public_key);                    
                    mRegisterServiceSocket.set(zmq::sockopt::curve_secretkey, secret_key);
                } else {
                    std::cerr << "Error reading broker public key file.\n";
                }
            }
        }

        // Destructor
        ~Server() {
            // Attempt to unregister service
            unregisterService();

            mRepSocket.close();
            mRegisterServiceSocket.close();
            mContext.shutdown();
            mContext.close();
        }


        void registerService() {
            mRegisterServiceSocket.connect(mRegistryAddress);

            // Bind the socket, if the service port is 0, zeromq will assign an ephemeral port
            mRepSocket.bind("tcp://" + mServiceIpAddress + ":" + mServicePort);

            // Get the final address
            mServiceAddress = mRepSocket.get(zmq::sockopt::last_endpoint);

            std::cout << "Service " << mServiceName << " bound at address: " << mServiceAddress << std::endl;

            // Update the current IP address and the assigned port (if its an ephemeral port)
            mServiceIpAddress = mServiceAddress.substr(mServiceAddress.find("://") + 3, mServiceAddress.rfind(":") - (mServiceAddress.find("://") + 3));
            mServicePort = mServiceAddress.substr(mServiceAddress.rfind(":") + 1);

            // Register the service name with the broker (use REGISTER command)
            std::string reg_msg = "REGISTER " + mServiceName + " " + mServiceAddress;
            mRegisterServiceSocket.send(zmq::buffer(reg_msg), zmq::send_flags::none);
            
            std::cout << "Registering service " << mServiceName << " at " << mServiceAddress << std::endl;

            zmq::message_t reply;
            mRegisterServiceSocket.recv(reply, zmq::recv_flags::none);
            std::cout << "Registered service " << mServiceName << " at " << mServiceAddress << std::endl;
        }

        void unregisterService() {
            if (mServiceName.empty() || mRegistryAddress.empty()) {
                return; // nothing to unregister
            }

            // Ensure connected to broker (connect is idempotent)
            mRegisterServiceSocket.connect(mRegistryAddress);

            std::string msg = "UNREGISTER " + mServiceName;
            mRegisterServiceSocket.send(zmq::buffer(msg), zmq::send_flags::none);

            zmq::message_t reply;
            mRegisterServiceSocket.recv(reply, zmq::recv_flags::none);
            std::cout << "Unregistered service " << mServiceName << std::endl;
        }

        // Store user-defined handler that maps Request -> Response
        void onRequestObject(std::function<Response(const Request&)> handler) {
            mHandler = std::move(handler);
        }

        // Blocking run: loop checking for requests; on errors throw back to caller.
        void run(std::atomic<bool>* stop_flag = nullptr) {
            if (!mHandler) {
                throw std::runtime_error("No message handler registered");
            }

            while (true) {
                // Check stop flag for graceful shutdown
                if (stop_flag && stop_flag->load()) {
                    std::cout << "[" << nName << "] Stop signal received, shutting down..." << std::endl;
                    break;
                }

                zmq::message_t request;

                auto received = mRepSocket.recv(request, zmq::recv_flags::dontwait);
                if (!received) {
                    std::this_thread::yield();
                    continue;
                }

                std::cout << "Received request of size " << request.size() << " bytes" << std::endl;

                // Deserialize Request
                Request req_obj;
                {
                    msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(request.data()), request.size());
                    oh.get().convert(req_obj);
                }

                // Execute handler 
                Response resp_obj = mHandler(req_obj);

                // Serialize Response
                msgpack::sbuffer sbuf;
                msgpack::pack(sbuf, resp_obj);

                // Send response non-blocking; on failure, throw an error to caller
                auto sent = mRepSocket.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::dontwait);
                if (!sent) {
                    // Couldn't send now (would block) — propagate as a ZMQ error
                    unregisterService();
                    throw zmq::error_t(EAGAIN);
                }
            }

            // requested to stop: unregister before returning
            std::cout << "[" << nName << "] Unregistering service before exit..." << std::endl;
            unregisterService();
        }
};

#endif // SERVER_HPP