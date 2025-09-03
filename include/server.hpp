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
            std::string service_ip_addr = "*",
            std::string service_port = "0",
            std::string broker_public_key_path = "") {
            nName = name;
            mRegistryAddress = proxy_address;
            mServiceName = service_name;
            mServiceIpAddress = service_ip_addr;
            mServicePort = service_port;

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
            // Bind the socket
            mRepSocket.bind("tcp://" + mServiceIpAddress + ":" + mServicePort);

            // Get the final address
            mServiceAddress = mRepSocket.get(zmq::sockopt::last_endpoint);

            // Update the current IP address and the assigned port (if its an ephemeral port)
            mServiceIpAddress = mServiceAddress.substr(mServiceAddress.find("://") + 3, mServiceAddress.rfind(":") - (mServiceAddress.find("://") + 3));
            mServicePort = mServiceAddress.substr(mServiceAddress.rfind(":") + 1);

            // Register the service name with the broker (use REGISTER command)
            mRegisterServiceSocket.connect(mRegistryAddress);
            {
                std::string reg_msg = "REGISTER " + mServiceName + " " + mServiceAddress;
                mRegisterServiceSocket.send(zmq::buffer(reg_msg), zmq::send_flags::none);
            }
            
            zmq::message_t reply;
            mRegisterServiceSocket.recv(reply, zmq::recv_flags::none);
            std::cout << "Registered service " << mServiceName << " at " << mServiceAddress << std::endl;
        }

        // New: unregister the service from the broker (sends "UNREGISTER <service_name>")
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

        // Non-blocking run: loop checking for requests; on errors throw back to caller.
        void run() {
            if (!mHandler) {
                throw std::runtime_error("No handler registered");
            }

            while (true) {
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
            unregisterService();
        }
};

#endif // SERVER_HPP