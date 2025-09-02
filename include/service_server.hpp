#ifndef SERVICE_SERVER_HPP
#define SERVICE_SERVER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <zmq.hpp>
#include <msgpack.hpp>

template<typename T>
class ServiceServer {
    std::string nName;
    std::string mProxyAddress;
    std::string mServiceName;
    std::string mServiceAddress;
    std::string mServiceIpAddress;
    std::string mServicePort;

    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the publisher
    char secret_key[41]; // Secret key for the publisher

    zmq::context_t mContext;
    zmq::socket_t mRepSocket, mRegisterServiceSocket;

    public:
        // Constructor
        ServiceServer(
            std::string name, 
            std::string proxy_address, 
            std::string service_name, 
            std::string service_ip_addr = "*",
            std::string service_port = "0",
            std::string broker_public_key_path = "") {
            nName = name;
            mProxyAddress = proxy_address;
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

            mRepSocket.bind("tcp://" + mServiceIpAddress + ":" + mServicePort); // Bind to a random available port
            mServicePort = std::to_string(mRepSocket.get(zmq::sockopt::last_endpoint));
            
            // Get the current machine IP address and the assigned port
            mServiceIpAddress = "tcp://" + mServiceIpAddress + ":" + mServicePort;
            
            // Register the service name with the broker
            if (!broker_public_key_path.empty()) {
                mRegisterServiceSocket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                mRegisterServiceSocket.set(zmq::sockopt::curve_publickey, public_key);
                mRegisterServiceSocket.set(zmq::sockopt::curve_secretkey, secret_key);
            }
            mRegisterServiceSocket.connect(mProxyAddress);

            mRegisterServiceSocket.send(zmq::buffer(mServiceName + " " + mServiceIpAddress + ":" + mServicePort));
            zmq::message_t reply;
            mRegisterServiceSocket.recv(reply, zmq::recv_flags::none);
            std::cout << "Received reply: " << reply.to_string() << std::endl;
        }
};

#endif // SERVICE_SERVER_HPP