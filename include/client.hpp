#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>

#include <zmq.hpp>
#include <msgpack.hpp>

template<typename Request, typename Response>
class Client {
    std::string mLookupAddress;
    std::string mServiceName;
    zmq::context_t mContext;
    zmq::socket_t mLookupSocket;

    char broker_public_key[41] = {0};
    char public_key[41] = {0};
    char secret_key[41] = {0};

public:
    // Constructor
    Client(const std::string& lookup_address, const std::string& service_name, const std::string& broker_public_key_path = "")
        : mLookupAddress(lookup_address),
          mServiceName(service_name),
          mContext(1),
          mLookupSocket(mContext, ZMQ_REQ)
    {
        if (!broker_public_key_path.empty()) {
            std::ifstream pub_file(broker_public_key_path);
            if (pub_file.is_open()) {
                pub_file >> broker_public_key;
                zmq_curve_keypair(public_key, secret_key);
                mLookupSocket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                mLookupSocket.set(zmq::sockopt::curve_publickey, public_key);
                mLookupSocket.set(zmq::sockopt::curve_secretkey, secret_key);
            } else {
                std::cerr << "Warning: cannot open broker public key file: " << broker_public_key_path << std::endl;
            }
        }

        mLookupSocket.connect(mLookupAddress);
    }

    ~Client() {
        mLookupSocket.close();
        mContext.shutdown();
        mContext.close();
    }

    // Call service using stored service name. Throws std::runtime_error on failure.
    Response call(const Request& req) {
        // 1) Lookup service address from broker (blocking)
        mLookupSocket.send(zmq::buffer(mServiceName), zmq::send_flags::none);

        zmq::message_t lookup_reply;
        bool ok = mLookupSocket.recv(lookup_reply, zmq::recv_flags::none);
        if (!ok) {
            throw std::runtime_error("Lookup failed: no reply from broker");
        }

        std::string service_address(static_cast<char*>(lookup_reply.data()), lookup_reply.size());

        std::cout << "Service " << mServiceName << " found at address: " << service_address << std::endl;
        if (service_address.empty()) {
            throw std::runtime_error("Service not found: " + mServiceName);
        }

        // 2) Create a short-lived REQ socket to call the service
        zmq::socket_t serviceSock(mContext, ZMQ_REQ);
        // If broker_public_key was provided we might need CURVE for service too; caller can extend if needed.
        if (broker_public_key[0] != '\0') {
            serviceSock.set(zmq::sockopt::curve_serverkey, broker_public_key);
            serviceSock.set(zmq::sockopt::curve_publickey, public_key);
            serviceSock.set(zmq::sockopt::curve_secretkey, secret_key);
        }

        serviceSock.connect(service_address);

        // Set receive timeout to 5 seconds
        serviceSock.set(zmq::sockopt::rcvtimeo, 5000);

        // 3) Serialize and send request (blocking)
        msgpack::sbuffer sbuf;
        msgpack::pack(sbuf, req);
        serviceSock.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::none);

        // 4) Receive and unpack response (blocking with timeout)
        zmq::message_t resp_msg;
        auto recv_result = serviceSock.recv(resp_msg, zmq::recv_flags::none);
        if (!recv_result) {
            throw std::runtime_error("Failed to receive response from service (timeout or error)");
        }

        msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(resp_msg.data()), resp_msg.size());
        Response resp;
        oh.get().convert(resp);

        // socket will be closed when serviceSock goes out of scope
        return resp;
    }
};

#endif // CLIENT_HPP
