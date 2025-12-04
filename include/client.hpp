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
    std::string name;
    std::string service_name;
    zmq::context_t context;
    std::string service_lookup_address;
    zmq::socket_t service_lookup_socket;

    char broker_public_key[41] = {0};
    char public_key[41] = {0};
    char secret_key[41] = {0};

public:
    // Constructor
    Client(std::string name, std::string broker_ip_address, std::string service_name, std::string broker_public_key_path = "") {
        this->name = name;
        this->service_lookup_address = "tcp://" + broker_ip_address + ":5557";
        this->service_name = service_name;
        context = zmq::context_t(1);
        service_lookup_socket = zmq::socket_t(context, ZMQ_REQ);

        if (!broker_public_key_path.empty()) {
            std::ifstream pub_file(broker_public_key_path);
            if (pub_file.is_open()) {
                pub_file >> broker_public_key;
                zmq_curve_keypair(public_key, secret_key);
                service_lookup_socket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                service_lookup_socket.set(zmq::sockopt::curve_publickey, public_key);
                service_lookup_socket.set(zmq::sockopt::curve_secretkey, secret_key);
            } else {
                std::cerr << "[" << name << "] Warning: cannot open broker public key file: " << broker_public_key_path << std::endl;
            }
        }
    }

    ~Client() {
        service_lookup_socket.close();
        context.shutdown();
        context.close();
    }

    // Call service using stored service name. Throws std::runtime_error on failure.
    Response call(const Request& req) {
        // 1) Lookup service address from broker (blocking)
        service_lookup_socket.connect(service_lookup_address);
        service_lookup_socket.send(zmq::buffer(service_name), zmq::send_flags::none);

        zmq::message_t lookup_reply;
        auto ok = service_lookup_socket.recv(lookup_reply, zmq::recv_flags::none);
        if (!ok.has_value()) {
            throw std::runtime_error("[" + name + "] Lookup failed: no reply from broker");
        }

        std::string service_address(static_cast<char*>(lookup_reply.data()), lookup_reply.size());

        std::cout << "[" << name << "] Service " << service_name << " found at address: " << service_address << std::endl;
        if (service_address.empty()) {
            throw std::runtime_error("[" + name + "] Service not found: " + service_name);
        }

        // 2) Create a short-lived REQ socket to call the service
        zmq::socket_t service_socket(context, ZMQ_REQ);
        // If broker_public_key was provided we might need CURVE for service too; caller can extend if needed.
        if (broker_public_key[0] != '\0') {
            service_socket.set(zmq::sockopt::curve_serverkey, broker_public_key);
            service_socket.set(zmq::sockopt::curve_publickey, public_key);
            service_socket.set(zmq::sockopt::curve_secretkey, secret_key);
        }

        service_socket.connect(service_address);

        // Set receive timeout to 5 seconds
        service_socket.set(zmq::sockopt::rcvtimeo, 5000);

        // 3) Serialize and send request (blocking)
        msgpack::sbuffer sbuf;
        msgpack::pack(sbuf, req);
        service_socket.send(zmq::buffer(sbuf.data(), sbuf.size()), zmq::send_flags::none);

        // 4) Receive and unpack response (blocking with timeout)
        zmq::message_t resp_msg;
        auto recv_result = service_socket.recv(resp_msg, zmq::recv_flags::none);
        if (!recv_result.has_value()) {
            throw std::runtime_error("[" + name + "] Failed to receive response from service (timeout or error)");
        }

        msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(resp_msg.data()), resp_msg.size());
        Response resp;
        oh.get().convert(resp);

        // socket will be closed when service_socket goes out of scope
        return resp;
    }
};

#endif // CLIENT_HPP
