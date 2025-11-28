#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <zmq.hpp>
#include <msgpack.hpp>

template<typename T>
class Publisher {
    std::string name; // Name of the publisher
    std::string topic; // Topic to publish to
    std::string broker_address; // IP address and port of the broker
    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the publisher
    char secret_key[41]; // Secret key for the publisher
    int queue_size; // Queue size for the publisher

    zmq::context_t context;
    zmq::socket_t socket;

    public:
        // Constructor
        Publisher(std::string name, std::string broker_ip_address, std::string topic, std::string broker_public_key_path = "", int queue_size = 1) {
            this->name = name;
            this->broker_address = "tcp://" + broker_ip_address + ":5555";
            this->queue_size = queue_size;
            
            context = zmq::context_t(1);
            socket = zmq::socket_t(context, ZMQ_PUB);
            
            // Set high water mark to control queue size
            socket.set(zmq::sockopt::sndhwm, queue_size);

            if (!broker_public_key_path.empty()) {
                std::ifstream pub_file(broker_public_key_path);
                if (pub_file.is_open()) {
                    pub_file >> broker_public_key;
                    zmq_curve_keypair(public_key, secret_key);

                    socket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                    socket.set(zmq::sockopt::curve_publickey, public_key);
                    socket.set(zmq::sockopt::curve_secretkey, secret_key);
                } else {
                    std::cerr << "Error reading broker public key file.\n";
                }
            }


            socket.connect(broker_address);
            this->topic = topic;
        }
        
        // Destructor
        ~Publisher() {
            socket.close();
            context.close();
        }

        void publish(const T& message) {
            // Serialize the message with msgpack
            msgpack::sbuffer buffer;
            msgpack::pack(buffer, message);

            // Create ZMQ message
            zmq::message_t topic_message(topic.data(), topic.size());
            zmq::message_t zmq_message(buffer.size());
            memcpy(zmq_message.data(), buffer.data(), buffer.size());

            // Publish the message
            socket.send(topic_message, zmq::send_flags::sndmore);
            socket.send(zmq_message, zmq::send_flags::none);
        }

};

#endif // PUBLISHER_HPP