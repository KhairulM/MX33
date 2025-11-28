#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>

#include <zmq.hpp>
#include <msgpack.hpp>

template<typename T>
class Subscriber {
    std::string name; // Name of the subscriber
    std::string topic; // Topic to subscribe to
    std::string broker_address; // IP address and port of the broker
    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the subscriber
    char secret_key[41]; // Secret key for the subscriber

    zmq::context_t context;
    zmq::socket_t socket;
    
    std::queue<zmq::message_t> message_queue;
    int max_message_queue_size = 45;

    std::thread thread;    
    std::mutex mutex;
    std::atomic<bool> is_stopped{false};

    void processZMQMessage() {
        while (!is_stopped.load()) {
            zmq::message_t topic, message;

            try {
                auto result_topic = socket.recv(topic, zmq::recv_flags::dontwait);
                if (!result_topic) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                auto result_msg = socket.recv(message, zmq::recv_flags::none);
                if (!result_msg) {
                    continue;
                }
            } catch (const zmq::error_t& e) {
                if (!is_stopped.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            std::lock_guard<std::mutex> lock(mutex);
            if (max_message_queue_size >= 0 && (int)message_queue.size() >= max_message_queue_size) {
                message_queue.pop(); // Remove the oldest message
            }
            message_queue.push(std::move(message));
        }
    }

    public:
        // Constructor
        Subscriber(std::string name, std::string broker_ip_address, std::string topic, std::string broker_public_key_path = "", int max_queue_size = 1) {
            this->name = name;
            this->broker_address = "tcp://" + broker_ip_address + ":5556";
            this->topic = topic;
            this->max_message_queue_size = max_queue_size;
            
            context = zmq::context_t(1);
            socket = zmq::socket_t(context, ZMQ_SUB);
            
            // Set high water mark to control queue size - only keep most recent messages
            socket.set(zmq::sockopt::rcvhwm, max_queue_size);

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
            socket.set(zmq::sockopt::subscribe, topic);
            
            thread = std::thread(&Subscriber::processZMQMessage, this);
        }

        // Destructor
        ~Subscriber() {
            stop();
            socket.close();
            context.close();
        }

        void stop() {
            is_stopped = true;  // Signal thread to stop
            if (thread.joinable()) {
                thread.join();  // Wait for thread to finish
            }
        }

        std::unique_ptr<T> getMessageObjectPtr() {
            std::lock_guard<std::mutex> lock(mutex);
            if (message_queue.empty()) {
                return nullptr;
            }

            zmq::message_t message = std::move(message_queue.front());
            message_queue.pop();

            // Deserialize the message with msgpack
            msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(message.data()), message.size());
            T message_data;
            oh.get().convert(message_data);

            return std::make_unique<T>(message_data);
        }
};

#endif // SUBSCRIBER_HPP