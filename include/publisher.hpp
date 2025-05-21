#include <iostream>
#include <fstream>
#include <string>
#include <zmq.hpp>
#include <msgpack.hpp>

template<typename T>
class Publisher {
    std::string mName; // Name of the publisher
    std::string mProxyAddress; // Proxy address written in the format of "tcp://hostname:port"
    std::string mTopic; // Topic to publish to
    char broker_public_key[41]; // Public key for the broker
    char public_key[41]; // Public key for the publisher
    char secret_key[41]; // Secret key for the publisher

    zmq::context_t mContext;
    zmq::socket_t mSocket;

    public:
        // Constructor
        Publisher(std::string name, std::string proxy_address, std::string topic, std::string broker_public_key_path = "") {
            mName = name;
            mProxyAddress = proxy_address;
            mContext = zmq::context_t(1);
            mSocket = zmq::socket_t(mContext, ZMQ_PUB);

            if (!broker_public_key_path.empty()) {
                std::ifstream pub_file(broker_public_key_path);
                if (pub_file.is_open()) {
                    pub_file >> broker_public_key;
                    zmq_curve_keypair(public_key, secret_key);

                    mSocket.set(zmq::sockopt::curve_serverkey, broker_public_key);
                    mSocket.set(zmq::sockopt::curve_publickey, public_key);
                    mSocket.set(zmq::sockopt::curve_secretkey, secret_key);
                } else {
                    std::cerr << "Error reading broker public key file.\n";
                }
            }


            mSocket.connect(mProxyAddress);
            mTopic = topic;
        }
        
        // Destructor
        ~Publisher() {
            mSocket.close();
            mContext.close();
        }

        void publish(const T& message) {
            // Serialize the message with msgpack
            msgpack::sbuffer buffer;
            msgpack::pack(buffer, message);

            // Create ZMQ message
            zmq::message_t topic_message(mTopic.data(), mTopic.size());
            zmq::message_t zmq_message(buffer.size());
            memcpy(zmq_message.data(), buffer.data(), buffer.size());

            // Publish the message
            mSocket.send(topic_message, zmq::send_flags::sndmore);
            mSocket.send(zmq_message, zmq::send_flags::none);
        }

};