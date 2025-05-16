#include <string>
#include <zmq.hpp>

class Publisher {
    std::string mName; // Name of the publisher
    std::string mProxyAddress; // Proxy address written in the format of "tcp://hostname:port"

    zmq::context_t mContext;
    zmq::socket_t mSocket;

    public:
        // Constructor
        Publisher(std::string name, std::string proxy_address) 
            : mName(name), mProxyAddress(proxy_address), mContext(1), mSocket(mContext, ZMQ_PUB) {
            mSocket.connect(mProxyAddress);
        }

        // Destructor
        ~Publisher() {
            mSocket.close();
        }

        // Function to publish a message
        void publish(const std::string& topic, const std::string& message) {
            zmq::message_t topic_msg(topic.data(), topic.size());
            zmq::message_t message_msg(message.data(), message.size());
            mSocket.send(topic_msg, zmq::send_flags::sndmore);
            mSocket.send(message_msg, zmq::send_flags::none);
        }

};