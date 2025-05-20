#include <string>
#include <zmq.hpp>
#include <msgpack.hpp>

template<typename T>
class Publisher {
    std::string mName; // Name of the publisher
    std::string mProxyAddress; // Proxy address written in the format of "tcp://hostname:port"
    std::string mTopic; // Topic to publish to

    zmq::context_t mContext;
    zmq::socket_t mSocket;

    public:
        // Constructor
        Publisher(std::string name, std::string proxy_address, std::string topic) {
            mName = name;
            mProxyAddress = proxy_address;
            mContext = zmq::context_t(1);
            mSocket = zmq::socket_t(mContext, ZMQ_PUB);
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