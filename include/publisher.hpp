#include <string>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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

        void publish(const json& message) {
            // Serialize the json to bson
            // std::vector<uint8_t> message_bson = json::to_bson(message);

            // Serialize the json to string
            std::string message_string = message.dump();

            // Create a multipart message
            zmq::message_t topic_message(mTopic.data(), mTopic.size());
            zmq::message_t zmq_message(message_string.data(), message_string.size());

            // Send the multipart message
            mSocket.send(topic_message, zmq::send_flags::sndmore);
            mSocket.send(zmq_message, zmq::send_flags::none);
        }

};