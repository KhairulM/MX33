#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>

#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <msgpack.hpp>

using json = nlohmann::json;

template<typename T>
class Subscriber {
    std::string mName; // Name of the subscriber
    std::string mProxyAddress; // Proxy address written in the format of "tcp://hostname:port"
    std::string mTopic; // Topic to subscribe to
    zmq::context_t mContext;
    zmq::socket_t mSocket;
    
    std::queue<zmq::message_t> mMessages;
    int mMaxQueueSize = 45;

    std::thread mThread;    
    std::mutex mMutex;

    void processZMQMessage() {
        while (true) {
            zmq::message_t topic, message;

            try {
                mSocket.recv(&topic);
                mSocket.recv(&message);
            } catch (const zmq::error_t& e) {
                break;
            }

            std::lock_guard<std::mutex> lock(mMutex);
            if (mMessages.size() >= mMaxQueueSize) {
                mMessages.pop(); // Remove the oldest message
            }
            mMessages.push(std::move(message));
        }
    }

    public:
        // Constructor
        Subscriber(std::string name, std::string proxy_address, std::string topic, int max_queue_size = 45) {
            mName = name;
            mProxyAddress = proxy_address;
            mTopic = topic;
            mMaxQueueSize = max_queue_size;
            
            mContext = zmq::context_t(1);
            mSocket = zmq::socket_t(mContext, ZMQ_SUB);
            mSocket.connect(mProxyAddress);
            mSocket.set(zmq::sockopt::subscribe, mTopic);

            mThread = std::thread(&Subscriber::processZMQMessage, this);
        }

        // Destructor
        ~Subscriber() {
            mSocket.close();
            mContext.close();
            if (mThread.joinable()) {
                mThread.join();
            }
        }

        json getMessage() {
            std::lock_guard<std::mutex> lock(mMutex);
            if (mMessages.empty()) {
                return json::object();
            }

            zmq::message_t message = std::move(mMessages.front());
            mMessages.pop();

            // Convert ZMQ message to bson std::vector<uint8_t>
            // std::vector<uint8_t> message_bson(message.size());
            // memcpy(message_bson.data(), message.data(), message.size());

            // Deserialize the message to JSON
            // json message_json = json::from_bson(message_bson);

            // Deserialize the message from string
            json message_json = json::parse(message.to_string());
            return message_json;
        }

        std::unique_ptr<T> getMessageObject() {
            std::lock_guard<std::mutex> lock(mMutex);
            if (mMessages.empty()) {
                return nullptr;
            }

            zmq::message_t message = std::move(mMessages.front());
            mMessages.pop();

            // Deserialize the message with msgpack
            msgpack::object_handle oh = msgpack::unpack(static_cast<const char*>(message.data()), message.size());
            T message_data;
            oh.get().convert(message_data);

            return std::make_unique<T>(message_data);
        }
};