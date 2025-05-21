#include <iostream>
#include <string>
#include <fstream>
#include <zmq.hpp>

class Broker {
    std::string mFrontendAddress, mBackendAddress;
    char private_key[41];

    public:
        Broker(
            std::string frontend_address, 
            std::string backend_address, 
            std::string secret_key_path = ""
        ): mFrontendAddress(frontend_address), mBackendAddress(backend_address) {
            if (secret_key_path.empty()) {
                return;
            }

            std::ifstream secret_file(secret_key_path);

            if (secret_file.is_open()) {
                secret_file >> private_key;
            } else {
                std::cerr << "Error reading key file.\n";
            }
        }

        void run() {
            // Create a ZMQ context
            zmq::context_t context(1);

            // Create a ZMQ socket for the frontend (The one that will receive messages from subscribers)
            zmq::socket_t frontend(context, ZMQ_XSUB);
            if (private_key[0] != '\0') {
                frontend.set(zmq::sockopt::curve_server, 1);
                frontend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            frontend.bind(mFrontendAddress);

            // Create a ZMQ socket for the backend (The one that will send messages to publishers)
            zmq::socket_t backend(context, ZMQ_XPUB);
            if (private_key[0] != '\0') {
                backend.set(zmq::sockopt::curve_server, 1);
                backend.set(zmq::sockopt::curve_secretkey, private_key);
            }
            backend.bind(mBackendAddress);
            
            std::cout << "Broker started, XSUB: " << mFrontendAddress << ", XPUB:  " << mBackendAddress << std::endl;

            // Start the proxy
            zmq::proxy(frontend, backend, NULL);

            // Close the sockets
            frontend.close();
            backend.close();
            context.close();
        }
};