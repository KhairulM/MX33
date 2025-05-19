#include <iostream>
#include <string>
#include <zmq.hpp>

class Broker {
    std::string mFrontendAddress, mBackendAddress;

    public:
        Broker(std::string frontend_address, std::string backend_address): mFrontendAddress(frontend_address), mBackendAddress(backend_address) {}

        void run() {
            // Create a new ZMQ context
            zmq::context_t context(1);

            // Create a new ZMQ socket for the frontend
            zmq::socket_t frontend(context, ZMQ_XSUB);
            frontend.bind(mFrontendAddress);

            // Create a new ZMQ socket for the backend
            zmq::socket_t backend(context, ZMQ_XPUB);
            backend.bind(mBackendAddress);

            std::cout << "Broker started, XSUB: " << mFrontendAddress << ", XPUB:  " << mBackendAddress << std::endl;

            // Start the proxy
            zmq_proxy(frontend, backend, NULL);

            // Close the sockets
            frontend.close();
            backend.close();
            context.close();
        }
};