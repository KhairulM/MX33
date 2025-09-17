#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <string>

std::string get_local_ip() {
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        return "";
    }

    std::string ip;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        // Only IPv4
        if (ifa->ifa_addr->sa_family == AF_INET) {
            // Skip loopback
            if (!(ifa->ifa_flags & IFF_LOOPBACK) && (ifa->ifa_flags & IFF_UP)) {
                char buf[INET_ADDRSTRLEN];
                void* addr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
                if (inet_ntop(AF_INET, addr, buf, sizeof(buf))) {
                    ip = buf;
                    break;  // take the first one we find
                }
            }
        }
    }

    freeifaddrs(ifaddr);
    return ip;
}