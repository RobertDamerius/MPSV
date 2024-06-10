#pragma once


#include <Common.hpp>


/**
 * @brief Represents the network configuration.
 */
class NetworkConfiguration {
    public:
        uint16_t localPort;                         // The local port to be bound.
        std::array<uint8_t,4> destinationAddress;   // The destination address to which to send results.
        uint16_t destinationPort;                   // The destination port to which to send results.
        std::array<uint8_t,4> interfaceAddress;     // The IPv4 address of the network interface to be used.
        std::string deviceName;                     // The device name to be set for this socket.
        int32_t socketPriority;                     // The socket priority (linux only, in range [0 (lowest), 6 (greatest)]).
        bool allowBroadcast;                        // True if broadcast is to be allowed, false otherwise.
        uint32_t socketErrorRetryTimeMs;            // Time in milliseconds to wait in case of an socket error before trying again.
        int32_t receiverThreadPriority;             // The priority of the UDP receive thread that receives and processes incomming messages.

        /**
         * @brief Construct a new network configuration object and set default values.
         */
        NetworkConfiguration(){ Clear(); }

        /**
         * @brief Clear configuration and set default values.
         */
        void Clear(void){
            localPort = 0;
            destinationAddress = {0,0,0,0};
            destinationPort = 0;
            interfaceAddress = {0,0,0,0};
            deviceName = "";
            socketPriority = 0;
            allowBroadcast = false;
            socketErrorRetryTimeMs = 0;
            receiverThreadPriority = 0;
        }
};

