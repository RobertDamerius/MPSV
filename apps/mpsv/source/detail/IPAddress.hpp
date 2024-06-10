#pragma once


#include <Common.hpp>


/**
 * @brief This class represents a network address including an IPv4 address and a port.
 */
class IPAddress {
    public:
        std::array<uint8_t,4> ip;   // IPv4 address.
        uint16_t port;              // Port value.

        /**
         * @brief Create an address object (IP version 4).
         * @details IPv4 address and port are set to zero.
         */
        IPAddress():IPAddress(0,0,0,0,0){};

        /**
         * @brief Create an address object (IP version 4).
         * @param[in] ip0 Byte 0 of IPv4 address.
         * @param[in] ip1 Byte 1 of IPv4 address.
         * @param[in] ip2 Byte 2 of IPv4 address.
         * @param[in] ip3 Byte 3 of IPv4 address.
         * @param[in] port Port value.
         */
        IPAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3, uint16_t port){
            ip[0] = ip0;
            ip[1] = ip1;
            ip[2] = ip2;
            ip[3] = ip3;
            this->port = port;
        }

        /**
         * @brief Create an address object (IP version 4).
         * @param[in] ip IPv4 address.
         * @param[in] port Port value.
         */
        IPAddress(std::array<uint8_t,4> ip, uint16_t port){
            this->ip[0] = ip[0];
            this->ip[1] = ip[1];
            this->ip[2] = ip[2];
            this->ip[3] = ip[3];
            this->port = port;
        }
};

