#pragma once


#include <Common.hpp>
#include <UDPSocket.hpp>
#include <NetworkConfiguration.hpp>


/**
 * @brief The socket for UDP unicast operation.
 */
class UnicastUDPSocket: protected UDPSocket {
    public:
        using UDPSocket::IsOpen;
        using UDPSocket::ReceiveFrom;
        using UDPSocket::SendTo;
        using UDPSocket::GetLastError;
        using UDPSocket::ResetLastError;

        /**
         * @brief Open the unicast UDP socket.
         * @param[in] configuration The network configuration to be used to open the unicast UDP socket.
         * @return True if success, false otherwise.
         */
        bool Open(const NetworkConfiguration configuration){
            conf = configuration;

            // open the UDP socket
            ResetLastError();
            if(!UDPSocket::Open()){
                auto [errorCode, errorString] = GetLastError();
                PrintE("Could not open UDP socket! %s\n", errorString.c_str());
                return false;
            }

            // set broadcast option and ignore errors
            ResetLastError();
            if(AllowBroadcast(conf.allowBroadcast) < 0){
                auto [errorCode, errorString] = GetLastError();
                PrintW("Could not set allow broadcast option for unicast UDP socket! %s\n", errorString.c_str());
            }

            // bind socket to a network interface device if required
            if(conf.deviceName.size()){
                ResetLastError();
                if(BindToDevice(conf.deviceName) < 0){
                    auto [errorCode, errorString] = GetLastError();
                    PrintE("Could not bind unicast UDP socket to the device \"%s\"! %s\n", conf.deviceName.c_str(), errorString.c_str());
                    UDPSocket::Close();
                    return false;
                }
            }

            // set priority and ignore errors
            #ifndef _WIN32
            int priority = static_cast<int>(conf.socketPriority);
            ResetLastError();
            if(SetOption(SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0){
                auto [errorCode, errorString] = GetLastError();
                PrintW("Could not set socket priority %d for UDP socket! %s\n", conf.socketPriority, errorString.c_str());
            }
            #endif

            // reuse port and ignore errors
            ResetLastError();
            if(ReusePort(true) < 0){
                auto [errorCode, errorString] = GetLastError();
                PrintW("Could not set reuse port option for UDP socket! %s\n", errorString.c_str());
            }

            // bind the port
            ResetLastError();
            if(Bind(conf.localPort, conf.interfaceAddress) < 0){
                auto [errorCode, errorString] = GetLastError();
                PrintE("Could not bind port for unicast UDP socket (port=%u, interface=%u.%u.%u.%u)! %s\n", conf.localPort, conf.interfaceAddress[0], conf.interfaceAddress[1], conf.interfaceAddress[2], conf.interfaceAddress[3], errorString.c_str());
                UDPSocket::Close();
                return false;
            }

            // success
            return true;
        }

        /**
         * @brief Close the unicast UDP socket.
         */
        void Close(void){
            UDPSocket::Close();
        }

    private:
        NetworkConfiguration conf;   // The socket configuration that has been used to open the socket.
};

