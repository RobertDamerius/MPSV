#pragma once


#include <Common.hpp>
#include <UnicastUDPSocket.hpp>
#include <mpsv/mpsv.hpp>


/**
 * @brief The network manager handles the UDP unicast socket and receives and processes messages.
 * Successfully received messages are forwarded to the planner.
 */
class NetworkManager {
    public:
        /**
         * @brief Construct a new network manager object.
         */
        NetworkManager();

        /**
         * @brief Start the network manager by launching a separate thread.
         * @param[in] networkConf The network configuration to be used.
         */
        void Start(const NetworkConfiguration networkConf);

        /**
         * @brief Stop the network manager.
         */
        void Stop(void);

        /**
         * @brief Send planner output.
         * @param[in] plannerOutput Planner output to be sent.
         */
        void Send(const mpsv::planner::AsyncOnlinePlannerOutput& plannerOutput);

    private:
        std::atomic<bool> started;                       // True if the network manager has already been started, false otherwise.
        std::thread networkReceiveThread;                // Thread object for the internal network manager receive thread.
        std::atomic<bool> terminate;                     // True if the thread should be terminated, false otherwise.
        UnicastUDPSocket udpSocket;                      // The unicast UDP socket.
        mpsv::core::Event udpRetryTimer;                 // A timer to wait before retrying to initialize a UDP socket in case of errors.
        mpsv::planner::serialization_output msgBuffer;   // Message buffer for the output message.
        IPAddress destination;                           // Destination to which to send output messages.

        /**
         * @brief The receive thread function of the network manager.
         * @param[in] networkConf The network configuration to be used.
         */
        void NetworkReceiveThread(const NetworkConfiguration networkConf);

        /**
         * @brief Process a received UDP message.
         * @param[in] bytes The bytes containing the message.
         * @param[in] length The length of the received UDP message.
         */
        void ProcessReceivedMessage(const uint8_t* bytes, int32_t length);
};

