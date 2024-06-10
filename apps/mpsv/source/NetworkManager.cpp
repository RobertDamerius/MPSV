#include <NetworkManager.hpp>
#include <MainApplication.hpp>


NetworkManager::NetworkManager(){
    started = false;
    terminate = false;
}

void NetworkManager::Start(const NetworkConfiguration networkConf){
    if(!started){
        started = true;
        destination.ip = networkConf.destinationAddress;
        destination.port = networkConf.destinationPort;
        networkReceiveThread = std::thread(&NetworkManager::NetworkReceiveThread, this, networkConf);
        struct sched_param param;
        param.sched_priority = networkConf.receiverThreadPriority;
        if(0 != pthread_setschedparam(networkReceiveThread.native_handle(), SCHED_FIFO, &param)){
            PrintW("Could not set thread priority %d for network manager!\n", networkConf.receiverThreadPriority);
        }
    }
}

void NetworkManager::Stop(void){
    if(started){
        started = false;
        terminate = true;
        udpSocket.Close();
        udpRetryTimer.NotifyOne(0);
        if(networkReceiveThread.joinable()){
            networkReceiveThread.join();
        }
        udpRetryTimer.Clear();
        terminate = false;
    }
}

void NetworkManager::Send(const mpsv::planner::AsyncOnlinePlannerOutput& plannerOutput){
    mpsv::planner::Serialize(&msgBuffer, plannerOutput);
    udpSocket.ResetLastError();
    (void) udpSocket.SendTo(destination, msgBuffer.bytes, sizeof(mpsv::planner::SerializationAsyncOnlinePlannerOutputUnion));
}

void NetworkManager::NetworkReceiveThread(const NetworkConfiguration networkConf){
    // local buffer where to store received messages
    constexpr size_t rxBufferSize = 65507;
    uint8_t* rxBuffer = new uint8_t[rxBufferSize];

    // main loop of the network manager
    while(!terminate){
        // (re)-initialize the socket operation
        if(!udpSocket.Open(networkConf)){
            udpRetryTimer.WaitFor(networkConf.socketErrorRetryTimeMs);
            continue;
        }
        PrintT("opened UDP socket (port=%u, interface=%u.%u.%u.%u, allowBroadcast=%u, deviceName=\"%s\")\n", networkConf.localPort, networkConf.interfaceAddress[0], networkConf.interfaceAddress[1], networkConf.interfaceAddress[2], networkConf.interfaceAddress[3], int(networkConf.allowBroadcast), networkConf.deviceName.c_str());

        // receive messages and process them
        IPAddress source;
        while(!terminate && udpSocket.IsOpen()){
            udpSocket.ResetLastError();
            int32_t rx = udpSocket.ReceiveFrom(source, &rxBuffer[0], rxBufferSize);
            auto [errorCode, errorString] = udpSocket.GetLastError();
            if(!udpSocket.IsOpen() || terminate){
                break;
            }
            if(rx < 0){
                #ifdef _WIN32
                if(WSAEMSGSIZE == errorCode){
                    continue;
                }
                #endif
                PrintE("Could not receive UDP message! %s\n", errorString.c_str());
                udpRetryTimer.WaitFor(networkConf.socketErrorRetryTimeMs);
                break;
            }
            ProcessReceivedMessage(&rxBuffer[0], rx);
            std::this_thread::yield();
        }

        // terminate the socket
        udpSocket.Close();
        PrintT("closed UDP socket (port=%u, interface=%u.%u.%u.%u, allowBroadcast=%u, deviceName=\"%s\")\n", networkConf.localPort, networkConf.interfaceAddress[0], networkConf.interfaceAddress[1], networkConf.interfaceAddress[2], networkConf.interfaceAddress[3], int(networkConf.allowBroadcast), networkConf.deviceName.c_str());
    }
    delete[] rxBuffer;
}

void NetworkManager::ProcessReceivedMessage(const uint8_t* bytes, int32_t length){
    if(static_cast<int32_t>(sizeof(mpsv::planner::SerializationAsyncOnlinePlannerInputUnion)) == length){
        const mpsv::planner::SerializationAsyncOnlinePlannerInputUnion* messageData = reinterpret_cast<const mpsv::planner::SerializationAsyncOnlinePlannerInputUnion*>(bytes);
        mpsv::planner::AsyncOnlinePlannerInput plannerInput;
        mpsv::planner::Deserialize(plannerInput, messageData);
        mainApplication.planner.SetInput(plannerInput);
    }
    else if(static_cast<int32_t>(sizeof(mpsv::planner::SerializationAsyncOnlinePlannerParameterUnion)) == length){
        const mpsv::planner::SerializationAsyncOnlinePlannerParameterUnion* messageData = reinterpret_cast<const mpsv::planner::SerializationAsyncOnlinePlannerParameterUnion*>(bytes);
        mpsv::planner::AsyncOnlinePlannerParameterSet plannerParameter;
        mpsv::planner::Deserialize(plannerParameter, messageData);
        mainApplication.planner.SetParameter(plannerParameter);
    }
}

