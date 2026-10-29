#include <MPSV_DriverAsynchronousOnlinePlanner.hpp>
#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>
#include <vector>


void MPSV_AsynchronousOnlinePlannerInitialize(void** workVector, int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, int32_t* cpuCoreIDs, uint32_t numCPUCoreIDs){
    // create the wrapper class and start it
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = new MPSV_WrapperAsynchronousOnlinePlanner();
    std::vector<int32_t> cpuCoreIDsVec(cpuCoreIDs, cpuCoreIDs + numCPUCoreIDs);
    wrapper->Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDsVec);
    *workVector = reinterpret_cast<void*>(wrapper);
}

void MPSV_AsynchronousOnlinePlannerTerminate(void* workVector){
    // stop the wrapper class and free memory
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    wrapper->Stop();
    delete wrapper;
    workVector = nullptr;
}

void MPSV_AsynchronousOnlinePlannerStep(void* workVector, uint8_t* serializedOutput, uint8_t* serializedInput, uint8_t* serializedParameter){
    // reinterpret wrapper, input and output data
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    serialization_asynchronous_online_planner_output* out = reinterpret_cast<serialization_asynchronous_online_planner_output*>(serializedOutput);
    serialization_asynchronous_online_planner_input* in = reinterpret_cast<serialization_asynchronous_online_planner_input*>(serializedInput);
    serialization_asynchronous_online_planner_parameter* param = reinterpret_cast<serialization_asynchronous_online_planner_parameter*>(serializedParameter);

    // call the step function of the wrapper class
    wrapper->Step(out, in, param);
}
