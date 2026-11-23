#include <MPSV_DriverAsynchronousOnlinePlanner.hpp>
#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>
#include <vector>


void MPSV_AsynchronousOnlinePlannerInitialize(void** workVector, std::int16_t pathMaxNumNodes, std::uint32_t pathMaxNumSamples, std::int16_t motionMaxNumNodes, std::uint32_t motionMaxNumSamples, std::uint32_t maxNumCells, std::int32_t threadPriority, std::int32_t ompNumThreads, std::int32_t ompDynamic, std::int32_t* cpuCoreIDs, std::uint32_t numCPUCoreIDs) {
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = new MPSV_WrapperAsynchronousOnlinePlanner();
    std::vector<std::int32_t> cpuCoreIDsVec;
    if (cpuCoreIDs && (numCPUCoreIDs > 0)) {
        cpuCoreIDsVec.assign(cpuCoreIDs, cpuCoreIDs + numCPUCoreIDs);
    }
    wrapper->Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDsVec);
    *workVector = reinterpret_cast<void*>(wrapper);
}

void MPSV_AsynchronousOnlinePlannerTerminate(void* workVector) {
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    wrapper->Stop();
    delete wrapper;
}

void MPSV_AsynchronousOnlinePlannerStep(void* workVector, std::uint8_t* serializedOutput, std::uint8_t* serializedInput, std::uint8_t* serializedParameter) {
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    serialization_asynchronous_online_planner_output* out = reinterpret_cast<serialization_asynchronous_online_planner_output*>(serializedOutput);
    serialization_asynchronous_online_planner_input* in = reinterpret_cast<serialization_asynchronous_online_planner_input*>(serializedInput);
    serialization_asynchronous_online_planner_parameter* param = reinterpret_cast<serialization_asynchronous_online_planner_parameter*>(serializedParameter);
    wrapper->Step(out, in, param);
}
