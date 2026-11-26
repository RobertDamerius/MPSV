#include <MPSV_DriverAsynchronousOnlinePlanner.hpp>
#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>
#include <vector>
#include <memory>


void MPSV_AsynchronousOnlinePlannerInitialize(void** const workVector, const std::int16_t pathMaxNumNodes, const std::uint32_t pathMaxNumSamples, const std::int16_t motionMaxNumNodes, const std::uint32_t motionMaxNumSamples, const std::uint32_t maxNumCells, const std::int32_t threadPriority, const std::int32_t ompNumThreads, const std::int32_t ompDynamic, const std::int32_t* const cpuCoreIDs, const std::uint32_t numCPUCoreIDs) {
    if (workVector == nullptr) {
        return;
    }
    MPSV_WrapperAsynchronousOnlinePlanner* const wrapper{new MPSV_WrapperAsynchronousOnlinePlanner()};
    std::vector<std::int32_t> cpuCoreIDsVec;
    if ((cpuCoreIDs != nullptr) && (numCPUCoreIDs > 0)) {
        cpuCoreIDsVec.assign(cpuCoreIDs, cpuCoreIDs + numCPUCoreIDs);
    }
    wrapper->Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDsVec);
    *workVector = static_cast<void*>(wrapper);
}

void MPSV_AsynchronousOnlinePlannerTerminate(void* const workVector) {
    if (workVector == nullptr) {
        return;
    }
    MPSV_WrapperAsynchronousOnlinePlanner* const wrapper{static_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector)};
    wrapper->Stop();
    delete wrapper;
}

void MPSV_AsynchronousOnlinePlannerStep(void* const workVector, std::uint8_t* const serializedOutput, const std::uint8_t* const serializedInput, const std::uint8_t* const serializedParameter) noexcept {
    if ((workVector == nullptr) || (serializedOutput == nullptr) || (serializedInput == nullptr) || (serializedParameter == nullptr)) {
        return;
    }
    MPSV_WrapperAsynchronousOnlinePlanner* const wrapper{static_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector)};
    const serialization_asynchronous_online_planner_input* const in = std::start_lifetime_as<serialization_asynchronous_online_planner_input>(serializedInput);
    const serialization_asynchronous_online_planner_parameter* const param = std::start_lifetime_as<serialization_asynchronous_online_planner_parameter>(serializedParameter);
    serialization_asynchronous_online_planner_output* const out = std::start_lifetime_as<serialization_asynchronous_online_planner_output>(serializedOutput);
    wrapper->Step(out, in, param);
}
