#include <MPSV_DriverPathPlanner.hpp>
#include <MPSV_WrapperPathPlanner.hpp>
#include <memory>


void MPSV_PathPlannerInitialize(void** const workVector, const std::int16_t maxNumNodes, const std::uint32_t maxNumSamples, const std::uint32_t maxNumCells) {
    if (workVector == nullptr) {
        return;
    }
    MPSV_WrapperPathPlanner* const wrapper{new MPSV_WrapperPathPlanner()};
    wrapper->Initialize(maxNumNodes, maxNumSamples, maxNumCells);
    *workVector = static_cast<void*>(wrapper);
}

void MPSV_PathPlannerTerminate(void* const workVector) {
    if (workVector == nullptr) {
        return;
    }
    MPSV_WrapperPathPlanner* const wrapper{static_cast<MPSV_WrapperPathPlanner*>(workVector)};
    wrapper->Terminate();
    delete wrapper;
}

void MPSV_PathPlannerStep(void* const workVector, std::uint8_t* const serializedDataOut, const std::uint8_t* const serializedDataIn) noexcept {
    if ((workVector == nullptr) || (serializedDataOut == nullptr) || (serializedDataIn == nullptr)) {
        return;
    }
    MPSV_WrapperPathPlanner* const wrapper{static_cast<MPSV_WrapperPathPlanner*>(workVector)};
    const serialization_path_planner_input* const in = std::start_lifetime_as<serialization_path_planner_input>(serializedDataIn);
    serialization_path_planner_output* const out = std::start_lifetime_as<serialization_path_planner_output>(serializedDataOut);
    wrapper->Step(out, in);
}
