#include <MPSV_DriverPathPlanner.hpp>
#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_PathPlannerInitialize(void** workVector, int16_t maxNumNodes, uint32_t maxNumSamples, uint32_t maxNumCells){
    MPSV_WrapperPathPlanner* wrapper = new MPSV_WrapperPathPlanner();
    wrapper->Initialize(maxNumNodes, maxNumSamples, maxNumCells);
    *workVector = reinterpret_cast<void*>(wrapper);
}

void MPSV_PathPlannerTerminate(void* workVector){
    MPSV_WrapperPathPlanner* wrapper = reinterpret_cast<MPSV_WrapperPathPlanner*>(workVector);
    wrapper->Terminate();
    delete wrapper;
    workVector = nullptr;
}

void MPSV_PathPlannerStep(void* workVector, uint8_t* serializedDataOut, uint8_t* serializedDataIn){
    MPSV_WrapperPathPlanner* wrapper = reinterpret_cast<MPSV_WrapperPathPlanner*>(workVector);
    serialization_path_planner_input* in = reinterpret_cast<serialization_path_planner_input*>(serializedDataIn);
    serialization_path_planner_output* out = reinterpret_cast<serialization_path_planner_output*>(serializedDataOut);
    wrapper->Step(out, in);
}
