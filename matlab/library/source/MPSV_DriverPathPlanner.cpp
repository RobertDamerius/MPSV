#include <MPSV_DriverPathPlanner.hpp>
#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_PathPlannerInitialize(void** workVector, int16_t maxNumNodes, uint32_t maxNumSamples){
    // Create the wrapper class and initialize it
    MPSV_WrapperPathPlanner* wrapper = new MPSV_WrapperPathPlanner();
    wrapper->Initialize(maxNumNodes, maxNumSamples);
    *workVector = reinterpret_cast<void*>(wrapper);
}

void MPSV_PathPlannerTerminate(void* workVector){
    // Terminate the wrapper class and free memory
    MPSV_WrapperPathPlanner* wrapper = reinterpret_cast<MPSV_WrapperPathPlanner*>(workVector);
    wrapper->Terminate();
    delete wrapper;
    workVector = nullptr;
}

void MPSV_PathPlannerStep(void* workVector, uint8_t* serializedDataOut, uint8_t* serializedDataIn){
    // Reinterpret wrapper, input and output data
    MPSV_WrapperPathPlanner* wrapper = reinterpret_cast<MPSV_WrapperPathPlanner*>(workVector);
    SerializationPathPlannerInputUnion* in = reinterpret_cast<SerializationPathPlannerInputUnion*>(serializedDataIn);
    SerializationPathPlannerOutputUnion* out = reinterpret_cast<SerializationPathPlannerOutputUnion*>(serializedDataOut);

    // Call the step function of the wrapper class
    wrapper->Step(out, in);
}

