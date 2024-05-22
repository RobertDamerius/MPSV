#include <MPSV_DriverAsynchronousOnlinePlanner.hpp>
#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>


void MPSV_AsynchronousOnlinePlannerInitialize(void** workVector, int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic){
    // Create the wrapper class and initialize it
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = new MPSV_WrapperAsynchronousOnlinePlanner();
    wrapper->Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, threadPriority, ompNumThreads, ompDynamic);
    *workVector = reinterpret_cast<void*>(wrapper);
}

void MPSV_AsynchronousOnlinePlannerTerminate(void* workVector){
    // Terminate the wrapper class and free memory
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    wrapper->Terminate();
    delete wrapper;
    workVector = nullptr;
}

void MPSV_AsynchronousOnlinePlannerStep(void* workVector, uint8_t* serializedOutput, uint8_t* serializedInput, uint8_t* serializedParameter){
    // Reinterpret wrapper, input and output data
    MPSV_WrapperAsynchronousOnlinePlanner* wrapper = reinterpret_cast<MPSV_WrapperAsynchronousOnlinePlanner*>(workVector);
    SerializationAsynchronousOnlinePlannerOutputUnion* out = reinterpret_cast<SerializationAsynchronousOnlinePlannerOutputUnion*>(serializedOutput);
    SerializationAsynchronousOnlinePlannerInputUnion* in = reinterpret_cast<SerializationAsynchronousOnlinePlannerInputUnion*>(serializedInput);
    SerializationAsynchronousOnlinePlannerParameterUnion* param = reinterpret_cast<SerializationAsynchronousOnlinePlannerParameterUnion*>(serializedParameter);

    // Call the step function of the wrapper class
    wrapper->Step(out, in, param);
}

