#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>


MPSV_WrapperAsynchronousOnlinePlanner::MPSV_WrapperAsynchronousOnlinePlanner(){}

MPSV_WrapperAsynchronousOnlinePlanner::~MPSV_WrapperAsynchronousOnlinePlanner(){
    Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs){
    (void) planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
}

void MPSV_WrapperAsynchronousOnlinePlanner::Terminate(void){
    planner.Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Step(mpsv::planner::serialization_output* output, mpsv::planner::serialization_input* input, mpsv::planner::serialization_parameter* parameter){
    // deserialize and set parameter
    mpsv::planner::Deserialize(plannerParameter, parameter);
    planner.SetParameter(plannerParameter);

    // deserialize and set input
    mpsv::planner::Deserialize(plannerInput, input);
    planner.SetInput(plannerInput);

    // get and serialize current output
    plannerOutput = planner.GetOutput();
    mpsv::planner::Serialize(output, plannerOutput);
}

