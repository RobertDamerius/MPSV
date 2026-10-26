#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>


MPSV_WrapperAsynchronousOnlinePlanner::MPSV_WrapperAsynchronousOnlinePlanner(){}

MPSV_WrapperAsynchronousOnlinePlanner::~MPSV_WrapperAsynchronousOnlinePlanner(){
    Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs){
    (void) m_planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
}

void MPSV_WrapperAsynchronousOnlinePlanner::Terminate(void){
    m_planner.Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Step(mpsv::planner::serialization_output* output, mpsv::planner::serialization_input* input, mpsv::planner::serialization_parameter* parameter){
    // deserialize and set parameter
    mpsv::planner::Deserialize(m_plannerParameter, parameter);
    m_planner.SetParameter(m_plannerParameter);

    // deserialize and set input
    mpsv::planner::Deserialize(m_plannerInput, input);
    m_planner.SetInput(m_plannerInput);

    // get and serialize current output
    m_plannerOutput = m_planner.GetOutput();
    mpsv::planner::Serialize(output, m_plannerOutput);
}
