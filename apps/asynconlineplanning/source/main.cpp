#include <mpsv/mpsv.hpp>
#include <ExampleAsyncOnlinePlanning.hpp>


int main(int, char**){
    // construct and initialize the online planner
    mpsv::planner::AsyncOnlinePlanner planner;
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    constexpr int32_t threadPriority = 20;
    int32_t ompNumThreads = -1; // ignore
    int32_t ompDynamic = -1; // ignore
    if(!planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, threadPriority, ompNumThreads, ompDynamic)){
        std::cerr << "ERROR: Could not initialize planner!\n";
        return 0;
    }

    // construct a timer for timestamp generation
    mpsv::core::PerformanceCounter timer;
    timer.Start();

    // set parameter
    mpsv::planner::AsyncOnlinePlannerParameterSet parameter = example::GetParameterSet(timer.TimeToStart());
    if(!parameter.IsValid()){
        std::cerr << "WARNING: parameters are invalid!\n";
    }
    planner.SetParameter(parameter);

    // solve the online planning problem
    std::cout << "solve async online planning problem\n\n";
    mpsv::planner::AsyncOnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
    if(!dataIn.IsValid()){
        std::cerr << "WARNING: input data is invalid!\n";
    }
    planner.SetInput(dataIn);

    // poll the output
    for(int i = 0; i != 10; ++i){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        mpsv::planner::AsyncOnlinePlannerOutput dataOut = planner.GetOutput();

        // print result (at most 10 trajectory points)
        std::cout << "threadState=" << mpsv::to_string(dataOut.threadState) <<
                     ", timestampInput=" << std::to_string(dataOut.timestampInput) <<
                     ", timestampParameter=" << std::to_string(dataOut.timestampParameter) <<
                     ", timeoutInput=" << dataOut.timeoutInput <<
                     ", validInput=" << dataOut.validInput <<
                     ", validParameter=" << dataOut.validParameter <<
                     ", error=" << dataOut.error <<
                     ", performedReset=" << dataOut.performedReset <<
                     ", timestamp=" << dataOut.timestamp <<
                     ", sampletime=" << dataOut.sampletime <<
                     ", trajectory=" << dataOut.trajectory.size() << " points\n";
    }

    // terminate the planner
    planner.Terminate();
    return 0;
}

