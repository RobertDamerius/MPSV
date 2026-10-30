#include <mpsv/mpsv.hpp>
#include <ExampleAsyncOnlinePlanning.hpp>


int main(int, char**){
    // construct and start the online planner
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    constexpr uint32_t maxNumCells = 4096 * 4096;
    constexpr int32_t threadPriority = 20;
    constexpr int32_t ompNumThreads = -1; // ignore
    constexpr int32_t ompDynamic = -1; // ignore
    std::vector<int32_t> cpuCoreIDs; // ignore
    mpsv::planner::AsyncOnlinePlanner planner;
    planner.Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);

    // set input data
    mpsv::runtime::PerformanceCounter timer;
    mpsv::planner::AsyncOnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
    mpsv::planner::AsyncOnlinePlannerParameterSet parameter = example::GetParameterSet();
    planner.SetInput(dataIn, parameter);

    // poll the output
    for(int i = 0; i != 10; ++i){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        mpsv::planner::AsyncOnlinePlannerOutput dataOut = planner.GetOutput();

        // print result (at most 10 trajectory points)
        std::cout << "threadState=" << mpsv::to_string(dataOut.threadState) <<
                     ", errorCode=" << static_cast<int32_t>(dataOut.errorCode) <<
                     ", performedReset=" << dataOut.performedReset <<
                     ", timestamp=" << dataOut.timestamp <<
                     ", sampletime=" << dataOut.sampletime <<
                     ", trajectory=" << dataOut.trajectory.size() << " points\n";
    }

    // stop the planner
    planner.Stop();
    return 0;
}
