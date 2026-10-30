#include <mpsv/mpsv.hpp>
#include <ExampleOnlinePlanning.hpp>


int main(int, char**){
    // construct online planner
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    constexpr uint32_t maxNumCells = 4096 * 4096;
    mpsv::planner::OnlinePlanner planner;
    planner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);

    // solve the online planning problem
    std::cout << "solve online planning problem (5 steps)\n\n";
    mpsv::runtime::PerformanceCounter timer;
    for(int i = 0; i != 5; ++i){
        // solve the problem
        mpsv::planner::OnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
        mpsv::planner::OnlinePlannerParameterSet parameter = example::GetParameterSet();
        mpsv::planner::OnlinePlannerOutput dataOut;
        planner.Solve(dataOut, dataIn, parameter);

        // print result (at most 10 trajectory points)
        std::cout << "timestamp:        " << dataOut.timestamp << "\n";
        std::cout << "sampletime:       " << dataOut.sampletime << "\n";
        std::cout << "errorCode:        " << static_cast<int32_t>(dataOut.errorCode) << "\n";
        std::cout << "performedReset:   " << dataOut.performedReset << "\n";
        std::cout << "originLLA:        " << "{" << dataOut.originLLA[0] << ", " << dataOut.originLLA[1] << ", " << dataOut.originLLA[2] << "}\n";
        std::cout << "trajectory:       " << dataOut.trajectory.size() << " points\n";
        std::cout << "trajectory:       " << dataOut.trajectory.size() << " points\n";
        for(size_t i = 0; (i < dataOut.trajectory.size()) && (i < 10); ++i){
            std::cout << "    x,y,psi={" << dataOut.trajectory[i][0] << ", " << dataOut.trajectory[i][1] << ", " << dataOut.trajectory[i][2] << "} ";
            std::cout << "u,v,r={" << dataOut.trajectory[i][3] << ", " << dataOut.trajectory[i][4] << ", " << dataOut.trajectory[i][5] << "} ";
            std::cout << "X,Y,N={" << dataOut.trajectory[i][6] << ", " << dataOut.trajectory[i][7] << ", " << dataOut.trajectory[i][8] << "} ";
            std::cout << "Xc,Yc,Nc={" << dataOut.trajectory[i][9] << ", " << dataOut.trajectory[i][10] << ", " << dataOut.trajectory[i][11] << "}\n";
        }
        std::cout << std::endl;
    }
    return 0;
}
