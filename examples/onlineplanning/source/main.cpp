#include <mpsv/mpsv.hpp>
#include <ExampleOnlinePlanning.hpp>


int main(int, char**) {
    // construct online planner
    constexpr std::int16_t pathMaxNumNodes{200};
    constexpr std::uint32_t pathMaxNumSamples{1000000U};
    constexpr std::int16_t motionMaxNumNodes{200};
    constexpr std::uint32_t motionMaxNumSamples{1000000U};
    constexpr std::uint32_t maxNumCells{4096U * 4096U};
    mpsv::planner::OnlinePlanner planner;
    planner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);

    // solve the online planning problem
    std::cout << "solve online planning problem (5 steps)\n\n";
    mpsv::runtime::PerformanceCounter timer;
    for (int i{0}; i != 5; ++i) {
        // solve the problem
        mpsv::planner::OnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
        mpsv::planner::OnlinePlannerParameterSet parameter = example::GetParameterSet();
        mpsv::planner::OnlinePlannerOutput dataOut;
        planner.Solve(dataOut, dataIn, parameter);

        // print result (at most 10 trajectory points)
        std::cout << "timestamp:        " << dataOut.timestamp << "\n";
        std::cout << "sampletime:       " << dataOut.sampletime << "\n";
        std::cout << "errorCode:        " << static_cast<std::int32_t>(dataOut.errorCode) << "\n";
        std::cout << "performedReset:   " << dataOut.performedReset << "\n";
        std::cout << "originLLA:        " << "{" << dataOut.originLLA[0U] << ", " << dataOut.originLLA[1U] << ", " << dataOut.originLLA[2U] << "}\n";
        std::cout << "trajectory:       " << dataOut.trajectory.size() << " points\n";
        for (std::size_t i{0U}; (i < dataOut.trajectory.size()) && (i < 10U); ++i) {
            std::cout << "    x,y,psi={" << dataOut.trajectory[i][0U] << ", " << dataOut.trajectory[i][1U] << ", " << dataOut.trajectory[i][2U] << "} ";
            std::cout << "u,v,r={" << dataOut.trajectory[i][3U] << ", " << dataOut.trajectory[i][4U] << ", " << dataOut.trajectory[i][5U] << "} ";
            std::cout << "X,Y,N={" << dataOut.trajectory[i][6U] << ", " << dataOut.trajectory[i][7U] << ", " << dataOut.trajectory[i][8U] << "} ";
            std::cout << "Xc,Yc,Nc={" << dataOut.trajectory[i][9U] << ", " << dataOut.trajectory[i][10U] << ", " << dataOut.trajectory[i][11U] << "}\n";
        }
        std::cout << std::endl;
    }
    return 0;
}
