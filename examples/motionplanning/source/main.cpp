#include <mpsv/mpsv.hpp>
#include <ExampleMotionPlanning.hpp>


int main(int, char**) {
    // construct motion planner
    constexpr std::int16_t maxNumNodes{200};
    constexpr std::uint32_t maxNumSamples{1000000U};
    constexpr std::uint32_t maxNumCells{4096U * 4096U};
    mpsv::planner::MotionPlanner planner(maxNumNodes, maxNumSamples, maxNumCells);

    // solve the motion planning problem
    constexpr double maxComputationTime{1.0};
    mpsv::planner::MotionPlannerInput dataIn = example::GetInput();
    mpsv::planner::MotionPlannerParameterSet parameter = example::GetParameterSet();
    mpsv::planner::MotionPlannerOutput dataOut;
    std::cout << "solve motion planning problem (computation time: " << maxComputationTime << " s)\n\n";
    planner.Solve(dataOut, dataIn, parameter, maxComputationTime);

    // print result (at most 10 trajectory points)
    std::cout << "goalReached:                   " << dataOut.goalReached << "\n";
    std::cout << "errorCode:                     " << static_cast<std::int32_t>(dataOut.errorCode) << "\n";
    std::cout << "numberOfPerformedIterations:   " << dataOut.numberOfPerformedIterations << "\n";
    std::cout << "cost:                          " << dataOut.cost << "\n";
    std::cout << "sampletime:                    " << dataOut.sampletime << "\n";
    std::cout << "trajectory:                    " << dataOut.trajectory.size() << " points\n";
    for (std::size_t i{0U}; (i < dataOut.trajectory.size()) && (i < 10U); ++i) {
        std::cout << "    x,y,psi={" << dataOut.trajectory[i][0U] << ", " << dataOut.trajectory[i][1U] << ", " << dataOut.trajectory[i][2U] << "} ";
        std::cout << "u,v,r={" << dataOut.trajectory[i][3U] << ", " << dataOut.trajectory[i][4U] << ", " << dataOut.trajectory[i][5U] << "} ";
        std::cout << "X,Y,N={" << dataOut.trajectory[i][6U] << ", " << dataOut.trajectory[i][7U] << ", " << dataOut.trajectory[i][8U] << "} ";
        std::cout << "Xc,Yc,Nc={" << dataOut.trajectory[i][9U] << ", " << dataOut.trajectory[i][10U] << ", " << dataOut.trajectory[i][11U] << "}\n";
    }
    return 0;
}
