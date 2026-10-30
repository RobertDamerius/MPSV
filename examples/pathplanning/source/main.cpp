#include <mpsv/mpsv.hpp>
#include <ExamplePathPlanning.hpp>


int main(int, char**){
    // construct path planner
    constexpr int16_t maxNumNodes = 200;
    constexpr uint32_t maxNumSamples = 1000000;
    constexpr uint32_t maxNumCells = 4096 * 4096;
    mpsv::planner::PathPlanner planner(maxNumNodes, maxNumSamples, maxNumCells);

    // solve the path planning problem
    constexpr double maxComputationTime = 1.0;
    mpsv::planner::PathPlannerInput dataIn = example::GetInput();
    mpsv::planner::PathPlannerParameterSet parameter = example::GetParameterSet();
    mpsv::planner::PathPlannerOutput dataOut;
    std::cout << "solve path planning problem (computation time: " << maxComputationTime << " s)\n\n";
    planner.Solve(dataOut, dataIn, parameter, maxComputationTime);

    // print result
    std::cout << "goalReached:                   " << dataOut.goalReached << "\n";
    std::cout << "errorCode:                     " << static_cast<int32_t>(dataOut.errorCode) << "\n";
    std::cout << "numberOfPerformedIterations:   " << dataOut.numberOfPerformedIterations << "\n";
    std::cout << "cost:                          " << dataOut.cost << "\n";
    std::cout << "path:                          " << dataOut.path.size() << " poses\n";
    for(auto&& pose : dataOut.path){
        std::cout << "    x,y,psi={" << pose[0] << ", " << pose[1] << ", " << pose[2] << "}\n";
    }
    return 0;
}
