#include <mpsv/mpsv.hpp>
#include <ExampleMotionPlanning.hpp>


int main(int, char**){
    // construct and initialize the motion planner
    mpsv::planner::MotionPlanner planner;
    constexpr int16_t maxNumNodes = 200;
    constexpr uint32_t maxNumSamples = 1000000;
    constexpr uint32_t maxNumCells = 4096 * 4096;
    planner.Reconfigure(maxNumNodes, maxNumSamples, maxNumCells);

    // get parameters for the motion planner
    mpsv::planner::MotionPlannerParameterSet parameter = example::GetParameterSet();
    if(mpsv::error_code::NONE != parameter.IsValid()){
        std::cerr << "ERROR: Parameter set is invalid!\n";
        return 0;
    }

    // get valid input data for planning problem
    mpsv::planner::MotionPlannerInput dataIn = example::GetInput();
    if(mpsv::error_code::NONE != dataIn.IsValid()){
        std::cerr << "ERROR: Input data is invalid!\n";
        return 0;
    }

    // prepare the planner and solve the motion planning problem
    constexpr double maxComputationTime = 1.0;
    mpsv::planner::MotionPlannerOutput dataOut;
    std::cout << "solve motion planning problem (computation time: " << maxComputationTime << " s)\n\n";
    planner.Solve(dataOut, dataIn, parameter, maxComputationTime);

    // print result (at most 10 trajectory points)
    std::cout << "goalReached:                   " << dataOut.goalReached << "\n";
    std::cout << "errorCode:                     " << static_cast<int32_t>(dataOut.errorCode) << "\n";
    std::cout << "numberOfPerformedIterations:   " << dataOut.numberOfPerformedIterations << "\n";
    std::cout << "cost:                          " << dataOut.cost << "\n";
    std::cout << "sampletime:                    " << dataOut.sampletime << "\n";
    std::cout << "trajectory:                    " << dataOut.trajectory.size() << " points\n";
    for(size_t i = 0; (i < dataOut.trajectory.size()) && (i < 10); ++i){
        std::cout << "    x,y,psi={" << dataOut.trajectory[i][0] << ", " << dataOut.trajectory[i][1] << ", " << dataOut.trajectory[i][2] << "} ";
        std::cout << "u,v,r={" << dataOut.trajectory[i][3] << ", " << dataOut.trajectory[i][4] << ", " << dataOut.trajectory[i][5] << "} ";
        std::cout << "X,Y,N={" << dataOut.trajectory[i][6] << ", " << dataOut.trajectory[i][7] << ", " << dataOut.trajectory[i][8] << "} ";
        std::cout << "Xc,Yc,Nc={" << dataOut.trajectory[i][9] << ", " << dataOut.trajectory[i][10] << ", " << dataOut.trajectory[i][11] << "}\n";
    }

    // terminate the planner
    return 0;
}
