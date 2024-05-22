#include <mpsv/mpsv.hpp>
#include <ExamplePathPlanning.hpp>


int main(int, char**){
    // construct and initialize the path planner
    mpsv::planner::PathPlanner planner;
    constexpr int16_t maxNumNodes = 200;
    constexpr uint32_t maxNumSamples = 1000000;
    if(!planner.Initialize(maxNumNodes, maxNumSamples)){
        std::cerr << "ERROR: Could not initialize planner!\n";
        return 0;
    }

    // apply a parameter set to the path planner
    mpsv::planner::PathPlannerParameterSet parameter = example::GetParameterSet();
    if(!planner.ApplyParameterSet(parameter)){
        std::cerr << "ERROR: Could apply parameter set!\n";
        return 0;
    }

    // get valid input data for planning problem
    mpsv::planner::PathPlannerInput dataIn = example::GetInput();
    if(!dataIn.IsValid()){
        std::cerr << "ERROR: Input data is invalid!\n";
        return 0;
    }

    // prepare the planner and solve the path planning problem
    constexpr double maxComputationTime = 1.0;
    mpsv::planner::PathPlannerOutput dataOut;
    std::cout << "solve path planning problem (computation time: " << maxComputationTime << " s)\n\n";
    planner.Prepare(dataOut, dataIn);
    planner.Solve(dataOut, dataIn, maxComputationTime);

    // print result
    std::cout << "goalReached:                   " << dataOut.goalReached << "\n";
    std::cout << "isFeasible:                    " << dataOut.isFeasible << "\n";
    std::cout << "outOfNodes:                    " << dataOut.outOfNodes << "\n";
    std::cout << "numberOfPerformedIterations:   " << dataOut.numberOfPerformedIterations << "\n";
    std::cout << "cost:                          " << dataOut.cost << "\n";
    std::cout << "path:                          " << dataOut.path.size() << " poses\n";
    for(auto&& pose : dataOut.path){
        std::cout << "    x,y,psi={" << pose[0] << ", " << pose[1] << ", " << pose[2] << "}\n";
    }

    // terminate the planner
    planner.Terminate();
    return 0;
}

