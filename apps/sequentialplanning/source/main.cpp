#include <mpsv/mpsv.hpp>
#include <ExampleSequentialPlanning.hpp>


int main(int, char**){
    // construct and initialize the sequential planner
    mpsv::planner::SequentialPlanner planner;
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    if(!planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples)){
        std::cerr << "ERROR: Could not initialize planner!\n";
        return 0;
    }

    // apply a parameter set to the sequential planner
    mpsv::planner::SequentialPlannerParameterSet parameter = example::GetParameterSet();
    if(!planner.ApplyParameterSet(parameter)){
        std::cerr << "ERROR: Could apply parameter set!\n";
        return 0;
    }

    // get valid input data for planning problem
    mpsv::planner::SequentialPlannerInput dataIn = example::GetInput();
    if(!dataIn.IsValid()){
        std::cerr << "ERROR: Input data is invalid!\n";
        return 0;
    }

    // solve the sequential planning problem
    constexpr double maxComputationTimePathPlanning = 1.0;
    constexpr double maxComputationTimeMotionPlanning = 1.0;
    mpsv::planner::SequentialPlannerOutput dataOut;
    std::cout << "solve sequential planning problem (computation time (path): " << maxComputationTimePathPlanning << " s, computation time (motion): " << maxComputationTimeMotionPlanning << " s)\n\n";
    planner.Solve(dataOut, dataIn, maxComputationTimePathPlanning, maxComputationTimeMotionPlanning);

    // print result (at most 10 trajectory points)
    std::cout << "pathPlanner.goalReached:                     " << dataOut.pathPlanner.goalReached << "\n";
    std::cout << "pathPlanner.isFeasible:                      " << dataOut.pathPlanner.isFeasible << "\n";
    std::cout << "pathPlanner.outOfNodes:                      " << dataOut.pathPlanner.outOfNodes << "\n";
    std::cout << "pathPlanner.numberOfPerformedIterations:     " << dataOut.pathPlanner.numberOfPerformedIterations << "\n";
    std::cout << "pathPlanner.cost:                            " << dataOut.pathPlanner.cost << "\n";
    std::cout << "pathPlanner.path:                            " << dataOut.pathPlanner.path.size() << " poses\n";
    for(auto&& pose : dataOut.pathPlanner.path){
        std::cout << "    x,y,psi={" << pose[0] << ", " << pose[1] << ", " << pose[2] << "}\n";
    }
    std::cout << "motionPlanner.goalReached:                   " << dataOut.motionPlanner.goalReached << "\n";
    std::cout << "motionPlanner.isFeasible:                    " << dataOut.motionPlanner.isFeasible << "\n";
    std::cout << "motionPlanner.outOfNodes:                    " << dataOut.motionPlanner.outOfNodes << "\n";
    std::cout << "motionPlanner.numberOfPerformedIterations:   " << dataOut.motionPlanner.numberOfPerformedIterations << "\n";
    std::cout << "motionPlanner.cost:                          " << dataOut.motionPlanner.cost << "\n";
    std::cout << "motionPlanner.sampletime:                    " << dataOut.motionPlanner.sampletime << "\n";
    std::cout << "motionPlanner.trajectory:                    " << dataOut.motionPlanner.trajectory.size() << " points\n";
    for(size_t i = 0; (i < dataOut.motionPlanner.trajectory.size()) && (i < 10); ++i){
        std::cout << "    x,y,psi={" << dataOut.motionPlanner.trajectory[i][0] << ", " << dataOut.motionPlanner.trajectory[i][1] << ", " << dataOut.motionPlanner.trajectory[i][2] << "} ";
        std::cout << "u,v,r={" << dataOut.motionPlanner.trajectory[i][3] << ", " << dataOut.motionPlanner.trajectory[i][4] << ", " << dataOut.motionPlanner.trajectory[i][5] << "} ";
        std::cout << "X,Y,N={" << dataOut.motionPlanner.trajectory[i][6] << ", " << dataOut.motionPlanner.trajectory[i][7] << ", " << dataOut.motionPlanner.trajectory[i][8] << "} ";
        std::cout << "Xc,Yc,Nc={" << dataOut.motionPlanner.trajectory[i][9] << ", " << dataOut.motionPlanner.trajectory[i][10] << ", " << dataOut.motionPlanner.trajectory[i][11] << "}\n";
    }

    // terminate the planner
    planner.Terminate();
    return 0;
}

