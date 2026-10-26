#include <mpsv/mpsv.hpp>
#include <ExampleSequentialPlanning.hpp>


int main(int, char**){
    // construct and initialize the sequential planner
    mpsv::planner::SequentialPlanner planner;
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    constexpr uint32_t maxNumCells = 4096 * 4096;
    planner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);

    // get valid input data for planning problem
    mpsv::planner::SequentialPlannerInput dataIn = example::GetInput();
    mpsv::planner::SequentialPlannerParameterSet parameter = example::GetParameterSet();
    if(mpsv::error_code::NONE != dataIn.IsValid()){
        std::cerr << "ERROR: Input data is invalid!\n";
        return 0;
    }
    if(mpsv::error_code::NONE != parameter.IsValid()){
        std::cerr << "ERROR: Parameter data is invalid!\n";
        return 0;
    }

    // solve the sequential planning problem
    constexpr double maxComputationTimePathPlanning = 1.0;
    constexpr double maxComputationTimeMotionPlanning = 1.0;
    mpsv::planner::SequentialPlannerOutput dataOut;
    std::cout << "solve sequential planning problem (computation time (path): " << maxComputationTimePathPlanning << " s, computation time (motion): " << maxComputationTimeMotionPlanning << " s)\n\n";
    planner.Solve(dataOut, dataIn, parameter, maxComputationTimePathPlanning, maxComputationTimeMotionPlanning);

    // write results to file
    mpsv::logging::File f;
    std::string filename = example::GetDataFilename();
    if(f.Open(filename)){
        parameter.WriteToFile(f, "parameter.");
        dataIn.WriteToFile(f, "input.");
        dataOut.WriteToFile(f, "output.");
        planner.WriteToFile(f, "planner.");
        f.Close();
    }

    // print result (at most 10 trajectory points)
    std::cout << "pathPlanner.goalReached:                     " << dataOut.pathPlanner.goalReached << "\n";
    std::cout << "pathPlanner.errorCode:                       " << static_cast<int32_t>(dataOut.pathPlanner.errorCode) << "\n";
    std::cout << "pathPlanner.numberOfPerformedIterations:     " << dataOut.pathPlanner.numberOfPerformedIterations << "\n";
    std::cout << "pathPlanner.cost:                            " << dataOut.pathPlanner.cost << "\n";
    std::cout << "pathPlanner.path:                            " << dataOut.pathPlanner.path.size() << " poses\n";
    for(auto&& pose : dataOut.pathPlanner.path){
        std::cout << "    x,y,psi={" << pose[0] << ", " << pose[1] << ", " << pose[2] << "}\n";
    }
    std::cout << "motionPlanner.goalReached:                   " << dataOut.motionPlanner.goalReached << "\n";
    std::cout << "motionPlanner.errorCode:                     " << static_cast<int32_t>(dataOut.motionPlanner.errorCode) << "\n";
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
    return 0;
}
