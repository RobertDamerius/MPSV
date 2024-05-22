#include <mpsv/mpsv.hpp>
#include <ExampleOnlinePlanning.hpp>


int main(int, char**){
    // construct and initialize the online planner
    mpsv::planner::OnlinePlanner planner;
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    if(!planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples)){
        std::cerr << "ERROR: Could not initialize planner!\n";
        return 0;
    }

    // apply a parameter set to the online planner
    mpsv::planner::OnlinePlannerParameterSet parameter = example::GetParameterSet();
    if(!planner.ApplyParameterSet(parameter)){
        std::cerr << "ERROR: Could apply parameter set!\n";
        return 0;
    }

    // solve the online planning problem
    std::cout << "solve online planning problem (5 steps)\n\n";
    mpsv::core::PerformanceCounter timer;
    timer.Start();
    for(int i = 0; i != 5; ++i){
        // get valid input data for planning problem
        mpsv::planner::OnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
        if(!dataIn.IsValid()){
            std::cerr << "ERROR: Input data is invalid!\n";
            return 0;
        }

        // solve the problem
        mpsv::planner::OnlinePlannerOutput dataOut;
        planner.Solve(dataOut, dataIn);

        // print result (at most 10 trajectory points)
        std::cout << "timestamp:        " << dataOut.timestamp << "\n";
        std::cout << "sampletime:       " << dataOut.sampletime << "\n";
        std::cout << "error:            " << dataOut.error << "\n";
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

    // terminate the planner
    planner.Terminate();
    return 0;
}

