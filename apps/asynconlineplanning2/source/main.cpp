#include <ExamplePlanner.hpp>
#include <ExampleAsyncOnlinePlanning2.hpp>


int main(int, char**){
    // construct and start the planner
    ExamplePlanner planner;
    if(!planner.Start()){
        std::cerr << "ERROR: Could not start the planner!\n";
        return 0;
    }

    // construct a timer for timestamp generation
    mpsv::core::PerformanceCounter timer;
    timer.Start();

    // get parameter and input
    mpsv::planner::AsyncOnlinePlannerParameterSet parameter = example::GetParameterSet(timer.TimeToStart());
    mpsv::planner::AsyncOnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());

    // save parameter and input to a data file
    mpsv::core::DataLogFile f;
    std::string filename = example::GetDataFilename();
    f.Open(filename);
    parameter.WriteToFile(f, "parameter.");
    dataIn.WriteToFile(f, "input.");
    f.Close();

    // set parameter
    planner.SetParameter(parameter);

    // set new input data for the next 120 seconds
    for(int i = 0; i != 120; ++i){
        dataIn = example::GetInput(timer.TimeToStart());
        planner.SetInput(dataIn);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // wait and bring the planner into standby mode
    std::this_thread::sleep_for(std::chrono::seconds(5));
    dataIn = example::GetInput(timer.TimeToStart());
    dataIn.enable = false;
    planner.SetInput(dataIn);

    // sleep for another second and stop the planner
    std::this_thread::sleep_for(std::chrono::seconds(1));
    planner.Stop();
    return 0;
}

