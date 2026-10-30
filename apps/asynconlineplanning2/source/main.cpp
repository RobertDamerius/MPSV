#include <ExamplePlanner.hpp>
#include <ExampleAsyncOnlinePlanning2.hpp>


int main(int, char**){
    // construct and start the planner
    ExamplePlanner planner;
    planner.Start();

    // get input data and save to file
    mpsv::runtime::PerformanceCounter timer;
    mpsv::planner::AsyncOnlinePlannerInput dataIn = example::GetInput(timer.TimeToStart());
    mpsv::planner::AsyncOnlinePlannerParameterSet parameter = example::GetParameterSet();
    mpsv::logging::File f;
    std::string filename = example::GetDataFilename();
    if(f.Open(filename)){
        parameter.WriteToFile(f, "parameter.");
        dataIn.WriteToFile(f, "input.");
        f.Close();
    }

    // set new input data for the next 120 seconds
    for(int i = 0; i != 120; ++i){
        dataIn = example::GetInput(timer.TimeToStart());
        parameter = example::GetParameterSet();
        planner.SetInput(dataIn, parameter);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // wait and bring the planner into standby mode
    std::this_thread::sleep_for(std::chrono::seconds(5));
    dataIn = example::GetInput(timer.TimeToStart());
    dataIn.enable = false;
    parameter = example::GetParameterSet();
    planner.SetInput(dataIn, parameter);

    // sleep for another second and stop the planner
    std::this_thread::sleep_for(std::chrono::seconds(1));
    planner.Stop();
    return 0;
}
