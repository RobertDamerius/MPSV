#include <Planner.hpp>
#include <MainApplication.hpp>


bool Planner::Start(const PlannerConfiguration plannerConf){
    if(!Initialize(plannerConf.pathMaxNumNodes, plannerConf.pathMaxNumSamples, plannerConf.motionMaxNumNodes, plannerConf.motionMaxNumSamples, plannerConf.threadPriorityPlanner, plannerConf.ompNumThreads, plannerConf.ompDynamic)){
        return false;
    }
    if(!periodicTimer.Start(plannerConf.updatePeriod)){
        Terminate();
        return false;
    }
    periodicThread = std::thread(&Planner::PeriodicUpdateThread, this);
    struct sched_param param;
    param.sched_priority = plannerConf.threadPriorityUpdater;
    if(0 != pthread_setschedparam(periodicThread.native_handle(), SCHED_FIFO, &param)){
        PrintW("Could not set thread priority %d for periodic update thread!\n", plannerConf.threadPriorityUpdater);
    }
    return true;
}

void Planner::Stop(void){
    periodicTimer.Stop();
    if(periodicThread.joinable()){
        periodicThread.join();
    }
    Terminate();
}

void Planner::CallbackNewOutputAvailable(void){
    mpsv::planner::AsyncOnlinePlannerOutput plannerOutput = GetOutput();
    mainApplication.networkManager.Send(plannerOutput);
}

void Planner::PeriodicUpdateThread(void){
    while(periodicTimer.WaitForTick()){
        CallbackNewOutputAvailable();
    }
}

