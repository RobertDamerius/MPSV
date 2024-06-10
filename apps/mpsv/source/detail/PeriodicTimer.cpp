#include <PeriodicTimer.hpp>


PeriodicTimer::PeriodicTimer(){
    #ifdef _WIN32
    hTimer = NULL;
    #else
    fdTimer = -1;
    #endif
}

bool PeriodicTimer::Start(double sampletime){
    if(!std::isfinite(sampletime) || (sampletime <= 0.0)){
        PrintE("Invalid sampletime %lf for a periodic timer!\n", sampletime);
        return false;
    }
    double timeToWaitBeforeStart = sampletime;
    Stop();
    #ifdef _WIN32
    if(sampletime < 0.001){
        PrintE("A sample rate less than 1 millisecond can not be set on windows!\n");
        return false;
    }
    hTimer = CreateWaitableTimer(NULL, false, NULL); // No SECURITY_ATRIBUTES, no manual reset, no timer name
    if(!hTimer){
        PrintE("Could not create waitable timer!\n");
        return false;
    }
    LARGE_INTEGER lpDueTime;
    lpDueTime.QuadPart = (LONGLONG)(timeToWaitBeforeStart * 1e7 * -1); // in 100 ns, negative for relative time (positive would be absolute time using UTC)
    LONG lPeriod = (LONG)(sampletime * 1e3);
    if(!SetWaitableTimer(hTimer, &lpDueTime, lPeriod, NULL, NULL, false)){ // No completetion routine, no args for completetion routine, no resume
        PrintE("Could not set waitable timer!\n");
        CloseHandle(hTimer);
        hTimer = NULL;
    }
    #else
    if((fdTimer = timerfd_create(CLOCK_MONOTONIC, 0)) < 0){ // 0: no flags
        PrintE("Could not create timer!\n");
        return false;
    }
    struct itimerspec its;
    its.it_interval.tv_sec = (time_t)sampletime;
    its.it_interval.tv_nsec = (sampletime - (time_t)sampletime) * 1000000000;
    its.it_value.tv_sec = (time_t)timeToWaitBeforeStart;
    its.it_value.tv_nsec = (timeToWaitBeforeStart - (time_t)timeToWaitBeforeStart) * 1000000000;
    if(timerfd_settime(fdTimer, 0, &its, nullptr) < 0){ // 0: no flags
        PrintE("Could not set time for timer!\n");
        close(fdTimer);
        fdTimer = -1;
        return false;
    }
    #endif
    return true;
}

void PeriodicTimer::Stop(void){
    #ifdef _WIN32
    if(hTimer){
        CloseHandle(hTimer);
        hTimer = NULL;
    }
    #else
    if(fdTimer >= 0){
        close(fdTimer);
        fdTimer = -1;
    }
    #endif
}

bool PeriodicTimer::WaitForTick(void){
    #ifdef _WIN32
    bool result = (WaitForSingleObject(hTimer, INFINITE) == WAIT_OBJECT_0);
    return result;
    #else
    int s = -1;
    uint64_t exp = 0;
    for(;;){
        s = read(fdTimer, &exp, sizeof(exp));
        if((s == -1) && (errno == EINTR)){
            continue;
        }
        break;
    }
    return (s != -1);
    #endif
}

