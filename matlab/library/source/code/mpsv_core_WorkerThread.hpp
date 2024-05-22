#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_Event.hpp>
#ifndef MPSV_DONT_USE_OMP
#include <omp.h>
#endif


namespace mpsv {


namespace core {


/**
 * @brief The states of a worker thread.
 */
enum WorkerThreadState : uint8_t {
    offline = 0,   // Thread has not been started.
    standby = 1,   // Thread is suspended and waits until it's resumed or stopped.
    running = 2    // Thread is running and executes the user callback.
};


/**
 * @brief Represents a suspendable worker thread.
 */
class WorkerThread {
    public:
        /**
         * @brief Construct a new worker thread object.
         */
        WorkerThread() noexcept {
            isStarted = false;
            stopWorkerThread = false;
            state = mpsv::core::WorkerThreadState::offline;
            commandedState = mpsv::core::WorkerThreadState::standby;
        }

        /**
         * @brief Destroy the worker thread object.
         */
        ~WorkerThread() noexcept { Stop(); }

        /**
         * @brief Start the worker thread with a given priority.
         * @param[in] callbackRun A callable that is executed during the running state of the worker thread.
         * @param[in] threadPriority The priority to be set for the worker thread (1: low, 99: high).
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @details If the thread has already been started, nothing happens. The result of setting the thread priority is ignored.
         * @note The worker thread starts in standby mode. Call @ref Resume to enter the running state.
         */
        template <typename CallbackRun> void Start(CallbackRun callbackRun, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic) noexcept {
            if(!isStarted){
                // start the worker thread
                workerThread = std::thread(&WorkerThread::WorkerThreadFunction<CallbackRun>, this, callbackRun, ompNumThreads, ompDynamic);
                isStarted = true;

                // set priority and ignore errors
                struct sched_param param;
                param.sched_priority = threadPriority;
                (void) pthread_setschedparam(workerThread.native_handle(), SCHED_FIFO, &param);
            }
        }

        /**
         * @brief Stop the worker thread.
         */
        void Stop(void) noexcept {
            // set flags to interrupt running thread
            stopWorkerThread = true;
            eventLeaveStandby.NotifyOne(1);

            // Wait until thread terminates and terminate solver
            if(workerThread.joinable()){
                workerThread.join();
            }

            // set default values
            isStarted = false;
            stopWorkerThread = false;
            state = mpsv::core::WorkerThreadState::offline;
            commandedState = mpsv::core::WorkerThreadState::standby;
        }

        /**
         * @brief Suspend the worker thread.
         * @details The next time the thread is scheduled, it goes into standby mode. A running user callback (@ref ThreadCallbackRunning) cannot be interrupted.
         */
        void Suspend(void) noexcept {
            commandedState = mpsv::core::WorkerThreadState::standby;
        }

        /**
         * @brief Resume a suspended thread and enter the running state.
         */
        void Resume(void) noexcept {
            commandedState = mpsv::core::WorkerThreadState::running;
            eventLeaveStandby.NotifyOne();
        }

        /**
         * @brief Get the current state of the worker thread.
         * @return The current state of the worker thread.
         */
        mpsv::core::WorkerThreadState GetState(void) noexcept {
            return state;
        }


    private:
        std::thread workerThread;                                    // The internal worker thread.
        std::atomic<bool> isStarted;                                 // True if the worker thread has been started, false otherwise.
        std::atomic<bool> stopWorkerThread;                          // True if the worker thread should be stopped, false otherwise.
        std::atomic<mpsv::core::WorkerThreadState> state;            // Internal state of the worker thread.
        std::atomic<mpsv::core::WorkerThreadState> commandedState;   // The commanded state for this worker thread (either standby or running).
        mpsv::core::Event eventLeaveStandby;                         // The event to be set if the standby state is to be left.

        /**
         * @brief The thread function of the worker thread.
         * @param[in] callbackRun A callable that is executed during the running state of the worker thread.
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         */
        template <typename CallbackRun> void WorkerThreadFunction(CallbackRun callbackRun, int32_t ompNumThreads, int32_t ompDynamic) noexcept {
            #ifdef MPSV_DONT_USE_OMP
            (void) ompNumThreads;
            (void) ompdynamic;
            #else
            if(ompNumThreads > 0){
                omp_set_num_threads(static_cast<int>(ompNumThreads));
            }
            if(ompDynamic >= 0){
                omp_set_dynamic(static_cast<int>(ompDynamic));
            }
            #endif
            while(!stopWorkerThread){
                switch(commandedState){
                    case mpsv::core::WorkerThreadState::offline:
                    case mpsv::core::WorkerThreadState::standby:
                        state = mpsv::core::WorkerThreadState::standby;
                        (void) eventLeaveStandby.Wait(); // wait for the leave event (blocking call, ignore actual returned event value)
                        break;
                    case mpsv::core::WorkerThreadState::running:
                        state = mpsv::core::WorkerThreadState::running;
                        callbackRun();
                        break;
                }
            }
            state = mpsv::core::WorkerThreadState::offline;
        }
};


} /* namespace: core */


/**
 * @brief Convert a worker thread state to a string.
 * @param[in] state The worker thread state to be converted.
 * @return A string holding the converted value.
 */
inline std::string to_string(mpsv::core::WorkerThreadState state) noexcept {
    switch(state){
        case mpsv::core::WorkerThreadState::offline: return std::string("offline");
        case mpsv::core::WorkerThreadState::standby: return std::string("standby");
        case mpsv::core::WorkerThreadState::running: return std::string("running");
    }
    return std::string("unknown");
}


} /* namespace: mpsv */

