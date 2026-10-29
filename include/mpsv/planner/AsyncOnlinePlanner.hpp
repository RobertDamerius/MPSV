#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/runtime/WorkerThread.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/common/ErrorCode.hpp>
#include <mpsv/planner/OnlinePlanner.hpp>
#include <mpsv/planner/AsyncOnlinePlannerInput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerOutput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerParameterSet.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class runs the online planner in a separate thread and continuously solves sequential planning problems (path + motion planning).
 * It solves motion planning problems for a future timepoint and merges the solution trajectory seamless with the previous solution. An overwritable
 * callback member function informs about new output data.
 */
class AsyncOnlinePlanner {
    public:
        /**
         * @brief Construct a new asynchronous online planner object.
         */
        AsyncOnlinePlanner() noexcept {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
            pthread_mutex_init(&m_mtxData, &attr);
            pthread_mutexattr_destroy(&attr);
            m_dataOut.errorCode = mpsv::error_code::NOT_AVAILABLE;
            m_terminate = false;
        }

        /**
         * @brief Destroy the asynchronous online planner object.
         * @details Calls the @ref Stop member function.
         */
        virtual ~AsyncOnlinePlanner() noexcept {
            Stop();
            pthread_mutex_destroy(&m_mtxData);
        }

        /* Make this class non-copyable */
        AsyncOnlinePlanner(const AsyncOnlinePlanner&) = delete;
        AsyncOnlinePlanner& operator=(const AsyncOnlinePlanner&) = delete;

        /**
         * @brief Start the asynchronous online planner.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @param[in] maxNumCells Maximum number of cells to be used in the cost map.
         * @param[in] threadPriority The priority to be set for the thread (1: low, 99: high).
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero!
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored.
         * @param[in] cpuCoreIDs (linux-only) List of CPU cores on which the worker thread is eligible to run. Only IDs in range and available to the worker are used.
         * @return True if success, false otherwise.
         */
        [[nodiscard]] bool Start(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs) noexcept {
            m_terminate = false;
            pthread_mutex_lock(&m_mtxData);
            m_dataIn.Clear();
            m_dataParam.Clear();
            m_dataOut.Clear();
            m_dataOut.errorCode = mpsv::error_code::NOT_AVAILABLE;
            pthread_mutex_unlock(&m_mtxData);
            m_onlinePlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);
            m_workerThread.Start(std::bind(&AsyncOnlinePlanner::WorkerThreadRun, this), threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
            return true;
        }

        /**
         * @brief Stop the asynchronous online planner.
         */
        void Stop(void) noexcept {
            m_terminate = true;
            m_onlinePlanner.Interrupt();
            m_workerThread.Stop();
            pthread_mutex_lock(&m_mtxData);
            m_dataIn.Clear();
            m_dataParam.Clear();
            m_dataOut.Clear();
            m_dataOut.errorCode = mpsv::error_code::NOT_AVAILABLE;
            pthread_mutex_unlock(&m_mtxData);
            m_terminate = false;
        }

        /**
         * @brief Set new input and parameter data for the asynchronous online planner.
         * @param[in] input The new input data to be set. This data is moved to the internal data and is undefined after this call!
         * @param[in] parameter The new parameter data to be set. This data is moved to the internal data and is undefined after this call!
         * @details The worker thread is suspended or resumed depending on the enable-flag of the input.
         * @note The timestamp of the input data is used as time source for online trajectory calculation. The output timestamp relates to this time source.
         * @note The static obstacles used for planning always relate to the latest input data.
         * @note The geographical origin used for planning always relate to the latest input data.
         */
        void SetInput(mpsv::planner::AsyncOnlinePlannerInput& input, mpsv::planner::AsyncOnlinePlannerParameterSet& parameter) noexcept {
            // move input to internal data and handle reset flag
            pthread_mutex_lock(&m_mtxData);
            bool suspendWorkerThread = !input.enable;
            input.reset |= suspendWorkerThread; // reset is required when going into standby and later leaving the standby mode
            input.reset |= m_dataIn.reset; // make sure the reset is not lost if SetInput is called with high frequency
            bool interruptPlanner = input.reset || suspendWorkerThread;
            input.MoveTo(m_dataIn);
            parameter.MoveTo(m_dataParam);
            pthread_mutex_unlock(&m_mtxData);

            // suspend or resume worker thread
            if(suspendWorkerThread){
                m_workerThread.Suspend();
            }
            else{
                m_workerThread.Resume();
            }
            if(interruptPlanner){
                m_onlinePlanner.Interrupt();
            }
        }

        /**
         * @brief Get the current output of the asynchronous online planner.
         * @return The output of the asynchronous online planner.
         */
        [[nodiscard]] mpsv::planner::AsyncOnlinePlannerOutput GetOutput(void){
            mpsv::planner::AsyncOnlinePlannerOutput result;
            pthread_mutex_lock(&m_mtxData);
            result = m_dataOut;
            result.threadState = m_workerThread.GetState();
            pthread_mutex_unlock(&m_mtxData);
            return result;
        }

    protected:
        /**
         * @brief This function is called from inside the worker thread of the planner, if a planning problem has been solved and new
         * output data is available. The actual data is obtained via @ref GetOutput.
         */
        virtual void CallbackNewOutputAvailable(void){}

    private:
        mpsv::runtime::WorkerThread m_workerThread;                  // The internal worker thread.
        mpsv::planner::OnlinePlanner m_onlinePlanner;                // The online sequential planner.
        std::atomic<bool> m_terminate{false};                        // True if the asynchronous online planner should be terminated, false otherwise.
        mpsv::planner::AsyncOnlinePlannerInput m_dataIn;             // Thread-safe input data storage.
        mpsv::planner::AsyncOnlinePlannerParameterSet m_dataParam;   // Thread-safe parameter data storage.
        mpsv::planner::AsyncOnlinePlannerOutput m_dataOut;           // Thread-safe output data storage.
        pthread_mutex_t m_mtxData;                                   // Protect @ref m_dataIn, @ref m_dataParam and @ref m_dataOut.

        /**
         * @brief The callback function that is executed if the next asynchronous step calculation is performed by the worker thread.
         */
        void WorkerThreadRun(void) noexcept {
            if(m_terminate){
                return;
            }

            // copy thread-safe input data to local storage
            pthread_mutex_lock(&m_mtxData);
            mpsv::planner::AsyncOnlinePlannerInput dataIn = m_dataIn;
            mpsv::planner::AsyncOnlinePlannerParameterSet dataParam = m_dataParam;
            m_dataIn.reset = !m_dataIn.enable;
            pthread_mutex_unlock(&m_mtxData);

            // solve the planning problem
            mpsv::planner::AsyncOnlinePlannerOutput dataOut;
            if(dataIn.enable){
                m_onlinePlanner.Solve(dataOut, dataIn, dataParam, dataIn.reset);
            }

            // move result to internal database and run callback
            pthread_mutex_lock(&m_mtxData);
            dataOut.MoveTo(m_dataOut);
            if((m_dataOut.errorCode != mpsv::error_code::NONE) || !dataIn.enable){
                m_dataIn.reset = true; // reset is required when going into standby and later leaving the standby mode
                m_workerThread.Suspend();
            }
            pthread_mutex_unlock(&m_mtxData);
            CallbackNewOutputAvailable();
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
