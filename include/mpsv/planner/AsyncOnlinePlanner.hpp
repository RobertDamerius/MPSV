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
            pthread_mutex_init(&m_mtxDatabase, &attr);
            pthread_mutexattr_destroy(&attr);
            m_terminate = false;
            m_previousParameterTimestamp = std::numeric_limits<double>::quiet_NaN();
        }

        /**
         * @brief Destroy the asynchronous online planner object.
         * @details Calls the @ref Terminate member function.
         */
        virtual ~AsyncOnlinePlanner() noexcept {
            Terminate();
            pthread_mutex_destroy(&m_mtxDatabase);
        }

        /**
         * @brief Initialize the asynchronous online planner.
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
         * @details Call this function before calling any of the other member functions.
         */
        [[nodiscard]] bool Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs) noexcept {
            m_terminate = false;
            pthread_mutex_lock(&m_mtxDatabase);
            m_database.Clear();
            pthread_mutex_unlock(&m_mtxDatabase);
            m_onlinePlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);
            m_workerThread.Start(std::bind(&AsyncOnlinePlanner::WorkerThreadRun, this), threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
            return true;
        }

        /**
         * @brief Terminate the asynchronous online planner.
         */
        void Terminate(void) noexcept {
            m_terminate = true;
            m_onlinePlanner.Interrupt();
            m_workerThread.Stop();
            pthread_mutex_lock(&m_mtxDatabase);
            m_database.Clear();
            pthread_mutex_unlock(&m_mtxDatabase);
            m_terminate = false;
            m_previousParameterTimestamp = std::numeric_limits<double>::quiet_NaN();
        }

        /**
         * @brief Set new input data for the asynchronous online planner.
         * @param[in] input The new input data to be set. This data is moved to the internal data and is undefined after this call!
         * @details The worker thread is suspended or resumed depending on the enable-flag of the input.
         * @note The timestamp of the input data is used as time source for online trajectory calculation. The output timestamp relates to this time source.
         * @note The static obstacles used for planning always relate to the latest input data.
         * @note The geographical origin used for planning always relate to the latest input data.
         */
        void SetInput(mpsv::planner::AsyncOnlinePlannerInput& input) noexcept {
            // check for data validity
            error_code inputError = input.IsValid();

            // move valid input to internal database
            pthread_mutex_lock(&m_mtxDatabase);
            m_database.inputError = inputError;
            if(error_code::NONE == inputError){
                if(input.reset){ // detected reset: interrupt planner
                    m_database.resetIsRequired = true; // make sure the reset is not lost if SetInput is called with high frequency
                    m_onlinePlanner.Interrupt();
                }
                input.MoveTo(m_database.input);
                m_database.timeoutCounter.Start();
            }
            bool suspendWorkerThread = (error_code::NONE != m_database.inputError) || (error_code::NONE != m_database.parameterError) || !m_database.input.enable;
            pthread_mutex_unlock(&m_mtxDatabase);

            // suspend or resume worker thread
            if(suspendWorkerThread){
                m_workerThread.Suspend();
                pthread_mutex_lock(&m_mtxDatabase);
                m_database.resetIsRequired = true; // reset is required when leaving the standby mode
                pthread_mutex_unlock(&m_mtxDatabase);
            }
            else{
                m_workerThread.Resume();
            }
        }

        /**
         * @brief Set new parameter data for the asynchronous online planner.
         * @param[in] parameter The new parameter data to be set. This data is moved to the internal data and is undefined after this call!
         * @note The internal online planner resets automatically if valid parameters are set and the counter value has been changed.
         */
        void SetParameter(mpsv::planner::AsyncOnlinePlannerParameterSet& parameter) noexcept {
            // check for data validity
            error_code parameterError = parameter.IsValid();
            bool timestampChanged = !std::isfinite(m_previousParameterTimestamp) || (m_previousParameterTimestamp != parameter.timestamp);

            // move valid parameters to internal database
            pthread_mutex_lock(&m_mtxDatabase);
            m_database.parameterError = parameterError;
            if((error_code::NONE == parameterError) && timestampChanged){
                m_previousParameterTimestamp = parameter.timestamp;
                parameter.MoveTo(m_database.parameter);
                m_database.parameterUpdateRequired = true;
            }
            pthread_mutex_unlock(&m_mtxDatabase);
        }

        /**
         * @brief Get the current output of the asynchronous online planner.
         * @return The output of the asynchronous online planner.
         */
        [[nodiscard]] mpsv::planner::AsyncOnlinePlannerOutput GetOutput(void){
            mpsv::planner::AsyncOnlinePlannerOutput result;
            pthread_mutex_lock(&m_mtxDatabase);
            result = m_database.output;
            result.threadState = m_workerThread.GetState();
            if(result.threadState != mpsv::runtime::WorkerThreadState::running){
                result.timestampInput = m_database.input.timestamp;
                result.timestampParameter = m_database.parameter.timestamp;
                result.inputError = m_database.inputError;
                result.parameterError = m_database.parameterError;
            }
            pthread_mutex_unlock(&m_mtxDatabase);
            return result;
        }


    protected:
        /**
         * @brief This function is called from inside the worker thread of the planner, if a planning problem has been solved and new
         * output data is available. The actual data is obtained via @ref GetOutput.
         */
        virtual void CallbackNewOutputAvailable(void){}


    private:
        mpsv::runtime::WorkerThread m_workerThread;     // The internal worker thread.
        mpsv::planner::OnlinePlanner m_onlinePlanner;   // The online sequential planner.
        std::atomic<bool> m_terminate;                  // True if the asynchronous online planner should be terminated, false otherwise.
        volatile double m_previousParameterTimestamp;   // Previous parameter timestamp (used to detect new parameter sets).

        /* internal database */
        class Database {
            public:
                mpsv::planner::AsyncOnlinePlannerInput input;              // The latest valid input data.
                mpsv::planner::AsyncOnlinePlannerParameterSet parameter;   // The latest valid input parameter set.
                mpsv::planner::AsyncOnlinePlannerOutput output;            // Output of the online planner.
                mpsv::runtime::PerformanceCounter timeoutCounter;          // Performance counter to measure timeouts.
                mpsv::error_code inputError;                               // Error code of the latest @ref SetInput call.
                mpsv::error_code parameterError;                           // Error code of the latest @ref SetParameter call.
                bool resetIsRequired;                                      // True if an automatic reset is required or a reset has been commanded via @ref SetInput, false otherwise.
                bool parameterUpdateRequired;                              // True if parameters should be applied to the planner, false otherwise.

                /**
                 * @brief Construct a new database object and set default values.
                 */
                Database() noexcept { Clear(); }

                /**
                 * @brief Clear the database.
                 */
                void Clear(void) noexcept {
                    input.Clear();
                    parameter.Clear();
                    output.Clear();
                    timeoutCounter.Reset();
                    inputError = error_code::NOT_AVAILABLE;
                    parameterError = error_code::NOT_AVAILABLE;
                    resetIsRequired = true;
                    parameterUpdateRequired = false;
                }
        } m_database;
        pthread_mutex_t m_mtxDatabase;   // Protect @ref m_database.


        /**
         * @brief The callback function that is executed if the next asynchronous step calculation is performed by the worker thread.
         */
        void WorkerThreadRun(void) noexcept {
            // do nothing if thread should be terminated
            if(m_terminate){
                return;
            }

            // storage for current input, output and parameter data
            mpsv::planner::AsyncOnlinePlannerInput dataIn;
            mpsv::planner::AsyncOnlinePlannerOutput dataOut;
            mpsv::planner::OnlinePlannerParameterSet dataParameter;

            // get input, apply parameters, handle errors
            pthread_mutex_lock(&m_mtxDatabase);
            if(m_database.timeoutCounter.TimeToStart() >= m_database.parameter.timeoutInput){
                m_database.inputError = (m_database.inputError == error_code::NONE) ? error_code::NOT_AVAILABLE : m_database.inputError;
            }
            bool reset = m_database.resetIsRequired || m_database.parameterUpdateRequired;
            m_database.parameterUpdateRequired = false;
            bool suspend = (error_code::NONE != m_database.inputError) || (error_code::NONE != m_database.parameterError) || !m_database.input.enable;
            m_database.resetIsRequired = false;
            dataIn = m_database.input;
            dataParameter = m_database.parameter;
            dataOut.timestampInput = m_database.input.timestamp;
            dataOut.timestampParameter = m_database.parameter.timestamp;
            dataOut.inputError = m_database.inputError;
            dataOut.parameterError = m_database.parameterError;
            pthread_mutex_unlock(&m_mtxDatabase);

            // perform a reset if commanded and solve the planning problem
            if(!suspend){
                m_onlinePlanner.Solve(dataOut, dataIn, dataParameter, reset);
            }

            // move result to internal database and run callback
            pthread_mutex_lock(&m_mtxDatabase);
            if((dataOut.errorCode != mpsv::error_code::NONE) || suspend){
                m_database.resetIsRequired = true; // reset is required when leaving the standby mode
                m_workerThread.Suspend();
            }
            dataOut.MoveTo(m_database.output);
            pthread_mutex_unlock(&m_mtxDatabase);
            CallbackNewOutputAvailable();
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
