#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/WorkerThread.hpp>
#include <mpsv/core/PerformanceCounter.hpp>
#include <mpsv/core/ErrorCode.hpp>
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
            terminate = false;
            previousParameterTimestamp = std::numeric_limits<double>::quiet_NaN();
        }

        /**
         * @brief Destroy the asynchronous online planner object.
         * @details Calls the @ref Terminate member function.
         */
        virtual ~AsyncOnlinePlanner() noexcept { Terminate(); }

        /**
         * @brief Initialize the asynchronous online planner.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @param[in] threadPriority The priority to be set for the thread (1: low, 99: high).
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @param[in] cpuCoreIDs (linux-only) List of CPU cores on which the worker thread is eligible to run. If no or invalid IDs are specified, no thread affinity is set.
         * @return True if success, false otherwise.
         * @details Call this function before calling any of the other member functions
         */
        bool Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs) noexcept {
            terminate = false;
            mtxDatabase.lock();
            database.Clear();
            mtxDatabase.unlock();
            if(!onlinePlanner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples)){
                return false;
            }
            workerThread.Start(std::bind(&AsyncOnlinePlanner::WorkerThreadRun, this), threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
            return true;
        }

        /**
         * @brief Terminate the asynchronous online planner.
         */
        void Terminate(void) noexcept {
            terminate = true;
            onlinePlanner.Interrupt();
            workerThread.Stop();
            onlinePlanner.Terminate();
            mtxDatabase.lock();
            database.Clear();
            mtxDatabase.unlock();
            terminate = false;
            previousParameterTimestamp = std::numeric_limits<double>::quiet_NaN();
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
            mtxDatabase.lock();
            database.inputError = inputError;
            if(error_code::NONE == inputError){
                if(input.reset){ // detected reset: interrupt planner
                    database.resetIsRequired = true; // make sure the reset is not lost if SetInput is called with high frequency
                    onlinePlanner.Interrupt();
                }
                input.MoveTo(database.input);
                database.timeoutCounter.Start();
            }
            bool suspendWorkerThread = (error_code::NONE != database.inputError) || (error_code::NONE != database.parameterError) || !database.input.enable;
            mtxDatabase.unlock();

            // suspend or resume worker thread
            if(suspendWorkerThread){
                workerThread.Suspend();
                mtxDatabase.lock();
                database.resetIsRequired = true; // reset is required when leaving the standby mode
                mtxDatabase.unlock();
            }
            else{
                workerThread.Resume();
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
            bool timestampChanged = !std::isfinite(previousParameterTimestamp) || (previousParameterTimestamp != parameter.timestamp);

            // move valid parameters to internal database
            mtxDatabase.lock();
            database.parameterError = parameterError;
            if((error_code::NONE == parameterError) && timestampChanged){
                previousParameterTimestamp = parameter.timestamp;
                parameter.MoveTo(database.parameter);
                database.parameterUpdateRequired = true;
            }
            mtxDatabase.unlock();
        }

        /**
         * @brief Get the current output of the asynchronous online planner.
         * @return The output of the asynchronous online planner.
         */
        mpsv::planner::AsyncOnlinePlannerOutput GetOutput(void){
            mpsv::planner::AsyncOnlinePlannerOutput result;
            mtxDatabase.lock();
            result = database.output;
            result.threadState = workerThread.GetState();
            if(result.threadState != mpsv::core::WorkerThreadState::running){
                result.timestampInput = database.input.timestamp;
                result.timestampParameter = database.parameter.timestamp;
                result.inputError = database.inputError;
                result.parameterError = database.parameterError;
            }
            mtxDatabase.unlock();
            return result;
        }


    protected:
        /**
         * @brief This function is called from inside the worker thread of the planner, if a planning problem has been solved and new
         * output data is available. The actual data is obtained via @ref GetOutput.
         */
        virtual void CallbackNewOutputAvailable(void){}


    private:
        mpsv::core::WorkerThread workerThread;        // The internal worker thread.
        mpsv::planner::OnlinePlanner onlinePlanner;   // The online sequential planner.
        std::atomic<bool> terminate;                  // True if the asynchronous online planner should be terminated, false otherwise.
        volatile double previousParameterTimestamp;   // Previous parameter timestamp (used to detect new parameter sets).

        /* internal database */
        class Database {
            public:
                mpsv::planner::AsyncOnlinePlannerInput input;              // The latest valid input data.
                mpsv::planner::AsyncOnlinePlannerParameterSet parameter;   // The latest valid input parameter set.
                mpsv::planner::AsyncOnlinePlannerOutput output;            // Output of the online planner.
                mpsv::core::PerformanceCounter timeoutCounter;             // Performance counter to measure timeouts.
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
        } database;
        std::mutex mtxDatabase;   // Protect @ref database.


        /**
         * @brief The callback function that is executed if the next asynchronous step calculation is performed by the worker thread.
         */
        void WorkerThreadRun(void) noexcept {
            // do nothing if thread should be terminated
            if(terminate){
                return;
            }

            // storage for current input and output data
            mpsv::planner::AsyncOnlinePlannerInput dataIn;
            mpsv::planner::AsyncOnlinePlannerOutput dataOut;

            // get input, apply parameters, handle errors
            mtxDatabase.lock();
            if(database.timeoutCounter.TimeToStart() >= database.parameter.timeoutInput){
                database.inputError = (database.inputError == error_code::NONE) ? error_code::NOT_AVAILABLE : database.inputError;
            }
            if(database.parameterUpdateRequired){
                database.parameterError = onlinePlanner.ApplyParameterSet(database.parameter);
                database.parameterUpdateRequired = false;
            }
            bool suspend = (error_code::NONE != database.inputError) || (error_code::NONE != database.parameterError) || !database.input.enable;
            bool reset = database.resetIsRequired;
            database.resetIsRequired = false;
            dataIn = database.input;
            dataOut.timestampInput = database.input.timestamp;
            dataOut.timestampParameter = database.parameter.timestamp;
            dataOut.inputError = database.inputError;
            dataOut.parameterError = database.parameterError;
            mtxDatabase.unlock();

            // perform a reset if commanded and solve the planning problem
            if(!suspend){
                if(reset){
                    onlinePlanner.Reset();
                }
                onlinePlanner.Solve(dataOut, dataIn);
            }

            // move result to internal database and run callback
            mtxDatabase.lock();
            if(dataOut.error || suspend){
                database.resetIsRequired = true; // reset is required when leaving the standby mode
                workerThread.Suspend();
            }
            dataOut.MoveTo(database.output);
            mtxDatabase.unlock();
            CallbackNewOutputAvailable();
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

