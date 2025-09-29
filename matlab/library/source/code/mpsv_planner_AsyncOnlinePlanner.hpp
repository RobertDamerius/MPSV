#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_WorkerThread.hpp>
#include <mpsv_core_PerformanceCounter.hpp>
#include <mpsv_planner_OnlinePlanner.hpp>
#include <mpsv_planner_AsyncOnlinePlannerInput.hpp>
#include <mpsv_planner_AsyncOnlinePlannerOutput.hpp>
#include <mpsv_planner_AsyncOnlinePlannerParameterSet.hpp>


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
            bool valid = input.IsValid();
            bool suspendWorkerThread = !input.enable;

            // move valid input to internal database
            mtxDatabase.lock();
            if(valid){
                if(input.reset){ // detected reset: interrupt planner
                    database.resetIsRequired = true;
                    onlinePlanner.Interrupt();
                }
                input.MoveTo(database.input);
                database.timeoutCounter.Start();
            }
            if(suspendWorkerThread){
                database.resetIsRequired = true; // reset is required when leaving the standby mode
            }
            if(mpsv::core::WorkerThreadState::running != workerThread.GetState()){ // update some outputs if thread is not running
                database.output.validInput = valid;
                if(valid){
                    database.output.timestampInput = database.input.timestamp;
                }
            }
            database.validInput = valid;
            mtxDatabase.unlock();

            // suspend or resume worker thread
            if(suspendWorkerThread){
                workerThread.Suspend();
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
            bool valid = parameter.IsValid();

            // move valid parameters to internal database
            mtxDatabase.lock();
            if(valid){
                parameter.MoveTo(database.parameter);
            }
            database.validParameter = valid;
            if(mpsv::core::WorkerThreadState::running != workerThread.GetState()){ // update some outputs if thread is not running
                database.output.validParameter = valid;
                if(valid){
                    database.output.timestampParameter = database.parameter.timestamp;
                }
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
            mtxDatabase.unlock();
            result.threadState = workerThread.GetState();
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

        /* internal database */
        class Database {
            public:
                mpsv::planner::AsyncOnlinePlannerInput input;              // The latest valid input data.
                mpsv::planner::AsyncOnlinePlannerParameterSet parameter;   // The latest valid input parameter set.
                mpsv::planner::AsyncOnlinePlannerOutput output;            // Output of the planner.
                mpsv::core::PerformanceCounter timeoutCounter;             // Performance counter to measure timeouts.
                bool validInput;                                           // True if @ref input is valid, false otherwise.
                bool validParameter;                                       // True if @ref parameter is valid, false otherwise.
                bool resetIsRequired;                                      // True if an automatic reset is required or a reset has been commanded via @ref SetInput, false otherwise.
                double previousParameterTimestamp;                         // The previous timestamp of a parameter set. The initial value is quiet_NaN.

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
                    validInput = false;
                    validParameter = false;
                    resetIsRequired = true;
                    previousParameterTimestamp = std::numeric_limits<double>::quiet_NaN();
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

            // handle timeout and invalid input data: suspend worker thread in case of errors
            mtxDatabase.lock();
            bool timeout = (database.timeoutCounter.TimeToStart() >= database.parameter.timeoutInput);
            dataOut.timestampInput = database.input.timestamp;
            dataOut.timestampParameter = database.parameter.timestamp;
            dataOut.timeoutInput = timeout;
            dataOut.validInput = database.validInput;
            dataOut.validParameter = database.validParameter;
            if(timeout || !database.validInput || !database.validParameter || !database.input.enable){
                dataOut.MoveTo(database.output);
                database.resetIsRequired = true; // reset is required when leaving the standby mode
                mtxDatabase.unlock();
                workerThread.Suspend();
                return;
            }
            if(!std::isfinite(database.previousParameterTimestamp) || (database.previousParameterTimestamp != database.parameter.timestamp)){
                database.previousParameterTimestamp = database.parameter.timestamp;
                dataOut.validParameter = onlinePlanner.ApplyParameterSet(database.parameter);
                if(!dataOut.validParameter){
                    dataOut.MoveTo(database.output);
                    database.resetIsRequired = true; // reset is required when leaving the standby mode
                    mtxDatabase.unlock();
                    workerThread.Suspend();
                    return;
                }
            }
            bool reset = database.resetIsRequired;
            database.resetIsRequired = false;
            dataIn = database.input;
            mtxDatabase.unlock();

            // perform a reset if commanded
            if(reset){
                onlinePlanner.Reset();
            }

            // solve the planning problem
            onlinePlanner.Solve(dataOut, dataIn);

            // move result to internal database and run callback
            mtxDatabase.lock();
            if(dataOut.error){
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

