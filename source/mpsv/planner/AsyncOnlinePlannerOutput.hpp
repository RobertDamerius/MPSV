#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/OnlinePlannerOutput.hpp>
#include <mpsv/core/WorkerThread.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output to the asynchronous online planning algorithm (online sequential planner).
 */
class AsyncOnlinePlannerOutput: public mpsv::planner::OnlinePlannerOutput {
    public:
        double timestampInput;                       // The user-defined timestamp of the corresponding input data that has been used to compute the solution. The default value is quiet_NaN.
        double timestampParameter;                   // The user-defined timestamp of the corresponding parameter data that has been used to compute the solution. The default value is quiet_NaN.
        bool timeoutInput;                           // True if the given input data timed out, false otherwise.
        bool validInput;                             // True if the given input data is valid, false otherwise.
        bool validParameter;                         // True if the given parameter data is valid, false otherwise.
        mpsv::core::WorkerThreadState threadState;   // The state of the planning thread.

        /**
         * @brief Construct a new output object for the asynchronous online planner and set default values.
         */
        AsyncOnlinePlannerOutput() noexcept { Clear(); }

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            mpsv::planner::OnlinePlannerOutput::Clear();
            timestampInput = std::numeric_limits<double>::quiet_NaN();
            timestampParameter = std::numeric_limits<double>::quiet_NaN();
            timeoutInput = false;
            validInput = false;
            validParameter = false;
            threadState = mpsv::core::WorkerThreadState::offline;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::AsyncOnlinePlannerOutput& dst) noexcept {
            mpsv::planner::OnlinePlannerOutput::MoveTo(dst);
            dst.timestampInput = timestampInput;
            dst.timestampParameter = timestampParameter;
            dst.timeoutInput = timeoutInput;
            dst.validInput = validInput;
            dst.validParameter = validParameter;
            dst.threadState = threadState;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

