#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/PathPlannerOutput.hpp>
#include <mpsv/planner/MotionPlannerOutput.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output of the sequential planning algorithm.
 */
class SequentialPlannerOutput {
    public:
        mpsv::error_code errorCode{mpsv::error_code::NONE};   // Path planner error if present, otherwise the motion planner error.
        mpsv::planner::PathPlannerOutput pathPlanner;         // Output of the internal path planner.
        mpsv::planner::MotionPlannerOutput motionPlanner;     // Output of the internal motion planner.

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            errorCode = mpsv::error_code::NONE;
            pathPlanner.Clear();
            motionPlanner.Clear();
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("uint8_t", preString + "errorCode", {1}, &errorCode, sizeof(errorCode));
            pathPlanner.WriteToFile(file, preString + "pathPlanner.");
            motionPlanner.WriteToFile(file, preString + "motionPlanner.");
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
