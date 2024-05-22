#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_planner_PathPlannerOutput.hpp>
#include <mpsv_planner_MotionPlannerOutput.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output of the sequential planning algorithm.
 */
class SequentialPlannerOutput {
    public:
        mpsv::planner::PathPlannerOutput pathPlanner;       // Output of the internal path planner.
        mpsv::planner::MotionPlannerOutput motionPlanner;   // Output of the internal motion planner.

        /**
         * @brief Construct a new sequential planner output object and set default values.
         */
        SequentialPlannerOutput() noexcept { Clear(); }

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            pathPlanner.Clear();
            motionPlanner.Clear();
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            pathPlanner.WriteToFile(file, preString + "pathPlanner.");
            motionPlanner.WriteToFile(file, preString + "motionPlanner.");
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

