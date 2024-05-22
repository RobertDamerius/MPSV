#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_planner_ParameterTypes.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the parameter set for the motion planning algorithm.
 */
class MotionPlannerParameterSet {
    public:
        mpsv::planner::ParameterGeometry geometry;             // Parameters for geometric properties.
        mpsv::planner::ParameterCostMap costMap;               // Parameters for the cost map (2D look-up table of additional cost values).
        mpsv::planner::ParameterMetric metric;                 // Parameters for the distance metric function.
        mpsv::planner::ParameterModel model;                   // Parameters for the nonlinear dynamic motion model.
        mpsv::planner::ParameterMotionPlanner motionPlanner;   // Parameters for the RRT* motion planning algorithm.

        /**
         * @brief Construct a new motion planner parameter object and set default values.
         */
        MotionPlannerParameterSet() noexcept { Clear(); }

        /**
         * @brief Clear the motion planner parameter object and set default values.
         */
        void Clear(void) noexcept {
            geometry.Clear();
            costMap.Clear();
            metric.Clear();
            model.Clear();
            motionPlanner.Clear();
        }

        /**
         * @brief Check whether this parameter set is valid or not.
         * @return True if parameter set is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validGeometry = geometry.IsValid();
            bool validCostMap = costMap.IsValid();
            bool validMetric = metric.IsValid();
            bool validModel = model.IsValid();
            bool validMotionPlanner = motionPlanner.IsValid();
            return validGeometry && validCostMap && validMetric && validModel && validMotionPlanner;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            geometry.WriteToFile(file, preString + "geometry.");
            costMap.WriteToFile(file, preString + "costMap.");
            metric.WriteToFile(file, preString + "metric.");
            model.WriteToFile(file, preString + "model.");
            motionPlanner.WriteToFile(file, preString + "motionPlanner.");
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

