#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/core/DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the parameter set for the path planning algorithm.
 */
class PathPlannerParameterSet {
    public:
        mpsv::planner::ParameterGeometry geometry;         // Parameters for geometric properties.
        mpsv::planner::ParameterCostMap costMap;           // Parameters for the cost map (2D look-up table of additional cost values).
        mpsv::planner::ParameterMetric metric;             // Parameters for the distance metric function.
        mpsv::planner::ParameterPathPlanner pathPlanner;   // Parameters for the RRT* path planning algorithm.

        /**
         * @brief Construct a new path planner parameter object and set default values.
         */
        PathPlannerParameterSet() noexcept { Clear(); }

        /**
         * @brief Clear the path planner parameter object and set default values.
         */
        void Clear(void) noexcept {
            geometry.Clear();
            costMap.Clear();
            metric.Clear();
            pathPlanner.Clear();
        }

        /**
         * @brief Check whether this parameter set is valid or not.
         * @return True if parameter set is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validGeometry = geometry.IsValid();
            bool validCostMap = costMap.IsValid();
            bool validMetric = metric.IsValid();
            bool validPathPlanner = pathPlanner.IsValid();
            return validGeometry && validCostMap && validMetric && validPathPlanner;
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
            pathPlanner.WriteToFile(file, preString + "pathPlanner.");
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

