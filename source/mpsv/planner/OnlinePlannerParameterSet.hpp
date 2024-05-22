#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/SequentialPlannerParameterSet.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/core/DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the parameter set for the online planning algorithm (online sequential planner).
 */
class OnlinePlannerParameterSet {
    public:
        mpsv::planner::SequentialPlannerParameterSet sequentialPlanner;   // Parameters for the sequential planner.
        mpsv::planner::ParameterOnlinePlanner onlinePlanner;              // Parameters for the online planner.

        /**
         * @brief Construct a new online planner parameter object and set default values.
         */
        OnlinePlannerParameterSet() noexcept { Clear(); }

        /**
         * @brief Clear the sequential planner parameter object and set default values.
         */
        void Clear(void) noexcept {
            sequentialPlanner.Clear();
            onlinePlanner.Clear();
        }

        /**
         * @brief Check whether this parameter set is valid or not.
         * @return True if parameter set is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            return sequentialPlanner.IsValid() && onlinePlanner.IsValid();
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::OnlinePlannerParameterSet& dst) noexcept {
            dst.sequentialPlanner.costMap = sequentialPlanner.costMap;
            dst.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation = sequentialPlanner.geometry.collisionCheckMaxPositionDeviation;
            dst.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation = sequentialPlanner.geometry.collisionCheckMaxAngleDeviation;
            dst.sequentialPlanner.geometry.skeletalPoints.swap(sequentialPlanner.geometry.skeletalPoints);
            dst.sequentialPlanner.geometry.vehicleShape.convexVehicleShapes.swap(sequentialPlanner.geometry.vehicleShape.convexVehicleShapes);
            dst.sequentialPlanner.metric = sequentialPlanner.metric;
            dst.sequentialPlanner.model = sequentialPlanner.model;
            dst.sequentialPlanner.motionPlanner = sequentialPlanner.motionPlanner;
            dst.sequentialPlanner.pathPlanner = sequentialPlanner.pathPlanner;
            dst.onlinePlanner.predictInitialStateOnReset = onlinePlanner.predictInitialStateOnReset;
            dst.onlinePlanner.maxComputationTimePathOnReset = onlinePlanner.maxComputationTimePathOnReset;
            dst.onlinePlanner.maxComputationTimeMotionOnReset = onlinePlanner.maxComputationTimeMotionOnReset;
            dst.onlinePlanner.maxComputationTimePath = onlinePlanner.maxComputationTimePath;
            dst.onlinePlanner.maxComputationTimeMotion = onlinePlanner.maxComputationTimeMotion;
            dst.onlinePlanner.additionalAheadPlanningTime = onlinePlanner.additionalAheadPlanningTime;
            dst.onlinePlanner.additionalTrajectoryDuration = onlinePlanner.additionalTrajectoryDuration;
            dst.onlinePlanner.timeKeepPastTrajectory = onlinePlanner.timeKeepPastTrajectory;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            sequentialPlanner.WriteToFile(file, preString + "sequentialPlanner.");
            onlinePlanner.WriteToFile(file, preString + "onlinePlanner.");
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

