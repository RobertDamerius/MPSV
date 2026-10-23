#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output of the path planning algorithm.
 */
class PathPlannerOutput {
    public:
        bool goalReached{false};                              // True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
        mpsv::error_code errorCode{mpsv::error_code::NONE};   // Error code indicating the status of the path planning operation.
        uint32_t numberOfPerformedIterations{0};              // The total number of iterations that have been performed since the latest prepare step.
        double timestampOfComputationUTC{0.0};                // Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
        double cost{0.0};                                     // The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        std::vector<std::array<double,3>> path;               // Resulting path, where each pose is given as {x,y,psi}. The first element always corresponds to the initial pose. If the goal is reached, then the final pose corresponds to the desired final pose of the path planning problem.

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            goalReached = false;
            errorCode = mpsv::error_code::NONE;
            numberOfPerformedIterations = 0;
            timestampOfComputationUTC = cost = 0.0;
            path.clear();
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[inout] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::PathPlannerOutput& dst) noexcept {
            dst.goalReached = goalReached;
            dst.errorCode = errorCode;
            dst.numberOfPerformedIterations = numberOfPerformedIterations;
            dst.timestampOfComputationUTC = timestampOfComputationUTC;
            dst.cost = cost;
            dst.path.swap(path);
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("bool", preString + "goalReached", {1}, &goalReached, sizeof(goalReached));
            file.WriteField("bool", preString + "errorCode", {1}, &errorCode, sizeof(errorCode));
            file.WriteField("uint32_t", preString + "numberOfPerformedIterations", {1}, &numberOfPerformedIterations, sizeof(numberOfPerformedIterations));
            file.WriteField("double", preString + "timestampOfComputationUTC", {1}, &timestampOfComputationUTC, sizeof(timestampOfComputationUTC));
            file.WriteField("double", preString + "cost", {1}, &cost, sizeof(cost));
            file.WriteVectorField(preString + "path", path);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
