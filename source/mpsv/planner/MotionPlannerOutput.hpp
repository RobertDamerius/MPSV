#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output of the motion planning algorithm.
 */
class MotionPlannerOutput {
    public:
        bool goalReached;                                  // True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
        bool isFeasible;                                   // True if problem is feasible, false otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
        bool outOfNodes;                                   // True if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
        uint32_t numberOfPerformedIterations;              // The total number of iterations that have been performed since the latest prepare step.
        double timestampOfComputationUTC;                  // Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
        double cost;                                       // The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        std::vector<std::array<double,3>> referencePath;   // Reference path that has been used as basis for trajectory generation. Each pose is given as {x,y,psi}. The first element always corresponds to the initial pose. If the goal is reached, then the final pose corresponds to the desired final pose of the planning problem.
        std::vector<std::array<double,12>> trajectory;     // Resulting trajectory, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
        double sampletime;                                 // The sampletime of the trajectory in seconds.

        /**
         * @brief Construct a new motion planner output object and set default values.
         */
        MotionPlannerOutput() noexcept { Clear(); }

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            goalReached = isFeasible = outOfNodes = false;
            timestampOfComputationUTC = cost = 0.0;
            referencePath.clear();
            trajectory.clear();
            sampletime = 0.0;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::MotionPlannerOutput& dst) noexcept {
            dst.goalReached = goalReached;
            dst.isFeasible = isFeasible;
            dst.outOfNodes = outOfNodes;
            dst.numberOfPerformedIterations = numberOfPerformedIterations;
            dst.timestampOfComputationUTC = timestampOfComputationUTC;
            dst.cost = cost;
            dst.referencePath.swap(referencePath);
            dst.trajectory.swap(trajectory);
            dst.sampletime = sampletime;
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("bool", preString + "goalReached", {1}, &goalReached, sizeof(goalReached));
            file.WriteField("bool", preString + "isFeasible", {1}, &isFeasible, sizeof(isFeasible));
            file.WriteField("bool", preString + "outOfNodes", {1}, &outOfNodes, sizeof(outOfNodes));
            file.WriteField("uint32_t", preString + "numberOfPerformedIterations", {1}, &numberOfPerformedIterations, sizeof(numberOfPerformedIterations));
            file.WriteField("double", preString + "timestampOfComputationUTC", {1}, &timestampOfComputationUTC, sizeof(timestampOfComputationUTC));
            file.WriteField("double", preString + "cost", {1}, &cost, sizeof(cost));
            file.WriteVectorField(preString + "referencePath", referencePath);
            file.WriteVectorField(preString + "trajectory", trajectory);
            file.WriteField("double", preString + "sampletime", {1}, &sampletime, sizeof(sampletime));
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

