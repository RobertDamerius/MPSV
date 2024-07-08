#pragma once


#include <mpsv_core_MPSVCommon.hpp>


namespace mpsv {


namespace planner {


class PathPlannerState {
    public:
        int16_t idxSolutionNode;                 // Index to the solution node. The solution node is a node inside the goal region or the node closest to the goal region and is defined to be at least the root node.
        bool isFeasible;                         // True if problem is actually feasible, false otherwise. The problem is feasible, if the initial pose does not collide with static obstacles or if the initial or final pose is not inside the sampling area.
        bool goalReached;                        // True if solution was found and the goal pose is reached from the initial pose.
        bool outOfNodes;                         // True if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
        double closestDistanceToGoal;            // The closest distance value to the goal.
        std::array<double,3> initialPose;        // Initial pose of path planning problem, given as {x, y, psi}.
        std::array<double,3> finalPose;          // Final pose of path planning problem, given as {x, y, psi}.
        uint32_t sampleIteration;                // Iteration variable that is incremented in each non-goal-sampling step and is at most maxSampleIteration. This value is used to get a new random sample with a specific index.
        uint32_t goalSampleIteration;            // Iteration variable that is incremented in each goal-sampling step and is at most periodGoalSampling. This value is used to get a new random sample with a specific index.
        uint32_t numberOfPerformedIterations;    // Total number of iterations that have been performed since the latest prepare step.

        /**
         * @brief Construct a new state object for the path planner and set default values.
         */
        PathPlannerState() noexcept { Clear(); }

        /**
         * @brief Clear the state and set default values.
         * @param[in] keepSolution True if solution should be kept (@ref idxSolutionNode and @ref closestDistanceToGoal remain unchanged), false if everything is to be cleared. The default value is false.
         */
        void Clear(bool keepSolution = false) noexcept {
            if(!keepSolution){
                idxSolutionNode = 0;
                closestDistanceToGoal = std::numeric_limits<double>::infinity();
                sampleIteration = 0;
            }
            isFeasible = false;
            goalReached = false;
            outOfNodes = false;
            initialPose.fill(0.0);
            finalPose.fill(0.0);
            goalSampleIteration = 0;
            numberOfPerformedIterations = 0;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

