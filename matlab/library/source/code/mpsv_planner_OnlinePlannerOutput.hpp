#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the output of the online planning algorithm (online sequential planner).
 */
class OnlinePlannerOutput {
    public:
        double timestamp;                                      // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint of the @ref trajectory.
        std::array<double,3> originLLA;                        // Geographical origin to which this outputs belongs to, given as {lat,lon,alt}.
        std::vector<std::array<double,12>> trajectory;         // Resulting trajectory, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
        double sampletime;                                     // The sampletime of the trajectory data in seconds.
        bool performedReset;                                   // True if reset has been performed, false otherwise.
        bool error;                                            // True if path or motion planner reported an error, e.g. not feasible or out of nodes, false otherwise. This value is equal to (!pathPlanner.isFeasible || pathPlanner.outOfNodes || !motionPlanner.isFeasible || motionPlanner.outOfNodes).
        struct {
            std::vector<std::array<double,3>> path;            // [PathPlanner] Resulting path of the internal path planning problem, where each pose is given as {x,y,psi}.
            bool goalReached;                                  // [PathPlanner] True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            bool isFeasible;                                   // [PathPlanner] True if problem is feasible, false otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            bool outOfNodes;                                   // [PathPlanner] True if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            uint32_t numberOfPerformedIterations;              // [PathPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                  // [PathPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                       // [PathPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } pathPlanner;
        struct {
            std::vector<std::array<double,3>> referencePath;   // [MotionPlanner] Resulting reference path of the internal motion planning problem, where each pose is given as {x,y,psi}.
            std::vector<std::array<double,12>> trajectory;     // [MotionPlanner] Resulting trajectory of the internal motion planning problem, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
            double startingTimepoint;                          // [MotionPlanner] The starting timepoint of the internal motion planner trajectory (arbitrary time origin defined by the user, same as @ref timestamp).
            bool goalReached;                                  // [MotionPlanner] True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            bool isFeasible;                                   // [MotionPlanner] True if problem is feasible, false otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            bool outOfNodes;                                   // [MotionPlanner] True if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            bool usedSmoothedReferencePath;                    // [MotionPlanner] True if @ref referencePath has been smoothed for final trajectory generation, false otherwise.
            uint32_t numberOfPerformedIterations;              // [MotionPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                  // [MotionPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                       // [MotionPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } motionPlanner;

        /**
         * @brief Construct a new online planner output object and set default values.
         */
        OnlinePlannerOutput() noexcept { Clear(); }

        /**
         * @brief Clear the output and set default values.
         */
        void Clear(void) noexcept {
            timestamp = 0.0;
            originLLA.fill(0.0);
            trajectory.clear();
            sampletime = 0.0;
            performedReset = false;
            error = true;
            pathPlanner.path.clear();
            pathPlanner.goalReached = pathPlanner.isFeasible = pathPlanner.outOfNodes = false;
            pathPlanner.numberOfPerformedIterations = 0;
            pathPlanner.timestampOfComputationUTC = pathPlanner.cost = 0.0;
            motionPlanner.referencePath.clear();
            motionPlanner.trajectory.clear();
            motionPlanner.goalReached = motionPlanner.isFeasible = motionPlanner.outOfNodes = motionPlanner.usedSmoothedReferencePath = false;
            motionPlanner.numberOfPerformedIterations = 0;
            motionPlanner.startingTimepoint = motionPlanner.timestampOfComputationUTC = motionPlanner.cost = 0.0;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::OnlinePlannerOutput& dst) noexcept {
            dst.timestamp = timestamp;
            dst.originLLA = originLLA;
            dst.trajectory.swap(trajectory);
            dst.sampletime = sampletime;
            dst.performedReset = performedReset;
            dst.error = error;
            dst.pathPlanner.path.swap(pathPlanner.path);
            dst.pathPlanner.goalReached = pathPlanner.goalReached;
            dst.pathPlanner.isFeasible = pathPlanner.isFeasible;
            dst.pathPlanner.outOfNodes = pathPlanner.outOfNodes;
            dst.pathPlanner.numberOfPerformedIterations = pathPlanner.numberOfPerformedIterations;
            dst.pathPlanner.timestampOfComputationUTC = pathPlanner.timestampOfComputationUTC;
            dst.pathPlanner.cost = pathPlanner.cost;
            dst.motionPlanner.referencePath.swap(motionPlanner.referencePath);
            dst.motionPlanner.trajectory.swap(motionPlanner.trajectory);
            dst.motionPlanner.startingTimepoint = motionPlanner.startingTimepoint;
            dst.motionPlanner.goalReached = motionPlanner.goalReached;
            dst.motionPlanner.isFeasible = motionPlanner.isFeasible;
            dst.motionPlanner.outOfNodes = motionPlanner.outOfNodes;
            dst.motionPlanner.usedSmoothedReferencePath = motionPlanner.usedSmoothedReferencePath;
            dst.motionPlanner.numberOfPerformedIterations = motionPlanner.numberOfPerformedIterations;
            dst.motionPlanner.timestampOfComputationUTC = motionPlanner.timestampOfComputationUTC;
            dst.motionPlanner.cost = motionPlanner.cost;
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "timestamp", {1}, &timestamp, sizeof(timestamp));
            file.WriteField("double", preString + "originLLA", {3}, &originLLA[0], sizeof(originLLA));
            file.WriteVectorField(preString + "trajectory", trajectory);
            file.WriteField("double", preString + "sampletime", {1}, &sampletime, sizeof(sampletime));
            file.WriteField("bool", preString + "performedReset", {1}, &performedReset, sizeof(performedReset));
            file.WriteField("bool", preString + "error", {1}, &error, sizeof(error));
            file.WriteVectorField(preString + "pathPlanner.path", pathPlanner.path);
            file.WriteField("bool", preString + "pathPlanner.goalReached", {1}, &pathPlanner.goalReached, sizeof(pathPlanner.goalReached));
            file.WriteField("bool", preString + "pathPlanner.isFeasible", {1}, &pathPlanner.isFeasible, sizeof(pathPlanner.isFeasible));
            file.WriteField("bool", preString + "pathPlanner.outOfNodes", {1}, &pathPlanner.outOfNodes, sizeof(pathPlanner.outOfNodes));
            file.WriteField("uint32_t", preString + "pathPlanner.numberOfPerformedIterations", {1}, &pathPlanner.numberOfPerformedIterations, sizeof(pathPlanner.numberOfPerformedIterations));
            file.WriteField("double", preString + "pathPlanner.timestampOfComputationUTC", {1}, &pathPlanner.timestampOfComputationUTC, sizeof(pathPlanner.timestampOfComputationUTC));
            file.WriteField("double", preString + "pathPlanner.cost", {1}, &pathPlanner.cost, sizeof(pathPlanner.cost));
            file.WriteVectorField(preString + "motionPlanner.referencePath", motionPlanner.referencePath);
            file.WriteVectorField(preString + "motionPlanner.trajectory", motionPlanner.trajectory);
            file.WriteField("double", preString + "motionPlanner.startingTimepoint", {1}, &motionPlanner.startingTimepoint, sizeof(motionPlanner.startingTimepoint));
            file.WriteField("bool", preString + "motionPlanner.goalReached", {1}, &motionPlanner.goalReached, sizeof(motionPlanner.goalReached));
            file.WriteField("bool", preString + "motionPlanner.isFeasible", {1}, &motionPlanner.isFeasible, sizeof(motionPlanner.isFeasible));
            file.WriteField("bool", preString + "motionPlanner.outOfNodes", {1}, &motionPlanner.outOfNodes, sizeof(motionPlanner.outOfNodes));
            file.WriteField("bool", preString + "motionPlanner.usedSmoothedReferencePath", {1}, &motionPlanner.usedSmoothedReferencePath, sizeof(motionPlanner.usedSmoothedReferencePath));
            file.WriteField("uint32_t", preString + "motionPlanner.numberOfPerformedIterations", {1}, &motionPlanner.numberOfPerformedIterations, sizeof(motionPlanner.numberOfPerformedIterations));
            file.WriteField("double", preString + "motionPlanner.timestampOfComputationUTC", {1}, &motionPlanner.timestampOfComputationUTC, sizeof(motionPlanner.timestampOfComputationUTC));
            file.WriteField("double", preString + "motionPlanner.cost", {1}, &motionPlanner.cost, sizeof(motionPlanner.cost));
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

