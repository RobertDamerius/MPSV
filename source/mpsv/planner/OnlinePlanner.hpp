#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/SequentialPlanner.hpp>
#include <mpsv/planner/SequentialPlannerOutput.hpp>
#include <mpsv/planner/SequentialPlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>
#include <mpsv/planner/OnlinePlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerOutput.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/Time.hpp>
#include <mpsv/core/PerformanceCounter.hpp>
#include <mpsv/math/WGS84.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class implements the online planner that continuously solves sequential planning problems (path + motion planning).
 * It solves motion planning problems for a future timepoint and merges the solution trajectory seamless with the previous solution.
 */
class OnlinePlanner {
    public:
        /**
         * @brief Construct a new online planner object and set default values.
         */
        OnlinePlanner() noexcept { Terminate(); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initiallization / Termination
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Initialize the online planner.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @return True if success, false otherwise.
         * @details Call this function before calling any of the other member functions
         */
        bool Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples) noexcept { return sequentialPlanner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples); }

        /**
         * @brief Terminate the online planner. This calls the @ref Reset member function.
         */
        void Terminate(void) noexcept {
            sequentialPlanner.Terminate();
            parameter.Clear();
            state.Clear();
            Reset();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Algorithm API: ApplyParameterSet, Reset and Solve
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Apply a new parameter set to the internal solvers.
         * @param[in] parameter The parameter set to be applied.
         * @return True if parameter set is valid and has been applied, false otherwise.
         * @details If a new parameter set has been applied successfully, @ref Reset is called internally.
         */
        bool ApplyParameterSet(const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            if(!parameter.onlinePlanner.IsValid() || !sequentialPlanner.ApplyParameterSet(parameter.sequentialPlanner)){
                return false;
            }
            if(static_cast<uint32_t>(std::floor((parameter.onlinePlanner.maxComputationTimeMotionOnReset + parameter.onlinePlanner.maxComputationTimePathOnReset + parameter.onlinePlanner.additionalAheadPlanningTime) / parameter.sequentialPlanner.motionPlanner.sampletime)) >= 0x0000FFFF){
                return false;
            }
            if(static_cast<uint32_t>(std::floor((parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.additionalAheadPlanningTime) / parameter.sequentialPlanner.motionPlanner.sampletime)) >= 0x0000FFFF){
                return false;
            }
            this->parameter = parameter.onlinePlanner;
            Reset();
            return true;
        }

        /**
         * @brief Reset the online planner to make it start from the initial state during the next solve operation.
         * @details The parameters are not cleared and the interrupt flags are not changed.
         */
        void Reset(void) noexcept { reset = true; }

        /**
         * @brief Solve the online sequential motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). The timestamp must be monotonically increasing.
         * @details After a @ref Reset call this planner solves the motion planning based on the initial state by optionally predicting the initial state by the expected
         * computation time. A solution trajectory is saved to the internal @ref state of this planner. The next solve operation selects the initial state for the
         * planning problem on a future timepoint of the latest solution trajectory. If this is not possible, a reset is done internally and the planner falls back using
         * the initial state given in the input data.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::OnlinePlannerOutput& dataOut, const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            if(reset){
                SolveInitialProblem(dataIn);
            }
            else{
                SolveNextProblem(dataIn);
            }
            dataOut = state;
        }

        /**
         * @brief Interrupt the @ref Solve operation of the online planner by interrupting the internal sequential planner.
         * If no @ref Solve operation is running, then the next @ref Solve operation is to be interrupted.
         */
        void Interrupt(void) noexcept {
            sequentialPlanner.Interrupt();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Additional
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Call this function directly after the @ref Solve function to further improve the path.
         * @param[in] maxComputationTime The maximum computation time in seconds before stopping the solve iteration loop.
         * @details The path improvement can be interrupted by calling the @ref Interrupt member function.
         */
        void ImprovePath(double maxComputationTime) noexcept { sequentialPlanner.ImprovePath(maxComputationTime); }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept { return sequentialPlanner.PredictMotion(initialStateAndInput, predictionHorizon); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Data Recording / Debugging
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration. All trajectories of the tree are rebuilt. Otherwise the trajectories from tree nodes are undefined.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept { sequentialPlanner.WriteToFile(file, preString + "sequentialPlanner.", staticObstacles); }


    protected:
        mpsv::planner::OnlinePlannerOutput state;                         // Internal state of the online planner representing the latest output.
        bool reset;                                                       // True if the online planner has been reset.
        mpsv::planner::SequentialPlanner sequentialPlanner;               // Internal sequential planning solver.
        mpsv::planner::SequentialPlannerInput sequentialPlannerInput;     // Input to the @ref sequentialPlanner.
        mpsv::planner::SequentialPlannerOutput sequentialPlannerOutput;   // Output of the @ref sequentialPlanner.
        mpsv::planner::ParameterOnlinePlanner parameter;                  // Parameter for the online planner.
        mpsv::core::PerformanceCounter performanceCounter;                // Performance counter used to measure execution time.

        /**
         * @brief Get the state/input vector from the latest solution trajectory (@ref state) for a given absolute timestamp.
         * @param[in] timestamp The timestamp (time origin defined by user) for which to obtain the state/input vector.
         * @param[in] offsetOldToNew If the origin of the current trajectory (state.trajectory) is different to the origin of the new problem, then this coordinate translation must be taken into account. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
         * @return Tuple containing index, resulting timestamp and the actual state/input.
         * @return [0] Index indicating the state/vector that was chosen from state.trajectory. If the given timestamp is out of range, -1 is returned.
         * @return [1] The actual timestamp that corresponds to that index.
         * @return [2] The state/input vector for that index.
         */
        std::tuple<int32_t,double,std::array<double,12>> GetTrajectoryStateAtTimestamp(double timestamp, std::array<double,2> offsetOldToNew) noexcept {
            // Return values
            std::tuple<int32_t,double,std::array<double,12>> result;
            int32_t& retIndex = std::get<0>(result);
            double& retTimestamp = std::get<1>(result);
            std::array<double,12>& retInitialStateAndInput = std::get<2>(result);

            // Calculate the time offset from the old starting point (state.timestamp) to the new starting point
            double timeFromOldStart = timestamp - state.timestamp;

            // Convert that time offset to an index and check for a valid range
            double dSteps = timeFromOldStart / state.sampletime;
            retIndex = static_cast<int32_t>(std::floor(dSteps));
            if((retIndex < 0) || (retIndex >= static_cast<int32_t>(state.trajectory.size()))){
                // Requested timestamp is not within range of latest trajectory
                retIndex = -1;
            }
            else{
                // Update new starting point (time) to lie exactly on the grid of the old trajectory (at the returned index)
                retTimestamp = state.timestamp + (state.sampletime * static_cast<double>(retIndex));
                retInitialStateAndInput = state.trajectory[retIndex];
                retInitialStateAndInput[0] -= offsetOldToNew[0];
                retInitialStateAndInput[1] -= offsetOldToNew[1];
            }
            return result;
        }

        /**
         * @brief Callback function to check whether to change to a new trajectory or not.
         * @return At the moment this function always returns true.
         */
        bool ChangeToNewTrajectory(double timestampOld, const std::vector<std::array<double,12>>& trajectoryOld, double timestampNew, const std::vector<std::array<double,12>>& trajectoryNew, std::array<double,2> offsetOldToNew, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles){
            (void)timestampOld;
            (void)trajectoryOld;
            (void)timestampNew;
            (void)trajectoryNew;
            (void)offsetOldToNew;
            (void)staticObstacles;
            return true; // for now: always change to new trajectory
        }

        /**
         * @brief Solve the first online sequential planning problem using the initial state given in the input and optionally predicting it by the expected computation time.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles).
         */
        void SolveInitialProblem(const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            // Timestamp and initial state/input
            performanceCounter.Start();
            double timestamp = dataIn.timestamp;
            std::array<double,12> initialStateAndInput = dataIn.initialStateAndInput;

            // Get the offset from the old origin (state) to the new origin (dataIn)
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, state.originLLA);

            // Change the timestamp and the initial state vector by predicting the motion for the expected computation time
            std::vector<std::array<double,12>> predictionTrajectory;
            double aheadPlanningTime = parameter.maxComputationTimePathOnReset + parameter.maxComputationTimeMotionOnReset + parameter.additionalAheadPlanningTime;
            if(parameter.predictInitialStateOnReset){
                auto [success, simulationTime] = sequentialPlanner.PredictMotionTrajectory(predictionTrajectory, initialStateAndInput, aheadPlanningTime);
                if(!success){
                    state.Clear();
                    return;
                }
                timestamp += simulationTime;
                initialStateAndInput = predictionTrajectory.back();
            }
            else{
                predictionTrajectory.push_back(initialStateAndInput);
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            sequentialPlannerInput.finalPose = dataIn.finalPose;
            sequentialPlannerInput.originOldToNew = offsetOldToNew;
            sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = parameter.maxComputationTimePath + parameter.maxComputationTimeMotion + parameter.additionalAheadPlanningTime + parameter.additionalTrajectoryDuration;
            sequentialPlanner.Solve(sequentialPlannerOutput, sequentialPlannerInput, parameter.maxComputationTimePathOnReset, parameter.maxComputationTimeMotionOnReset, minTrajectoryDuration);
            timestamp += sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            state.motionPlanner.startingTimepoint = timestamp;
            state.motionPlanner.trajectory = sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + performanceCounter.TimeToStart();

            // Merge prediction trajectory to solution trajectory
            double dt = timestamp - timeNow + parameter.timeKeepPastTrajectory;
            int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::ceil(dt / sequentialPlannerOutput.motionPlanner.sampletime));
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, 0x0000FFFF);
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, static_cast<int32_t>(predictionTrajectory.size()));
            if(numOldTrajectoryPoints > 0){
                // The prediction trajectory relates to the same origin, thus offsetOldToNew does not have to be considered
                sequentialPlannerOutput.motionPlanner.trajectory.insert(sequentialPlannerOutput.motionPlanner.trajectory.begin(), predictionTrajectory.end() - numOldTrajectoryPoints, predictionTrajectory.end());
                timestamp -= static_cast<double>(numOldTrajectoryPoints) * sequentialPlannerOutput.motionPlanner.sampletime;
            }

            // Update the internal state: save current planning results (copy values, swap containers)
            state.timestamp = timestamp;
            state.originLLA = dataIn.originLLA;
            state.trajectory.swap(sequentialPlannerOutput.motionPlanner.trajectory);
            state.sampletime = sequentialPlannerOutput.motionPlanner.sampletime;
            state.performedReset = true;
            state.error = (!sequentialPlannerOutput.pathPlanner.isFeasible || sequentialPlannerOutput.pathPlanner.outOfNodes || !sequentialPlannerOutput.motionPlanner.isFeasible || sequentialPlannerOutput.motionPlanner.outOfNodes);
            state.pathPlanner.path.swap(sequentialPlannerOutput.pathPlanner.path);
            state.pathPlanner.goalReached = sequentialPlannerOutput.pathPlanner.goalReached;
            state.pathPlanner.isFeasible = sequentialPlannerOutput.pathPlanner.isFeasible;
            state.pathPlanner.outOfNodes = sequentialPlannerOutput.pathPlanner.outOfNodes;
            state.pathPlanner.numberOfPerformedIterations = sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            state.pathPlanner.timestampOfComputationUTC = sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            state.pathPlanner.cost = sequentialPlannerOutput.pathPlanner.cost;
            state.motionPlanner.referencePath.swap(sequentialPlannerOutput.motionPlanner.referencePath);
            state.motionPlanner.goalReached = sequentialPlannerOutput.motionPlanner.goalReached;
            state.motionPlanner.isFeasible = sequentialPlannerOutput.motionPlanner.isFeasible;
            state.motionPlanner.outOfNodes = sequentialPlannerOutput.motionPlanner.outOfNodes;
            state.motionPlanner.numberOfPerformedIterations = sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            state.motionPlanner.timestampOfComputationUTC = sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            state.motionPlanner.cost = sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            reset = state.error;
        }

        /**
         * @brief Solve the next online sequential planning problem by selecting the initial state from a future timepoint on the previous trajectory solution.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles).
         */
        void SolveNextProblem(const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            // Calculate the absolute starting timepoint for the next problem
            performanceCounter.Start();
            double aheadPlanningTime = parameter.maxComputationTimePath + parameter.maxComputationTimeMotion + parameter.additionalAheadPlanningTime;
            double futureTimestamp = dataIn.timestamp + aheadPlanningTime;

            // Get the offset from the old origin (state) to the new origin (dataIn)
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, state.originLLA);

            // The new initial state and timepoint for the next problem to be solved is selected from the latest trajectory (state.trajectory) at the future timepoint
            auto [index, timestamp, initialStateAndInput] = GetTrajectoryStateAtTimestamp(futureTimestamp, offsetOldToNew);
            if(index < 0){
                // Timepoint does not lie on trajectory: solve an initial problem
                SolveInitialProblem(dataIn);
                return;
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            sequentialPlannerInput.finalPose = dataIn.finalPose;
            sequentialPlannerInput.originOldToNew = offsetOldToNew;
            sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = parameter.maxComputationTimePath + parameter.maxComputationTimeMotion + parameter.additionalAheadPlanningTime + parameter.additionalTrajectoryDuration;
            sequentialPlanner.Solve(sequentialPlannerOutput, sequentialPlannerInput, parameter.maxComputationTimePath, parameter.maxComputationTimeMotion, minTrajectoryDuration);
            timestamp += sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            state.motionPlanner.startingTimepoint = timestamp;
            state.motionPlanner.trajectory = sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + performanceCounter.TimeToStart();

            // Get indices to range of old trajectory that should be kept
            double dt = timestamp - timeNow + parameter.timeKeepPastTrajectory;
            int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::ceil(dt / state.sampletime));
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, 0x0000FFFF);
            int32_t indexOldEnd = index + 1;
            int32_t indexOldBegin = std::max(0, indexOldEnd - numOldTrajectoryPoints);
            numOldTrajectoryPoints = indexOldEnd - indexOldBegin; // index-range is [indexOldBegin, indexOldEnd), excluding indexOldEnd

            // Either use the new trajectory or keep the old one
            if(ChangeToNewTrajectory(state.timestamp, state.trajectory, timestamp, sequentialPlannerOutput.motionPlanner.trajectory, offsetOldToNew, dataIn.staticObstacles)){
                if(numOldTrajectoryPoints > 0){
                    timestamp -= static_cast<double>(numOldTrajectoryPoints)*sequentialPlannerOutput.motionPlanner.sampletime;
                    // The old trajectory refers to the old origin and must be transformed to the new origin
                    for(int32_t i = indexOldBegin; i < indexOldEnd; ++i){
                        state.trajectory[i][0] -= offsetOldToNew[0];
                        state.trajectory[i][1] -= offsetOldToNew[1];
                    }
                    // Insert part of old trajectory at beginning of new solution trajectory
                    sequentialPlannerOutput.motionPlanner.trajectory.insert(sequentialPlannerOutput.motionPlanner.trajectory.begin(), state.trajectory.begin() + indexOldBegin, state.trajectory.begin() + indexOldEnd);
                }
            }
            else{
                // Keep the old trajectory
                sequentialPlannerOutput.motionPlanner.trajectory.swap(state.trajectory);
                timestamp = state.timestamp;

                // Remove too old trajectory points and adjust timestamp
                if(indexOldBegin > 0){
                    sequentialPlannerOutput.motionPlanner.trajectory.erase(sequentialPlannerOutput.motionPlanner.trajectory.begin(), sequentialPlannerOutput.motionPlanner.trajectory.begin() + indexOldBegin);
                    timestamp += static_cast<double>(indexOldBegin)*sequentialPlannerOutput.motionPlanner.sampletime;
                }

                // Transform old trajectory to new origin
                for(auto&& x : sequentialPlannerOutput.motionPlanner.trajectory){
                    x[0] -= offsetOldToNew[0];
                    x[1] -= offsetOldToNew[1];
                }
            }

            // Update the internal state: save current planning results (copy values, swap containers)
            state.timestamp = timestamp;
            state.originLLA = dataIn.originLLA;
            state.trajectory.swap(sequentialPlannerOutput.motionPlanner.trajectory);
            state.sampletime = sequentialPlannerOutput.motionPlanner.sampletime;
            state.performedReset = false;
            state.error = (!sequentialPlannerOutput.pathPlanner.isFeasible || sequentialPlannerOutput.pathPlanner.outOfNodes || !sequentialPlannerOutput.motionPlanner.isFeasible || sequentialPlannerOutput.motionPlanner.outOfNodes);
            state.pathPlanner.path.swap(sequentialPlannerOutput.pathPlanner.path);
            state.pathPlanner.goalReached = sequentialPlannerOutput.pathPlanner.goalReached;
            state.pathPlanner.isFeasible = sequentialPlannerOutput.pathPlanner.isFeasible;
            state.pathPlanner.outOfNodes = sequentialPlannerOutput.pathPlanner.outOfNodes;
            state.pathPlanner.numberOfPerformedIterations = sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            state.pathPlanner.timestampOfComputationUTC = sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            state.pathPlanner.cost = sequentialPlannerOutput.pathPlanner.cost;
            state.motionPlanner.referencePath.swap(sequentialPlannerOutput.motionPlanner.referencePath);
            state.motionPlanner.goalReached = sequentialPlannerOutput.motionPlanner.goalReached;
            state.motionPlanner.isFeasible = sequentialPlannerOutput.motionPlanner.isFeasible;
            state.motionPlanner.outOfNodes = sequentialPlannerOutput.motionPlanner.outOfNodes;
            state.motionPlanner.numberOfPerformedIterations = sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            state.motionPlanner.timestampOfComputationUTC = sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            state.motionPlanner.cost = sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            reset = state.error;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

