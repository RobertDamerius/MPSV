#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/SequentialPlanner.hpp>
#include <mpsv/planner/SequentialPlannerOutput.hpp>
#include <mpsv/planner/SequentialPlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>
#include <mpsv/planner/OnlinePlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerOutput.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/runtime/Time.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/common/ErrorCode.hpp>
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
         * @brief Initialize or re-initialize the online planner.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         */
        void Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples) noexcept {
            Terminate();
            m_sequentialPlanner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples);
        }

        /**
         * @brief Terminate the online planner. This calls the @ref Reset member function.
         */
        void Terminate(void) noexcept {
            m_sequentialPlanner.Terminate();
            m_parameter.Clear();
            m_state.Clear();
            Reset();
        }

        /**
         * @brief Apply a new parameter set to the internal solvers.
         * @param[in] parameter The parameter set to be applied.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         * @details If a new parameter set has been applied successfully, @ref Reset is called internally.
         */
        error_code ApplyParameterSet(mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            error_code e;
            if(error_code::NONE != (e = parameter.IsValid())){
                return e;
            }
            if(error_code::NONE != (e = m_sequentialPlanner.ApplyParameterSet(parameter.sequentialPlanner))){
                return e;
            }
            m_parameter = parameter.onlinePlanner;
            Reset();
            return error_code::NONE;
        }

        /**
         * @brief Reset the online planner to make it start from the initial state during the next solve operation.
         * @details The parameters are not cleared and the interrupt flags are not changed.
         */
        void Reset(void) noexcept { m_reset = true; }

        /**
         * @brief Solve the online sequential motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). The timestamp must be monotonically increasing.
         * @details After a @ref Reset call this planner solves the motion planning based on the initial state by optionally predicting the initial state by the expected
         * computation time. A solution trajectory is saved to the internal @ref m_state of this planner. The next solve operation selects the initial state for the
         * planning problem on a future timepoint of the latest solution trajectory. If this is not possible, a reset is done internally and the planner falls back using
         * the initial state given in the input data.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::OnlinePlannerOutput& dataOut, const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            if(m_reset){
                SolveInitialProblem(dataIn);
            }
            else{
                SolveNextProblem(dataIn);
            }
            dataOut = m_state;
        }

        /**
         * @brief Interrupt the @ref Solve operation of the online planner by interrupting the internal sequential planner.
         */
        void Interrupt(void) noexcept {
            m_sequentialPlanner.Interrupt();
        }

        /**
         * @brief Call this function directly after the @ref Solve function to further improve the path.
         * @param[in] maxComputationTime The maximum computation time in seconds before stopping the solve iteration loop.
         * @details The path improvement can be interrupted by calling the @ref Interrupt member function.
         */
        void ImprovePath(double maxComputationTime) noexcept { m_sequentialPlanner.ImprovePath(maxComputationTime); }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept { return m_sequentialPlanner.PredictMotion(initialStateAndInput, predictionHorizon); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration. All trajectories of the tree are rebuilt. Otherwise the trajectories from tree nodes are undefined.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept { m_sequentialPlanner.WriteToFile(file, preString + "sequentialPlanner.", staticObstacles); }

    protected:
        mpsv::planner::OnlinePlannerOutput m_state;                         // Internal state of the online planner representing the latest output.
        bool m_reset{false};                                                // True if the online planner has been reset.
        mpsv::planner::SequentialPlanner m_sequentialPlanner;               // Internal sequential planning solver.
        mpsv::planner::SequentialPlannerInput m_sequentialPlannerInput;     // Input to the @ref m_sequentialPlanner.
        mpsv::planner::SequentialPlannerOutput m_sequentialPlannerOutput;   // Output of the @ref m_sequentialPlanner.
        mpsv::planner::ParameterOnlinePlanner m_parameter;                  // Parameter for the online planner.
        mpsv::runtime::PerformanceCounter m_performanceCounter;             // Performance counter used to measure execution time.

        /**
         * @brief Get the state/input vector from the latest solution trajectory (@ref m_state) for a given absolute timestamp.
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
            double timeFromOldStart = timestamp - m_state.timestamp;

            // Convert that time offset to an index and check for a valid range
            double dSteps = timeFromOldStart / m_state.sampletime;
            retIndex = static_cast<int32_t>(std::floor(dSteps));
            if((retIndex < 0) || (retIndex >= static_cast<int32_t>(m_state.trajectory.size()))){
                // Requested timestamp is not within range of latest trajectory
                retIndex = -1;
            }
            else{
                // Update new starting point (time) to lie exactly on the grid of the old trajectory (at the returned index)
                retTimestamp = m_state.timestamp + (m_state.sampletime * static_cast<double>(retIndex));
                retInitialStateAndInput = m_state.trajectory[retIndex];
                retInitialStateAndInput[0] -= offsetOldToNew[0];
                retInitialStateAndInput[1] -= offsetOldToNew[1];
            }
            return result;
        }

        /**
         * @brief Solve the first online sequential planning problem using the initial state given in the input and optionally predicting it by the expected computation time.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles).
         */
        void SolveInitialProblem(const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            // Timestamp and initial state/input
            m_performanceCounter.Start();
            double timestamp = dataIn.timestamp;
            std::array<double,12> initialStateAndInput = dataIn.initialStateAndInput;

            // Get the offset from the old origin (state) to the new origin (dataIn)
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, m_state.originLLA);

            // Change the timestamp and the initial state vector by predicting the motion for the expected computation time
            std::vector<std::array<double,12>> predictionTrajectory;
            double aheadPlanningTime = m_parameter.maxComputationTimePathOnReset + m_parameter.maxComputationTimeMotionOnReset + m_parameter.additionalAheadPlanningTime;
            if(m_parameter.predictInitialStateOnReset){
                auto [success, simulationTime] = m_sequentialPlanner.PredictMotionTrajectory(predictionTrajectory, initialStateAndInput, aheadPlanningTime);
                if(!success){
                    m_state.Clear();
                    return;
                }
                timestamp += simulationTime;
                initialStateAndInput = predictionTrajectory.back();
            }
            else{
                predictionTrajectory.push_back(initialStateAndInput);
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = offsetOldToNew;
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = m_parameter.maxComputationTimePath + m_parameter.maxComputationTimeMotion + m_parameter.additionalAheadPlanningTime + m_parameter.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, m_parameter.maxComputationTimePathOnReset, m_parameter.maxComputationTimeMotionOnReset, minTrajectoryDuration);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart();

            // Merge prediction trajectory to solution trajectory
            double dt = timestamp - timeNow + m_parameter.timeKeepPastTrajectory;
            int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::ceil(dt / m_sequentialPlannerOutput.motionPlanner.sampletime));
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, 0x0000FFFF);
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, static_cast<int32_t>(predictionTrajectory.size()));
            if(numOldTrajectoryPoints > 0){
                // The prediction trajectory relates to the same origin, thus offsetOldToNew does not have to be considered
                m_sequentialPlannerOutput.motionPlanner.trajectory.insert(m_sequentialPlannerOutput.motionPlanner.trajectory.begin(), predictionTrajectory.end() - numOldTrajectoryPoints, predictionTrajectory.end());
                timestamp -= static_cast<double>(numOldTrajectoryPoints) * m_sequentialPlannerOutput.motionPlanner.sampletime;
            }

            // Update the internal state: save current planning results (copy values, swap containers)
            m_state.timestamp = timestamp;
            m_state.originLLA = dataIn.originLLA;
            m_state.trajectory.swap(m_sequentialPlannerOutput.motionPlanner.trajectory);
            m_state.sampletime = m_sequentialPlannerOutput.motionPlanner.sampletime;
            m_state.performedReset = true;
            m_state.error = (!m_sequentialPlannerOutput.pathPlanner.isFeasible || m_sequentialPlannerOutput.pathPlanner.outOfNodes || !m_sequentialPlannerOutput.motionPlanner.isFeasible || m_sequentialPlannerOutput.motionPlanner.outOfNodes);
            m_state.pathPlanner.path.swap(m_sequentialPlannerOutput.pathPlanner.path);
            m_state.pathPlanner.goalReached = m_sequentialPlannerOutput.pathPlanner.goalReached;
            m_state.pathPlanner.isFeasible = m_sequentialPlannerOutput.pathPlanner.isFeasible;
            m_state.pathPlanner.outOfNodes = m_sequentialPlannerOutput.pathPlanner.outOfNodes;
            m_state.pathPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            m_state.pathPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            m_state.pathPlanner.cost = m_sequentialPlannerOutput.pathPlanner.cost;
            m_state.motionPlanner.referencePath.swap(m_sequentialPlannerOutput.motionPlanner.referencePath);
            m_state.motionPlanner.goalReached = m_sequentialPlannerOutput.motionPlanner.goalReached;
            m_state.motionPlanner.isFeasible = m_sequentialPlannerOutput.motionPlanner.isFeasible;
            m_state.motionPlanner.outOfNodes = m_sequentialPlannerOutput.motionPlanner.outOfNodes;
            m_state.motionPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            m_state.motionPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            m_state.motionPlanner.cost = m_sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            m_reset = m_state.error;
        }

        /**
         * @brief Solve the next online sequential planning problem by selecting the initial state from a future timepoint on the previous trajectory solution.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles).
         */
        void SolveNextProblem(const mpsv::planner::OnlinePlannerInput& dataIn) noexcept {
            // Calculate the absolute starting timepoint for the next problem
            m_performanceCounter.Start();
            double aheadPlanningTime = m_parameter.maxComputationTimePath + m_parameter.maxComputationTimeMotion + m_parameter.additionalAheadPlanningTime;
            double futureTimestamp = dataIn.timestamp + aheadPlanningTime;

            // Get the offset from the old origin (state) to the new origin (dataIn)
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, m_state.originLLA);

            // The new initial state and timepoint for the next problem to be solved is selected from the latest trajectory (state.trajectory) at the future timepoint
            auto [index, timestamp, initialStateAndInput] = GetTrajectoryStateAtTimestamp(futureTimestamp, offsetOldToNew);
            if(index < 0){
                // Timepoint does not lie on trajectory: solve an initial problem
                SolveInitialProblem(dataIn);
                return;
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = offsetOldToNew;
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = m_parameter.maxComputationTimePath + m_parameter.maxComputationTimeMotion + m_parameter.additionalAheadPlanningTime + m_parameter.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, m_parameter.maxComputationTimePath, m_parameter.maxComputationTimeMotion, minTrajectoryDuration);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart();

            // Get indices to range of old trajectory that should be kept
            double dt = timestamp - timeNow + m_parameter.timeKeepPastTrajectory;
            int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::ceil(dt / m_state.sampletime));
            numOldTrajectoryPoints = std::min(numOldTrajectoryPoints, 0x0000FFFF);
            int32_t indexOldEnd = index + 1;
            int32_t indexOldBegin = std::max(0, indexOldEnd - numOldTrajectoryPoints);
            numOldTrajectoryPoints = indexOldEnd - indexOldBegin; // index-range is [indexOldBegin, indexOldEnd), excluding indexOldEnd

            // Merge old trajectory to new solution trajectory
            if(numOldTrajectoryPoints > 0){
                timestamp -= static_cast<double>(numOldTrajectoryPoints)*m_sequentialPlannerOutput.motionPlanner.sampletime;
                // The old trajectory refers to the old origin and must be transformed to the new origin
                for(int32_t i = indexOldBegin; i < indexOldEnd; ++i){
                    m_state.trajectory[i][0] -= offsetOldToNew[0];
                    m_state.trajectory[i][1] -= offsetOldToNew[1];
                }
                // Insert part of old trajectory at beginning of new solution trajectory
                m_sequentialPlannerOutput.motionPlanner.trajectory.insert(m_sequentialPlannerOutput.motionPlanner.trajectory.begin(), m_state.trajectory.begin() + indexOldBegin, m_state.trajectory.begin() + indexOldEnd);
            }

            // Update the internal state: save current planning results (copy values, swap containers)
            m_state.timestamp = timestamp;
            m_state.originLLA = dataIn.originLLA;
            m_state.trajectory.swap(m_sequentialPlannerOutput.motionPlanner.trajectory);
            m_state.sampletime = m_sequentialPlannerOutput.motionPlanner.sampletime;
            m_state.performedReset = false;
            m_state.error = (!m_sequentialPlannerOutput.pathPlanner.isFeasible || m_sequentialPlannerOutput.pathPlanner.outOfNodes || !m_sequentialPlannerOutput.motionPlanner.isFeasible || m_sequentialPlannerOutput.motionPlanner.outOfNodes);
            m_state.pathPlanner.path.swap(m_sequentialPlannerOutput.pathPlanner.path);
            m_state.pathPlanner.goalReached = m_sequentialPlannerOutput.pathPlanner.goalReached;
            m_state.pathPlanner.isFeasible = m_sequentialPlannerOutput.pathPlanner.isFeasible;
            m_state.pathPlanner.outOfNodes = m_sequentialPlannerOutput.pathPlanner.outOfNodes;
            m_state.pathPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            m_state.pathPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            m_state.pathPlanner.cost = m_sequentialPlannerOutput.pathPlanner.cost;
            m_state.motionPlanner.referencePath.swap(m_sequentialPlannerOutput.motionPlanner.referencePath);
            m_state.motionPlanner.goalReached = m_sequentialPlannerOutput.motionPlanner.goalReached;
            m_state.motionPlanner.isFeasible = m_sequentialPlannerOutput.motionPlanner.isFeasible;
            m_state.motionPlanner.outOfNodes = m_sequentialPlannerOutput.motionPlanner.outOfNodes;
            m_state.motionPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            m_state.motionPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            m_state.motionPlanner.cost = m_sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            m_reset = m_state.error;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
