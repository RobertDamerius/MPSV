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
         * @brief Reconfigure the online planner with the specified capacities.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @param[in] maxNumCells Maximum number of cells to be used in the cost map.
         */
        void Reconfigure(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells) noexcept {
            m_state.Clear();
            Reset();
            m_sequentialPlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);
        }

        /**
         * @brief Reset the online planner to make it start from the initial state during the next solve operation.
         * @details The interrupt flags are not changed. Parameters are supplied separately to each @ref Solve operation.
         */
        void Reset(void) noexcept { m_reset = true; }

        /**
         * @brief Solve the online sequential motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). The timestamp must be monotonically increasing.
         * @param[in] parameter The parameter set to use for this solve operation.
         * @details After a @ref Reset call this planner solves the motion planning based on the initial state by optionally predicting the initial state by the expected
         * computation time. A solution trajectory is saved to the internal @ref m_state of this planner. The next solve operation selects the initial state for the
         * planning problem on a future timepoint of the latest solution trajectory. If this is not possible, a reset is done internally and the planner falls back using
         * the initial state given in the input data.
         */
        void Solve(mpsv::planner::OnlinePlannerOutput& dataOut, const mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            if(m_reset){
                SolveInitialProblem(dataIn, parameter);
            }
            else{
                SolveNextProblem(dataIn, parameter);
            }
            dataOut = m_state;
        }

        /**
         * @brief Interrupt the @ref Solve operation of the online planner by interrupting the internal sequential planner.
         */
        void Interrupt(void) noexcept { m_sequentialPlanner.Interrupt(); }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept { return m_sequentialPlanner.PredictMotion(initialStateAndInput, predictionHorizon); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept { m_sequentialPlanner.WriteToFile(file, preString + "sequentialPlanner."); }

    protected:
        mpsv::planner::OnlinePlannerOutput m_state;                         // Internal state of the online planner representing the latest output.
        bool m_reset{false};                                                // True if the online planner has been reset.
        mpsv::planner::SequentialPlanner m_sequentialPlanner;               // Internal sequential planning solver.
        mpsv::planner::SequentialPlannerInput m_sequentialPlannerInput;     // Input to the @ref m_sequentialPlanner.
        mpsv::planner::SequentialPlannerOutput m_sequentialPlannerOutput;   // Output of the @ref m_sequentialPlanner.
        mpsv::runtime::PerformanceCounter m_performanceCounter;             // Performance counter used to measure execution time.

        /**
         * @brief Get the state/input vector from the latest solution trajectory (@ref m_state) for a given absolute timestamp.
         * @param[in] timestamp The timestamp (time origin defined by user) for which to obtain the state/input vector.
         * @param[in] offsetOldToNew If the origin of the current trajectory (state.trajectory) is different to the origin of the new problem, then this coordinate translation must be taken into account. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
         * @return A tuple containing the following values.
         * [0] Index indicating the state/vector that was chosen from state.trajectory. If the given timestamp is out of range, -1 is returned.
         * [1] The actual timestamp that corresponds to that index.
         * [2] The state/input vector for that index.
         */
        std::tuple<int32_t,double,std::array<double,12>> GetTrajectoryStateAtTimestamp(double timestamp, std::array<double,2> offsetOldToNew) noexcept {
            std::tuple<int32_t,double,std::array<double,12>> result{};
            auto& [retIndex, retTimestamp, retInitialStateAndInput] = result;
            double timeFromOldStart = timestamp - m_state.timestamp;
            double dSteps = std::floor(timeFromOldStart / m_state.sampletime);
            if(!std::isfinite(dSteps) || (dSteps < 0.0) || (dSteps > static_cast<double>(std::numeric_limits<int32_t>::max())) || (static_cast<size_t>(dSteps) >= m_state.trajectory.size())){
                retIndex = -1; // timestamp not within time range of latest trajectory
            }
            else{
                // Update new starting point (time) to lie exactly on the grid of the old trajectory (at the returned index)
                retIndex = static_cast<int32_t>(dSteps);
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
         * @param[in] parameter The parameter set to use for this solve operation.
         */
        void SolveInitialProblem(const mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            m_performanceCounter.Start();
            double timestamp = dataIn.timestamp;
            std::array<double,12> initialStateAndInput = dataIn.initialStateAndInput;
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, m_state.originLLA);

            // Change the timestamp and the initial state vector by predicting the motion for the expected computation time
            std::vector<std::array<double,12>> predictionTrajectory;
            double aheadPlanningTime = parameter.onlinePlanner.maxComputationTimePathOnReset + parameter.onlinePlanner.maxComputationTimeMotionOnReset + parameter.onlinePlanner.additionalAheadPlanningTime;
            if(parameter.onlinePlanner.predictInitialStateOnReset){
                auto [success, simulationTime] = m_sequentialPlanner.PredictMotionTrajectory(predictionTrajectory, initialStateAndInput, aheadPlanningTime);
                if(!success || predictionTrajectory.empty()){
                    m_state.Clear();
                    m_state.errorCode = mpsv::error_code::MOTIONPLANNER_INFEASIBLE_TRAJECTORY;
                    return;
                }
                timestamp += simulationTime;
                initialStateAndInput = predictionTrajectory.back();
            }
            else{
                predictionTrajectory.push_back(initialStateAndInput);
            }

            // Assign input data for the sequential planner and solve the actual planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = offsetOldToNew;
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime + parameter.onlinePlanner.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, parameter.sequentialPlanner, parameter.onlinePlanner.maxComputationTimePathOnReset, parameter.onlinePlanner.maxComputationTimeMotionOnReset, minTrajectoryDuration);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart();

            // Merge prediction trajectory to solution trajectory
            double dt = timestamp - timeNow + parameter.onlinePlanner.timeKeepPastTrajectory;
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
            m_state.errorCode = m_sequentialPlannerOutput.errorCode;
            m_state.pathPlanner.path.swap(m_sequentialPlannerOutput.pathPlanner.path);
            m_state.pathPlanner.goalReached = m_sequentialPlannerOutput.pathPlanner.goalReached;
            m_state.pathPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            m_state.pathPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            m_state.pathPlanner.cost = m_sequentialPlannerOutput.pathPlanner.cost;
            m_state.motionPlanner.referencePath.swap(m_sequentialPlannerOutput.motionPlanner.referencePath);
            m_state.motionPlanner.goalReached = m_sequentialPlannerOutput.motionPlanner.goalReached;
            m_state.motionPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            m_state.motionPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            m_state.motionPlanner.cost = m_sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            m_reset = (m_state.errorCode != mpsv::error_code::NONE);
        }

        /**
         * @brief Solve the next online sequential planning problem by selecting the initial state from a future timepoint on the previous trajectory solution.
         * @param[in] dataIn Input data containing the scene data (time, initial state, final pose, obstacles).
         * @param[in] parameter The parameter set to use for this solve operation.
         */
        void SolveNextProblem(const mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            // Calculate the absolute starting timepoint for the next problem
            m_performanceCounter.Start();
            double aheadPlanningTime = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime;
            double futureTimestamp = dataIn.timestamp + aheadPlanningTime;

            // Get the offset from the old origin (state) to the new origin (dataIn)
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, m_state.originLLA);

            // The new initial state and timepoint for the next problem to be solved is selected from the latest trajectory (state.trajectory) at the future timepoint
            auto [index, timestamp, initialStateAndInput] = GetTrajectoryStateAtTimestamp(futureTimestamp, offsetOldToNew);
            if(index < 0){
                // Timepoint does not lie on trajectory: solve an initial problem
                SolveInitialProblem(dataIn, parameter);
                return;
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = offsetOldToNew;
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles = dataIn.staticObstacles;
            double minTrajectoryDuration = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime + parameter.onlinePlanner.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, parameter.sequentialPlanner, parameter.onlinePlanner.maxComputationTimePath, parameter.onlinePlanner.maxComputationTimeMotion, minTrajectoryDuration);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Get current timestamp based on user timestamp definition
            double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart();

            // Get indices to range of old trajectory that should be kept
            double dt = timestamp - timeNow + parameter.onlinePlanner.timeKeepPastTrajectory;
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
            m_state.errorCode = m_sequentialPlannerOutput.errorCode;
            m_state.pathPlanner.path.swap(m_sequentialPlannerOutput.pathPlanner.path);
            m_state.pathPlanner.goalReached = m_sequentialPlannerOutput.pathPlanner.goalReached;
            m_state.pathPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.pathPlanner.numberOfPerformedIterations;
            m_state.pathPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.pathPlanner.timestampOfComputationUTC;
            m_state.pathPlanner.cost = m_sequentialPlannerOutput.pathPlanner.cost;
            m_state.motionPlanner.referencePath.swap(m_sequentialPlannerOutput.motionPlanner.referencePath);
            m_state.motionPlanner.goalReached = m_sequentialPlannerOutput.motionPlanner.goalReached;
            m_state.motionPlanner.numberOfPerformedIterations = m_sequentialPlannerOutput.motionPlanner.numberOfPerformedIterations;
            m_state.motionPlanner.timestampOfComputationUTC = m_sequentialPlannerOutput.motionPlanner.timestampOfComputationUTC;
            m_state.motionPlanner.cost = m_sequentialPlannerOutput.motionPlanner.cost;

            // Set reset flag
            m_reset = (m_state.errorCode != mpsv::error_code::NONE);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
