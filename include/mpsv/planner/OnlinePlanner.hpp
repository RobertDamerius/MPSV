#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/SequentialPlanner.hpp>
#include <mpsv/planner/SequentialPlannerOutput.hpp>
#include <mpsv/planner/SequentialPlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>
#include <mpsv/planner/OnlinePlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerOutput.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/control/VehicleSimulator.hpp>
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
            m_sequentialPlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells);
            m_state.Clear();
            m_resetRequired = true;
        }

        /**
         * @brief Solve the online sequential motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[inout] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). When solving sequential planning problems, the timestamp must be strictly greater than the previous input timestamp. The data, e.g. static obstacles, may be modified by this member function.
         * @param[in] parameter The parameter set to use for this solve operation.
         * @param[in] reset If true, reset the planner and solve based on the initial state by optionally predicting the initial state by the expected computation time. The default value is false
         * @details A solution trajectory is saved to the internal @ref m_state of this planner. The next solve operation selects the initial state for the planning
         * problem on a future timepoint of the latest solution trajectory. If this is not possible, a reset is done internally and the planner falls back using the
         * initial state given in the input data. The output indicates, whether a reset has been performed or not.
         * @note An explicit or internally required reset starts a new timestamp baseline.
         */
        void Solve(mpsv::planner::OnlinePlannerOutput& dataOut, mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter, bool reset = false) noexcept {
            // validate parameters and input
            mpsv::error_code errorCode = parameter.IsValid();
            if(mpsv::error_code::NONE == errorCode){
                errorCode = dataIn.IsValid();
            }
            if((mpsv::error_code::NONE == errorCode) && !m_state.trajectory.empty() && (m_state.sampletime != parameter.sequentialPlanner.motionPlanner.sampletime)){
                m_resetRequired = true;
            }

            // solve the planning problem (update m_state)
            if(mpsv::error_code::NONE == errorCode){
                if(m_resetRequired || reset){
                    SolveInitialProblem(dataIn, parameter);
                    m_lastInputTimestamp = dataIn.timestamp;
                }
                else if(!(dataIn.timestamp > m_lastInputTimestamp)){
                    errorCode = mpsv::error_code::ONLINEPLANNER_TIMESTAMP_NOT_INCREASING;
                }
                else{
                    SolveNextProblem(dataIn, parameter);
                    m_lastInputTimestamp = dataIn.timestamp;
                }
            }

            // check for errors and assign output
            if(mpsv::error_code::NONE != errorCode){
                m_state.Clear();
                m_state.errorCode = errorCode;
                m_resetRequired = true;
            }
            dataOut = m_state;
        }

        /**
         * @brief Interrupt the @ref Solve operation of the online planner by interrupting the internal sequential planner.
         */
        void Interrupt(void) noexcept { m_sequentialPlanner.Interrupt(); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept { m_sequentialPlanner.WriteToFile(file, preString + "sequentialPlanner."); }

    protected:
        bool m_resetRequired{false};                                        // True if a reset is required, false otherwise.
        double m_lastInputTimestamp{};                                      // Timestamp of the latest accepted input.
        mpsv::planner::OnlinePlannerOutput m_state;                         // Internal state of the online planner representing the latest output.
        mpsv::planner::SequentialPlannerInput m_sequentialPlannerInput;     // Input to the @ref m_sequentialPlanner.
        mpsv::planner::SequentialPlannerOutput m_sequentialPlannerOutput;   // Output of the @ref m_sequentialPlanner.
        mpsv::planner::SequentialPlanner m_sequentialPlanner;               // Internal sequential planning solver.
        mpsv::runtime::PerformanceCounter m_performanceCounter;             // Performance counter used to measure execution time.
        mpsv::control::VehicleSimulator m_vehicleSimulator;                 // Simulator used for predicting the initial state on reset.
        std::vector<std::array<double,12>> m_predictionTrajectory;          // Reusable buffer for predicted reset trajectories.

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
            if(!std::isfinite(dSteps) || (dSteps < 0.0) || (dSteps >= static_cast<double>(std::numeric_limits<int32_t>::max())) || (static_cast<size_t>(dSteps) >= m_state.trajectory.size())){
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
         * @brief Update @ref m_state and @ref m_resetRequired based on sequential planning results.
         * @param[in] originLLA Geographic origin of the current planning input.
         * @param[in] timestamp Timestamp indicating the initial timepoint of the trajectory.
         * @param[in] performedReset Flag indicating if a reset has been performed or not.
         */
        void UpdateState(const std::array<double,3>& originLLA, double timestamp, bool performedReset) noexcept {
            m_state.timestamp = timestamp;
            m_state.originLLA = originLLA;
            m_state.trajectory.swap(m_sequentialPlannerOutput.motionPlanner.trajectory);
            m_state.sampletime = m_sequentialPlannerOutput.motionPlanner.sampletime;
            m_state.performedReset = performedReset;
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
            m_resetRequired = (m_state.errorCode != mpsv::error_code::NONE);
        }

        /**
         * @brief Solve the first online sequential planning problem using the initial state given in the input and optionally predicting it by the expected computation time.
         * @param[inout] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). The data, e.g. static obstacles, may be modified by this member function.
         * @param[in] parameter The parameter set to use for this solve operation.
         */
        void SolveInitialProblem(mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            m_performanceCounter.Start();
            double timestamp = dataIn.timestamp;
            std::array<double,12> initialStateAndInput = dataIn.initialStateAndInput;

            // Change the timestamp and the initial state vector by predicting the motion for the expected computation time
            m_predictionTrajectory.clear();
            double aheadPlanningTime = parameter.onlinePlanner.maxComputationTimePathOnReset + parameter.onlinePlanner.maxComputationTimeMotionOnReset + parameter.onlinePlanner.additionalAheadPlanningTime;
            mpsv::error_code e = m_vehicleSimulator.SetParameter(parameter.sequentialPlanner.model, parameter.sequentialPlanner.motionPlanner.controller);
            if(mpsv::error_code::NONE == e){
                constexpr uint32_t maxNumSimulationSteps = 0x0000FFFF;
                double requestedNumSteps = std::floor(aheadPlanningTime / parameter.sequentialPlanner.motionPlanner.sampletime);
                uint32_t numSimulationSteps = static_cast<uint32_t>(std::clamp(std::isnan(requestedNumSteps) ? 1.0 : requestedNumSteps, 1.0, static_cast<double>(maxNumSimulationSteps)));
                (void) m_vehicleSimulator.PredictMotionTrajectory(m_predictionTrajectory, initialStateAndInput, parameter.sequentialPlanner.motionPlanner.sampletime, numSimulationSteps, maxNumSimulationSteps);
                double simulationTime = parameter.sequentialPlanner.motionPlanner.sampletime * static_cast<double>(m_predictionTrajectory.size());
                timestamp += simulationTime;
            }
            if(m_predictionTrajectory.empty()){
                m_predictionTrajectory.push_back(initialStateAndInput);
            }
            initialStateAndInput = m_predictionTrajectory.back();

            // Assign input data for the sequential planner and solve the actual planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = {0.0, 0.0};
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles.swap(dataIn.staticObstacles);
            double minTrajectoryDuration = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime + parameter.onlinePlanner.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, parameter.sequentialPlanner, parameter.onlinePlanner.maxComputationTimePathOnReset, parameter.onlinePlanner.maxComputationTimeMotionOnReset, minTrajectoryDuration, true);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Merge prediction trajectory to solution trajectory
            if((m_sequentialPlannerOutput.errorCode == mpsv::error_code::NONE) && (m_sequentialPlannerOutput.motionPlanner.sampletime > 0.0) && !m_sequentialPlannerOutput.motionPlanner.trajectory.empty()){
                double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart(); // current timestamp based on user timestamp definition
                double dt = timestamp - timeNow + parameter.onlinePlanner.timeKeepPastTrajectory;
                int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::min<double>(std::ceil(dt / m_sequentialPlannerOutput.motionPlanner.sampletime), std::min<size_t>(m_predictionTrajectory.size(), 0x0000FFFF)));
                if(numOldTrajectoryPoints > 0){
                    // The prediction trajectory relates to the same origin, thus offsetOldToNew does not have to be considered
                    m_sequentialPlannerOutput.motionPlanner.trajectory.insert(m_sequentialPlannerOutput.motionPlanner.trajectory.begin(), m_predictionTrajectory.end() - numOldTrajectoryPoints, m_predictionTrajectory.end());
                    timestamp -= static_cast<double>(numOldTrajectoryPoints) * m_sequentialPlannerOutput.motionPlanner.sampletime;
                }
            }
            UpdateState(dataIn.originLLA, timestamp, true);
        }

        /**
         * @brief Solve the next online sequential planning problem by selecting the initial state from a future timepoint on the previous trajectory solution.
         * @param[inout] dataIn Input data containing the scene data (time, initial state, final pose, obstacles). The data, e.g. static obstacles, may be modified by this member function.
         * @param[in] parameter The parameter set to use for this solve operation.
         */
        void SolveNextProblem(mpsv::planner::OnlinePlannerInput& dataIn, const mpsv::planner::OnlinePlannerParameterSet& parameter) noexcept {
            m_performanceCounter.Start();
            double aheadPlanningTime = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime;
            double futureTimestamp = dataIn.timestamp + aheadPlanningTime;
            std::array<double,2> offsetOldToNew = mpsv::math::wgs84::LLA2NE(dataIn.originLLA, m_state.originLLA);

            // The new initial state and timepoint for the next problem to be solved is selected from the latest trajectory (m_state.trajectory) at the future timepoint
            auto [index, timestamp, initialStateAndInput] = GetTrajectoryStateAtTimestamp(futureTimestamp, offsetOldToNew);
            if(index < 0){
                SolveInitialProblem(dataIn, parameter); // timepoint does not lie on trajectory: solve an initial problem
                return;
            }

            // Assign input data for the sequential planner and solve the actual motion planning problem
            m_sequentialPlannerInput.initialStateAndInput = initialStateAndInput;
            m_sequentialPlannerInput.finalPose = dataIn.finalPose;
            m_sequentialPlannerInput.originOldToNew = offsetOldToNew;
            m_sequentialPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_sequentialPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_sequentialPlannerInput.staticObstacles.swap(dataIn.staticObstacles);
            double minTrajectoryDuration = parameter.onlinePlanner.maxComputationTimePath + parameter.onlinePlanner.maxComputationTimeMotion + parameter.onlinePlanner.additionalAheadPlanningTime + parameter.onlinePlanner.additionalTrajectoryDuration;
            m_sequentialPlanner.Solve(m_sequentialPlannerOutput, m_sequentialPlannerInput, parameter.sequentialPlanner, parameter.onlinePlanner.maxComputationTimePath, parameter.onlinePlanner.maxComputationTimeMotion, minTrajectoryDuration, false);
            timestamp += m_sequentialPlannerOutput.motionPlanner.sampletime; // make the timestamp point to the first entry of the trajectory instead of pointing to the initial state (the trajectory itself does not include the initial state)

            // Save internal trajectory to state
            m_state.motionPlanner.startingTimepoint = timestamp;
            m_state.motionPlanner.trajectory = m_sequentialPlannerOutput.motionPlanner.trajectory;

            // Get indices to range of old trajectory that should be kept
            if((m_sequentialPlannerOutput.errorCode == mpsv::error_code::NONE) && (m_sequentialPlannerOutput.motionPlanner.sampletime > 0.0) && !m_sequentialPlannerOutput.motionPlanner.trajectory.empty() && (m_state.sampletime > 0.0)){
                double timeNow = dataIn.timestamp + m_performanceCounter.TimeToStart(); // current timestamp based on user timestamp definition
                double dt = timestamp - timeNow + parameter.onlinePlanner.timeKeepPastTrajectory;
                int32_t numOldTrajectoryPoints = static_cast<int32_t>(std::min<double>(std::ceil(dt / m_state.sampletime), 0x0000FFFF));
                int32_t indexOldEnd = index + 1;
                int32_t indexOldBegin = std::max(0, indexOldEnd - numOldTrajectoryPoints);
                numOldTrajectoryPoints = indexOldEnd - indexOldBegin; // index-range is [indexOldBegin, indexOldEnd), excluding indexOldEnd

                // Merge old trajectory to new solution trajectory
                if(numOldTrajectoryPoints > 0){
                    timestamp -= static_cast<double>(numOldTrajectoryPoints) * m_sequentialPlannerOutput.motionPlanner.sampletime;
                    // The old trajectory refers to the old origin and must be transformed to the new origin
                    for(int32_t i = indexOldBegin; i < indexOldEnd; ++i){
                        m_state.trajectory[i][0] -= offsetOldToNew[0];
                        m_state.trajectory[i][1] -= offsetOldToNew[1];
                    }
                    // Insert part of old trajectory at beginning of new solution trajectory
                    m_sequentialPlannerOutput.motionPlanner.trajectory.insert(m_sequentialPlannerOutput.motionPlanner.trajectory.begin(), m_state.trajectory.begin() + indexOldBegin, m_state.trajectory.begin() + indexOldEnd);
                }
            }
            UpdateState(dataIn.originLLA, timestamp, false);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
