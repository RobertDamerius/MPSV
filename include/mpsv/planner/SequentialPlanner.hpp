#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/SequentialPlannerInput.hpp>
#include <mpsv/planner/SequentialPlannerOutput.hpp>
#include <mpsv/planner/SequentialPlannerParameterSet.hpp>
#include <mpsv/planner/PathPlanner.hpp>
#include <mpsv/planner/PathPlannerParameterSet.hpp>
#include <mpsv/planner/PathPlannerInput.hpp>
#include <mpsv/planner/PathPlannerOutput.hpp>
#include <mpsv/planner/MotionPlanner.hpp>
#include <mpsv/planner/MotionPlannerParameterSet.hpp>
#include <mpsv/planner/MotionPlannerInput.hpp>
#include <mpsv/planner/MotionPlannerOutput.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/common/ErrorCode.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The sequential planner (contains path and motion planner). When creating the planner, call the @ref Initialize member function once to setup the planner before calling
 * any other member function. Apply a valid parameter set by calling the @ref ApplyParameterSet member function. Then you can solve the problem by calling the @ref Solve
 * member function. If you wish to apply a new parameter set, call the @ref ApplyParameterSet before the @ref Solve call. To interrupt a running @ref Solve operation, call the @ref Interrupt member function.
 */
class SequentialPlanner {
    public:
        /**
         * @brief Reconfigure the sequential planner with the specified capacities.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         */
        void Reconfigure(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples) noexcept {
            m_pathPlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples);
            m_motionPlanner.Reconfigure(motionMaxNumNodes, motionMaxNumSamples);
        }

        /**
         * @brief Apply a new parameter set to the internal solvers.
         * @param[in] parameter The parameter set to be applied.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code ApplyParameterSet(const mpsv::planner::SequentialPlannerParameterSet& parameter) noexcept {
            // split into two parameter sets for path and motion planning
            mpsv::planner::PathPlannerParameterSet parameterPath;
            mpsv::planner::MotionPlannerParameterSet parameterMotion;
            parameterPath.geometry = parameter.geometry;
            parameterPath.costMap = parameter.costMap;
            parameterPath.metric = parameter.metric;
            parameterPath.pathPlanner = parameter.pathPlanner;
            parameterMotion.geometry = parameter.geometry;
            parameterMotion.costMap = parameter.costMap;
            parameterMotion.metric = parameter.metric;
            parameterMotion.model = parameter.model;
            parameterMotion.motionPlanner = parameter.motionPlanner;

            // Validate and retain parameters for the internal planners.
            error_code e = parameterPath.IsValid();
            if(error_code::NONE != e){
                return e;
            }
            e = parameterMotion.IsValid();
            if(error_code::NONE != e){
                return e;
            }
            m_pathPlannerParameter = parameterPath;
            m_motionPlannerParameter = parameterMotion;
            return error_code::NONE;
        }

        /**
         * @brief Solve the sequential planning problem by solving a path planning problem, followed by a motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn The input data containing the scene of the problem.
         * @param[in] maxComputationTimePathPlanning The maximum computation time allowed for path planning. The iteration loop for path planning is terminated if this computation time limit is exceeded.
         * @param[in] maxComputationTimeMotionPlanning The maximum computation time allowed for motion planning. The iteration loop for motion planning is terminated if this computation time limit is exceeded.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::SequentialPlannerOutput& dataOut, const mpsv::planner::SequentialPlannerInput& dataIn, double maxComputationTimePathPlanning, double maxComputationTimeMotionPlanning, double minTrajectoryDuration = 0.0) noexcept {
            // Start execution time measurement for path planning section
            m_executionCounter.Start();

            // Assign planner inputs
            m_pathPlannerInput.initialPose[0] = dataIn.initialStateAndInput[0];
            m_pathPlannerInput.initialPose[1] = dataIn.initialStateAndInput[1];
            m_pathPlannerInput.initialPose[2] = dataIn.initialStateAndInput[2];
            m_pathPlannerInput.finalPose = dataIn.finalPose;
            m_pathPlannerInput.originOldToNew = dataIn.originOldToNew;
            m_pathPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_pathPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_pathPlannerInput.staticObstacles = dataIn.staticObstacles;
            m_motionPlannerInput.initialStateAndInput = dataIn.initialStateAndInput;
            m_motionPlannerInput.originOldToNew = dataIn.originOldToNew;
            m_motionPlannerInput.staticObstacles = dataIn.staticObstacles;
            m_motionPlannerInput.minTrajectoryDuration = minTrajectoryDuration;

            // Solve path planning problem
            double remainingTimeForProblemSolving = maxComputationTimePathPlanning - m_executionCounter.TimeToStart();
            m_pathPlanner.Solve(m_pathPlannerOutput, m_pathPlannerInput, m_pathPlannerParameter, remainingTimeForProblemSolving);

            // Use solution path of path planner as initial path for motion planner
            m_motionPlannerInput.initialPath = m_pathPlannerOutput.path;

            // Solve motion planning problem
            remainingTimeForProblemSolving = maxComputationTimePathPlanning + maxComputationTimeMotionPlanning - m_executionCounter.TimeToStart();
            m_motionPlanner.Solve(m_motionPlannerOutput, m_motionPlannerInput, m_motionPlannerParameter, remainingTimeForProblemSolving);

            // Move values from planning results to the output data
            m_pathPlannerOutput.MoveTo(dataOut.pathPlanner);
            m_motionPlannerOutput.MoveTo(dataOut.motionPlanner);
        }

        /**
         * @brief Interrupt the @ref Solve operation of the sequential planner by interrupting the internal path and motion planners.
         */
        void Interrupt(void) noexcept {
            m_motionPlanner.Interrupt();
            m_pathPlanner.Interrupt();
        }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept { return m_motionPlanner.PredictMotion(initialStateAndInput, predictionHorizon, m_motionPlannerParameter); }

        /**
         * @brief Predict the motion trajectory of the vehicle for a given prediction horizon assuming the control input to be constant all the time. At least one simulation step is computed.
         * @param[out] trajectory The prediction trajectory where each entry contains the state and the input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of a sampletime.
         * @return A tuple containing the following values.
         * [0] True if success, false otherwise. If the maximum number of simulation steps is exceeded, false is returned.
         * [1] The total time that has been simulated.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept { return m_motionPlanner.PredictMotionTrajectory(trajectory, initialStateAndInput, predictionHorizon, m_motionPlannerParameter); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept { m_pathPlanner.WriteToFile(file, preString + "pathPlanner."); m_motionPlanner.WriteToFile(file, preString + "motionPlanner.", m_motionPlannerInput, m_motionPlannerParameter); }

    protected:
        mpsv::planner::PathPlanner m_pathPlanner;                            // Internal path planning solver.
        mpsv::planner::PathPlannerInput m_pathPlannerInput;                  // The input to the path planner, set during @ref Solve.
        mpsv::planner::PathPlannerOutput m_pathPlannerOutput;                // The output of the path planner, set during @ref Solve and @ref Improve.
        mpsv::planner::PathPlannerParameterSet m_pathPlannerParameter;       // Parameters for the internal path planner.
        mpsv::planner::MotionPlanner m_motionPlanner;                        // Internal motion planning solver.
        mpsv::planner::MotionPlannerInput m_motionPlannerInput;              // The input to the motion planner, set during @ref Solve.
        mpsv::planner::MotionPlannerOutput m_motionPlannerOutput;            // The output of the motion planner, set during @ref Solve.
        mpsv::planner::MotionPlannerParameterSet m_motionPlannerParameter;   // Parameters for the internal motion planner.
        mpsv::runtime::PerformanceCounter m_executionCounter;                // Performance counter to measure the execution time during @ref Solve.
};


} /* namespace: planner */


} /* namespace: mpsv */
