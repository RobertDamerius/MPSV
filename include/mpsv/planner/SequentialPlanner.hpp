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
 * @brief The sequential planner (contains path and motion planner). Use the @ref Solve member function to solve sequential planning problems.
 * The iteration loop can be interrupted by @ref Interrupt. To change the size of pre-allocated memory, run @ref Reconfigure.
 */
class SequentialPlanner {
    public:
        /**
         * @brief Reconfigure the sequential planner with the specified capacities.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @param[in] maxNumCells Maximum number of cells to be used in the cost map.
         */
        void Reconfigure(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells) noexcept {
            m_pathPlanner.Reconfigure(pathMaxNumNodes, pathMaxNumSamples, maxNumCells);
            m_motionPlanner.Reconfigure(motionMaxNumNodes, motionMaxNumSamples, maxNumCells);
        }

        /**
         * @brief Solve the sequential planning problem by solving a path planning problem, followed by a motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn The input data containing the scene of the problem.
         * @param[in] parameter The parameter set to be used.
         * @param[in] maxComputationTimePathPlanning The maximum computation time allowed for path planning. The iteration loop for path planning is terminated if this computation time limit is exceeded.
         * @param[in] maxComputationTimeMotionPlanning The maximum computation time allowed for motion planning. The iteration loop for motion planning is terminated if this computation time limit is exceeded.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0.
         */
        void Solve(mpsv::planner::SequentialPlannerOutput& dataOut, const mpsv::planner::SequentialPlannerInput& dataIn, const mpsv::planner::SequentialPlannerParameterSet& parameter, double maxComputationTimePathPlanning, double maxComputationTimeMotionPlanning, double minTrajectoryDuration = 0.0) noexcept {
            m_interruptSolve = false;

            // Solve path planning problem
            m_pathPlannerInput.initialPose[0] = dataIn.initialStateAndInput[0];
            m_pathPlannerInput.initialPose[1] = dataIn.initialStateAndInput[1];
            m_pathPlannerInput.initialPose[2] = dataIn.initialStateAndInput[2];
            m_pathPlannerInput.finalPose = dataIn.finalPose;
            m_pathPlannerInput.originOldToNew = dataIn.originOldToNew;
            m_pathPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            m_pathPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            m_pathPlannerInput.staticObstacles = dataIn.staticObstacles;
            m_pathPlannerParameter.geometry = parameter.geometry;
            m_pathPlannerParameter.costMap = parameter.costMap;
            m_pathPlannerParameter.metric = parameter.metric;
            m_pathPlannerParameter.pathPlanner = parameter.pathPlanner;
            m_pathPlanner.Solve(m_pathPlannerOutput, m_pathPlannerInput, m_pathPlannerParameter, maxComputationTimePathPlanning, false, &m_interruptSolve);

            // Solve motion planning problem
            m_motionPlannerInput.initialPath = m_pathPlannerOutput.path;
            m_motionPlannerInput.initialStateAndInput = dataIn.initialStateAndInput;
            m_motionPlannerInput.originOldToNew = dataIn.originOldToNew;
            m_motionPlannerInput.staticObstacles = dataIn.staticObstacles;
            m_motionPlannerInput.minTrajectoryDuration = minTrajectoryDuration;
            m_motionPlannerParameter.geometry = parameter.geometry;
            m_motionPlannerParameter.costMap = parameter.costMap;
            m_motionPlannerParameter.metric = parameter.metric;
            m_motionPlannerParameter.model = parameter.model;
            m_motionPlannerParameter.motionPlanner = parameter.motionPlanner;
            if(!m_interruptSolve.load()){
                m_motionPlanner.Solve(m_motionPlannerOutput, m_motionPlannerInput, m_motionPlannerParameter, maxComputationTimeMotionPlanning, false, &m_interruptSolve);
            }
            else{
                m_motionPlannerOutput.Clear();
            }

            // Move results to output data
            m_pathPlannerOutput.MoveTo(dataOut.pathPlanner);
            m_motionPlannerOutput.MoveTo(dataOut.motionPlanner);
        }

        /**
         * @brief Interrupt the @ref Solve operation of the sequential planner, including both internal planners.
         */
        void Interrupt(void) noexcept { m_interruptSolve = true; }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return A tuple containing the following values.
         * [0] The total time that has been simulated.
         * [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
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
         */
        [[nodiscard]] std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept { return m_motionPlanner.PredictMotionTrajectory(trajectory, initialStateAndInput, predictionHorizon, m_motionPlannerParameter); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            m_pathPlanner.WriteToFile(file, preString + "pathPlanner.");
            m_motionPlanner.WriteToFile(file, preString + "motionPlanner.", m_motionPlannerInput, m_motionPlannerParameter);
        }

    protected:
        mpsv::planner::PathPlanner m_pathPlanner;                            // Internal path planning solver.
        mpsv::planner::PathPlannerInput m_pathPlannerInput;                  // The input to the path planner, set during @ref Solve.
        mpsv::planner::PathPlannerOutput m_pathPlannerOutput;                // The output of the path planner, set during @ref Solve and @ref Improve.
        mpsv::planner::PathPlannerParameterSet m_pathPlannerParameter;       // Parameters for the internal path planner.
        mpsv::planner::MotionPlanner m_motionPlanner;                        // Internal motion planning solver.
        mpsv::planner::MotionPlannerInput m_motionPlannerInput;              // The input to the motion planner, set during @ref Solve.
        mpsv::planner::MotionPlannerOutput m_motionPlannerOutput;            // The output of the motion planner, set during @ref Solve.
        mpsv::planner::MotionPlannerParameterSet m_motionPlannerParameter;   // Parameters for the internal motion planner.
        std::atomic<bool> m_interruptSolve{false};                           // True if the active sequential solve should be interrupted.
};


} /* namespace: planner */


} /* namespace: mpsv */
