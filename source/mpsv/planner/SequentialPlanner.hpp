#pragma once


#include <mpsv/core/MPSVCommon.hpp>
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
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/PerformanceCounter.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The sequential planner (contains path and motion planner). When creating the planner, call the @ref Initialize member function once to setup the planner before calling
 * any other member function. Apply a valid parameter set by calling the @ref ApplyParameterSet member function. Then you can solve the problem by calling the @ref Solve
 * member function. If you wish to apply a new parameter set, call the @ref ApplyParameterSet before the @ref Solve call. You can further improve the path calling the
 * @ref ImprovePath member function after the @ref Solve member function. To interrupt a running @ref Solve operation, call the @ref Interrupt member function. If no @ref Solve operation
 * is running while perfoming an interrupt, then the next @ref Solve operation is to be interrupted.
 */
class SequentialPlanner {
    public:
        /**
         * @brief Construct a new sequential planner object and set default values.
         */
        SequentialPlanner() noexcept { Terminate(); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initiallization / Termination
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Initialize the sequential planner.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planner).
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planner).
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planner).
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planner).
         * @return True if success, false otherwise.
         * @details Call this function before calling any of the other member functions.
         */
        bool Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples) noexcept {
            Terminate();
            if(!pathPlanner.Initialize(pathMaxNumNodes, pathMaxNumSamples) || !motionPlanner.Initialize(motionMaxNumNodes, motionMaxNumSamples)){
                Terminate();
                return false;
            }
            return true;
        }

        /**
         * @brief Terminate the sequential planner. This also resets the interrupt flags to false.
         */
        void Terminate(void) noexcept {
            pathPlanner.Terminate();
            motionPlanner.Terminate();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Algorithm API: ApplyParameterSet and Solve
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Apply a new parameter set to the internal solvers.
         * @param[in] parameter The parameter set to be applied.
         * @return True if parameter set is valid and has been applied, false otherwise.
         */
        bool ApplyParameterSet(const mpsv::planner::SequentialPlannerParameterSet& parameter) noexcept {
            // Split into two parameter sets for path and motion planning
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

            // Apply parameter set to path planner
            if(!pathPlanner.ApplyParameterSet(parameterPath)){
                return false;
            }

            // Apply parameter set to motion planner
            if(!motionPlanner.ApplyParameterSet(parameterMotion)){
                return false;
            }
            return true;
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
            executionCounter.Start();

            // Assign planner inputs
            pathPlannerInput.initialPose[0] = dataIn.initialStateAndInput[0];
            pathPlannerInput.initialPose[1] = dataIn.initialStateAndInput[1];
            pathPlannerInput.initialPose[2] = dataIn.initialStateAndInput[2];
            pathPlannerInput.finalPose = dataIn.finalPose;
            pathPlannerInput.originOldToNew = dataIn.originOldToNew;
            pathPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            pathPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            pathPlannerInput.staticObstacles = dataIn.staticObstacles;
            motionPlannerInput.initialStateAndInput = dataIn.initialStateAndInput;
            motionPlannerInput.originOldToNew = dataIn.originOldToNew;
            motionPlannerInput.staticObstacles = dataIn.staticObstacles;

            // Solve path planning problem
            pathPlanner.Prepare(pathPlannerOutput, pathPlannerInput);
            double remainingTimeForProblemSolving = maxComputationTimePathPlanning - executionCounter.TimeToStart();
            pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, remainingTimeForProblemSolving);

            // Use solution path of path planner as initial path for motion planner
            motionPlannerInput.initialPath = pathPlannerOutput.path;

            // Solve motion planning problem
            motionPlanner.Prepare(motionPlannerOutput, motionPlannerInput);
            remainingTimeForProblemSolving = maxComputationTimePathPlanning + maxComputationTimeMotionPlanning - executionCounter.TimeToStart();
            motionPlanner.Solve(motionPlannerOutput, motionPlannerInput, remainingTimeForProblemSolving, minTrajectoryDuration);

            // Move values from planning results to the output data
            pathPlannerOutput.MoveTo(dataOut.pathPlanner);
            motionPlannerOutput.MoveTo(dataOut.motionPlanner);
        }

        /**
         * @brief Solve the sequential planning problem by solving a path planning problem, followed by a motion planning problem.
         * @param[out] dataOut The output data containing the results.
         * @param[in] dataIn The input data containing the scene of the problem.
         * @param[in] maxNumIterationsPath The maximum number of iterations to be performed for path planning.
         * @param[in] maxNumIterationsMotion The maximum number of iterations to be performed for motion planning.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0.
         * @param[in] forceColdStart True if no warm start should be performed, false otherwise. The default value is false.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::SequentialPlannerOutput& dataOut, const mpsv::planner::SequentialPlannerInput& dataIn, uint32_t maxNumIterationsPath, uint32_t maxNumIterationsMotion, double minTrajectoryDuration = 0.0, bool forceColdStart = false) noexcept {
            // Start execution time measurement for path planning section
            executionCounter.Start();

            // Assign planner inputs
            pathPlannerInput.initialPose[0] = dataIn.initialStateAndInput[0];
            pathPlannerInput.initialPose[1] = dataIn.initialStateAndInput[1];
            pathPlannerInput.initialPose[2] = dataIn.initialStateAndInput[2];
            pathPlannerInput.finalPose = dataIn.finalPose;
            pathPlannerInput.originOldToNew = dataIn.originOldToNew;
            pathPlannerInput.samplingBoxCenterPose = dataIn.samplingBoxCenterPose;
            pathPlannerInput.samplingBoxDimension = dataIn.samplingBoxDimension;
            pathPlannerInput.staticObstacles = dataIn.staticObstacles;
            motionPlannerInput.initialStateAndInput = dataIn.initialStateAndInput;
            motionPlannerInput.originOldToNew = dataIn.originOldToNew;
            motionPlannerInput.staticObstacles = dataIn.staticObstacles;

            // Solve path planning problem
            pathPlanner.Prepare(pathPlannerOutput, pathPlannerInput, forceColdStart);
            pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, maxNumIterationsPath);

            // Use solution path of path planner as initial path for motion planner
            motionPlannerInput.initialPath = pathPlannerOutput.path;

            // Solve motion planning problem
            motionPlanner.Prepare(motionPlannerOutput, motionPlannerInput, forceColdStart);
            motionPlanner.Solve(motionPlannerOutput, motionPlannerInput, maxNumIterationsMotion, minTrajectoryDuration);

            // Move values from planning results to the output data
            pathPlannerOutput.MoveTo(dataOut.pathPlanner);
            motionPlannerOutput.MoveTo(dataOut.motionPlanner);
        }

        /**
         * @brief Interrupt the @ref Solve operation of the sequential planner by interrupting the internal path and motion planners.
         * If no @ref Solve operation is running, then the next @ref Solve operation is to be interrupted.
         */
        void Interrupt(void) noexcept {
            motionPlanner.Interrupt();
            pathPlanner.Interrupt();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Additional
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Call this function directly after the @ref Solve function to further improve the path.
         * @param[in] maxComputationTime The maximum computation time in seconds before stopping the solve iteration loop.
         * @details The path improvement can be interrupted by calling the @ref Interrupt member function.
         */
        void ImprovePath(double maxComputationTime) noexcept { pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, maxComputationTime); }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept { return motionPlanner.PredictMotion(initialStateAndInput, predictionHorizon); }

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
        std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept { return motionPlanner.PredictMotionTrajectory(trajectory, initialStateAndInput, predictionHorizon); }


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
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept { pathPlanner.WriteToFile(file, preString + "pathPlanner."); motionPlanner.WriteToFile(file, preString + "motionPlanner.", staticObstacles); }


    protected:
        mpsv::planner::PathPlannerInput pathPlannerInput;         // The input to the path planner, set during @ref Solve.
        mpsv::planner::PathPlannerOutput pathPlannerOutput;       // The output of the path planner, set during @ref Solve and @ref Improve.
        mpsv::planner::MotionPlannerInput motionPlannerInput;     // The input to the motion planner, set during @ref Solve.
        mpsv::planner::MotionPlannerOutput motionPlannerOutput;   // The output of the motion planner, set during @ref Solve.
        mpsv::planner::PathPlanner pathPlanner;                   // Internal path planning solver.
        mpsv::planner::MotionPlanner motionPlanner;               // Internal motion planning solver.
        mpsv::core::PerformanceCounter executionCounter;          // Performance counter to measure the execution time during @ref Solve.
};


} /* namespace: planner */


} /* namespace: mpsv */

