#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/MotionPlannerTree.hpp>
#include <mpsv/planner/MotionPlannerParameterSet.hpp>
#include <mpsv/planner/MotionPlannerState.hpp>
#include <mpsv/planner/MotionPlannerInput.hpp>
#include <mpsv/planner/MotionPlannerOutput.hpp>
#include <mpsv/planner/MotionPlannerCostMap.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/sampler/UniformBoxSamplerSE2.hpp>
#include <mpsv/sampler/PathSampler.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/PerformanceCounter.hpp>
#include <mpsv/core/Time.hpp>
#include <mpsv/control/VehicleSimulator.hpp>
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/math/Metric.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The motion planning algorithm. Before solving motion planning problems, initialize the planner by calling the @ref Initialize member function.
 * Then apply a parameter set by calling @ref ApplyParameterSet. To solve a planning problem, call the two member functions @ref Prepare and
 * @ref Solve. The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called. If no @ref Solve operation is running,
 * then the next one is to be interrupted.
 */
class MotionPlanner {
    public:
        /**
         * @brief Construct an motion planner and set default values.
         */
        MotionPlanner() noexcept { Terminate(); }

        /**
         * @brief Destroy the motion planner.
         */
        ~MotionPlanner() noexcept { Terminate(); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initialization / Termination
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Initialize the motion planner. This initializes the memory used by the motion planner. Call this member function before calling any other function.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree. A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance. A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
         * @return True if success, false otherwise.
         * @details Call this function before calling any of the other member functions of the algorithm interface (@ref ApplyParameterSet, @ref Prepare, @ref Solve).
         */
        bool Initialize(int16_t maxNumNodes, uint32_t maxNumSamples) noexcept {
            Terminate();
            if((maxNumSamples < 1) || (maxNumNodes < 2)){ // there must be at least one sample and two nodes (root and a sample to be added/removed)
                return false;
            }
            this->maxNumSamples = maxNumSamples;
            idxNeighbors.reserve(maxNumNodes);
            if(!tree.Initialize(maxNumNodes) || !boxSampler.Initialize(maxNumSamples) || !pathSampler.Initialize(maxNumSamples)){
                Terminate();
                return false;
            }
            return true;
        }

        /**
         * @brief Terminate the motion planner. All containers and parameters are cleared. Call this function, if the motion planner is no longer needed.
         */
        void Terminate(void) noexcept {
            interruptFlag = false;
            isSolving = false;
            tree.Terminate();
            state.Clear();
            parameter.Clear();
            regionOfAttraction.Clear();
            vehicleSimulator.ClearController();
            vehicleSimulator.ClearModel();
            boxSampler.Terminate();
            maxNumSamples = 1;
            idxNeighbors.clear();
            idxNeighbors.shrink_to_fit();
            epsPosition = epsAngle = epsDistanceMetric = std::numeric_limits<double>::epsilon();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Algorithm API: ApplyParameterSet, Prepare and Solve
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Apply a parameter set to the motion planner.
         * @param[in] parameterSet The parameter set to be applied.
         * @return True if the input parameter set is valid and parameters have been applied, false otherwise.
         * @details Call this function after a successfull initialization (using the @ref Initialize member function). Do not call this function between @ref Prepare and @ref Solve calls, otherwise the parameters used by
         * both member functions may be inconsistent! If new parameters are to be applied, always apply them BEFORE a prepare-solve-step.
         * @note The tree is cleared (all nodes except the root node are removed).
         */
        bool ApplyParameterSet(const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            if(!parameter.IsValid()){
                return false;
            }
            this->parameter = parameter;
            if(!vehicleSimulator.SetModel(
                                        this->parameter.model.matF,
                                        this->parameter.model.matB,
                                        this->parameter.model.vecTimeconstantsXYN,
                                        this->parameter.model.vecTimeconstantsInput,
                                        this->parameter.model.lowerLimitXYN,
                                        this->parameter.model.upperLimitXYN)){
                return false;
            }
            if(!vehicleSimulator.SetController(this->parameter.motionPlanner.controller.matK,
                                               this->parameter.motionPlanner.controller.maxRadiusX,
                                               this->parameter.motionPlanner.controller.maxRadiusY,
                                               this->parameter.motionPlanner.controller.maxRadiusPsi,
                                               this->parameter.motionPlanner.controller.minRadiusPosition)){
                return false;
            }
            if(!regionOfAttraction.SetParameter(this->parameter.motionPlanner.regionOfAttraction.rangePose,
                                                this->parameter.motionPlanner.regionOfAttraction.rangeUVR,
                                                this->parameter.motionPlanner.regionOfAttraction.rangeXYN)){
                return false;
            }
            state.Clear();
            return true;
        }

        /**
         * @brief Prepare the motion planner before solving the motion planning problem. The planner must be initialized using the @ref Initialize member function and a parameter set has to be applied using the @ref ApplyParameterSet member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[inout] dataIn The input data containing initial and final pose as well as static obstacles and the exported path planning tree. All static obstacles that are outside the area of interest are removed! The input argument is changed.
         * @param[in] forceColdStart True if the initial sample set of the motion planner should be initialized based on the input path only. False if the previous solution path should also be included into the sample set.
         * @details This member function copies the initial state/input and final pose to internal variables. The sampling area and cost look-up table are generated during this call.
         * @note NOTE: The feasibility output only indicates whether the initial pose collides with static obstacles! The actual dynamic feasibility (violation-free trajectory) is checked at the end of the @ref Solve step!
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Prepare(mpsv::planner::MotionPlannerOutput& dataOut, mpsv::planner::MotionPlannerInput& dataIn, bool forceColdStart = false) noexcept {
            // Set initial state/input
            state.Clear(!forceColdStart); // keep latest solution if no cold start is forced
            state.initialStateAndInput = dataIn.initialStateAndInput;
            if(forceColdStart){
                tree.ResetRandomNumberCounter();
            }

            // Initial path is useless if it does not start at initial pose
            if(!InitialPathStartsAtInitialPose(dataIn)){
                dataIn.initialPath.clear();
            }

            // Ensure at least one pose in the path (e.g. initial pose)
            if(dataIn.initialPath.empty()){
                dataIn.initialPath.push_back({dataIn.initialStateAndInput[0], dataIn.initialStateAndInput[1], dataIn.initialStateAndInput[2]});
            }

            // Trim the solution path by limiting the total length of the path and set the new final pose
            TrimPath(dataIn.initialPath, parameter.motionPlanner.maxInputPathLength);
            state.finalPose = dataIn.initialPath.back();

            // Set sampling area and initial sample set, build costmap
            mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn.initialPath);
            SetInitialSampleSet(dataIn.initialPath, dataIn.originOldToNew, forceColdStart);
            SetEps(samplingArea);
            tree.SetCSpaceVolume(pathSampler.GetVolume());
            BuildCostMap(samplingArea, dataIn.staticObstacles);

            // Reset tree and check feasibility
            state.idxSolutionNode = tree.ClearAndSetRoot(mpsv::planner::MotionPlannerTreeNode(state.initialStateAndInput));
            state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(state.initialStateAndInput, state.finalPose, parameter.metric.weightPsi);
            state.goalReached = (state.closestDistanceToGoal < epsDistanceMetric);
            state.isFeasible = !parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, state.initialStateAndInput); // initial pose must not collide for the problem to be feasible

            // Set initial path (initial pose) and trajectory (empty)
            tree.GetPathFromRootToNode(tree.GetRefNode(state.idxSolutionNode).pathBuffer, state.idxSolutionNode); // set reference path
            tree.GetRefNode(state.idxSolutionNode).trajectoryBuffer.clear();
            AssignOutput(dataOut);
        }

        /**
         * @brief Solve the motion planning problem by doing several iterations for an upper computation time limit (the limit is set as parameter when calling this function). The planner must be initialized, parameters must be set and the planner must be prepared before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[in] dataIn The input data containing static obstacles. Only the static obstacles are used. All remaining data has already been processed by the @ref Prepare member function.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0. If the simulation time is too large (more than 0x0000FFFF steps), then the problem is infeasible.
         * @details This member function implements an iteration loop. This loop is stopped, if the @ref interruptFlag is set via @ref Interrupt. The iteration loop is stopped
         * if the upper computation time limit is exceeded or if an error occurs. There are two types of errors: not feasible and out of nodes.
         * The problem is not feasible, if the inital pose already collides with obstacles. In this case no tree is built and the solution trajectory would be
         * an empty set. The second error outOfNodes is set, if no more nodes are available in the memory for more iterations.
         * This only happens if all nodes belong to the reference solution path. The number of nodes is limited. If the tree is full, a random node which must
         * not belong to the reference solution path is removed. If this is not possible, the internal memory of the tree is out of nodes.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::MotionPlannerOutput& dataOut, mpsv::planner::MotionPlannerInput& dataIn, double maxComputationTime, double minTrajectoryDuration = 0.0) noexcept {
            isSolving = true;
            if(state.isFeasible){
                mpsv::core::PerformanceCounter executionCounter, iterationCounter;
                double maxIterationTime = 0.0; // maximum execution time of one iteration
                executionCounter.Start();
                while(!interruptFlag){
                    ++state.numberOfPerformedIterations;
                    iterationCounter.Start();
                    Iterate(dataIn.staticObstacles);
                    maxIterationTime = std::max(maxIterationTime, iterationCounter.TimeToStart());
                    if(state.outOfNodes || ((maxIterationTime + executionCounter.TimeToStart()) >= maxComputationTime)){
                        break;
                    }
                }
                BuildFinalTrajectory(dataIn.staticObstacles, minTrajectoryDuration);
            }
            AssignOutput(dataOut);
            isSolving = false;
            interruptFlag = false;
        }

        /**
         * @brief Solve the motion planning problem by doing a fixed number of iterations. The planner must be initialized, parameters must be set and the planner must be prepared before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[in] dataIn The input data containing static obstacles. Only the static obstacles are used. All remaining data has already been processed by the @ref Prepare member function.
         * @param[in] maxIterations The maximum number of iterations to be performed.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0. If the simulation time is too large (more than 0x0000FFFF steps), then the problem is infeasible.
         * @details This member function implements an iteration loop. This loop is stopped before the maximum number of iterations is reached or
         * if the @ref interruptFlag is set via @ref Interrupt. In addition, the iteration loop is stopped before the maximum number of iterations is reached, if an
         * error occurs. There are two types of errors: not feasible and out of nodes. The problem is not feasible, if the inital pose already
         * collides with obstacles. In this case no tree is built and the solution trajectory would be an empty set. The second error outOfNodes
         * is set, if no more nodes are available in the memory for more iterations. This only happens if all nodes belong to the solution
         * reference path. The number of nodes is limited. If the tree is full, a random node which must not belong to the reference solution
         * path is removed. If this is not possible, the internal memory of the tree is out of nodes.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::MotionPlannerOutput& dataOut, mpsv::planner::MotionPlannerInput& dataIn, uint32_t maxIterations, double minTrajectoryDuration = 0.0) noexcept {
            isSolving = true;
            if(state.isFeasible){
                for(uint32_t i = 0; i < maxIterations; ++i){
                    ++state.numberOfPerformedIterations;
                    Iterate(dataIn.staticObstacles);
                    if(interruptFlag || state.outOfNodes){
                        break;
                    }
                }
                BuildFinalTrajectory(dataIn.staticObstacles, minTrajectoryDuration);
            }
            AssignOutput(dataOut);
            isSolving = false;
            interruptFlag = false;
        }

        /**
         * @brief Interrupt a running @ref Solve operation of the motion planner.
         * @note The internal @ref interruptFlag is automatically reset at the end of a @ref Solve operation.
         */
        void Interrupt(void) noexcept { interruptFlag.store(isSolving); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Additional
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept {
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(0.0, std::floor(predictionHorizon / parameter.motionPlanner.sampletime)));
            double simulationTime = static_cast<double>(numSimulationSteps) * parameter.motionPlanner.sampletime;
            std::array<double,9> finalState = vehicleSimulator.PredictMotion(initialStateAndInput, parameter.motionPlanner.sampletime, numSimulationSteps);
            return std::make_tuple(simulationTime, finalState);
        }

        /**
         * @brief Predict the motion trajectory of the vehicle for a given prediction horizon assuming the control input to be constant all the time. At least one simulation step is computed.
         * @param[out] trajectory The prediction trajectory where each entry contains the state and the input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of a sampletime.
         * @return A tuple containing the following values.
         * [0] True if success, false otherwise. If the maximum number of simulation steps (0x0000FFFF) is exceeded, false is returned.
         * [1] The total time that has been simulated.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon) noexcept {
            constexpr uint32_t maxNumSimulationSteps = 0x0000FFFF;
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(1.0, std::floor(predictionHorizon / parameter.motionPlanner.sampletime)));
            bool success = vehicleSimulator.PredictMotionTrajectory(trajectory, initialStateAndInput, parameter.motionPlanner.sampletime, numSimulationSteps, maxNumSimulationSteps);
            double totalSimulationTime = parameter.motionPlanner.sampletime * static_cast<double>(trajectory.size());
            return std::make_tuple(success, totalSimulationTime);
        }


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
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept { RebuildAllTreeTrajectories(staticObstacles); tree.WriteToFile(file, preString + "tree."); parameter.WriteToFile(file, preString + "parameter."); costMap.WriteToFile(file, preString + "costMap."); }


    protected:
        mpsv::planner::MotionPlannerTree tree;                  // Tree for motion planning.
        mpsv::planner::MotionPlannerState state;                // State of the motion planner.
        mpsv::planner::MotionPlannerParameterSet parameter;     // Parameter set for the motion planner.
        mpsv::planner::MotionPlannerCostMap costMap;            // The cost map to be used as additional cost term in the objective function.
        mpsv::sampler::UniformBoxSamplerSE2 boxSampler;         // The box sampler to be used to draw random samples from SE(2).
        mpsv::sampler::PathSampler pathSampler;                 // The path sampler to be used to draw random samples from SE(2).
        uint32_t maxNumSamples;                                 // The maximum number of samples set by @ref Initialize.
        std::vector<int16_t> idxNeighbors;                      // Reserved memory for nearest neighbors.
        mpsv::control::VehicleSimulator vehicleSimulator;       // The vehicle simulator to be used for generating dynamically feasible trajectories.
        mpsv::control::RegionOfAttraction regionOfAttraction;   // The region of attraction to be used to check if a state enters the region around a pose.
        std::atomic<bool> interruptFlag;                        // If set to true, no more iterations are performed and the @ref Solve member function returns.
        std::atomic<bool> isSolving;                            // True if the motion planner performs a @ref Solve step, false otherwise.
        double epsPosition;                                     // Eps (numeric threshold) for position.
        double epsAngle;                                        // Eps (numeric threshold) for angle.
        double epsDistanceMetric;                               // Eps (numeric threshold) for distance metric.

        /**
         * @brief Build the final trajectory. This member function calculates the reference path and the trajectory for the current solution node.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds.
         * @details If the trajectory is infeasible, then the isFeasible flag of the @ref state is set.
         */
        void BuildFinalTrajectory(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, double minTrajectoryDuration){
            uint32_t minNumSteps = static_cast<uint32_t>(std::ceil(std::max(0.0, minTrajectoryDuration) / parameter.motionPlanner.sampletime));
            tree.GetPathFromRootToNode(tree.GetRefNode(state.idxSolutionNode).pathBuffer, state.idxSolutionNode);
            state.isFeasible = PathFromRootIsFeasible(tree.GetRefNode(state.idxSolutionNode).pathBuffer, staticObstacles, tree.GetRefNode(state.idxSolutionNode).trajectoryBuffer, minNumSteps);
        }

        /**
         * @brief Assign output data based on the internal @ref state and the @ref tree.
         * @param[out] dataOut Output data to be assigned.
         * @details The corresponding path and trajectory buffer of the solution node are swapped with the path and trajectory outputs, respectively.
         */
        void AssignOutput(mpsv::planner::MotionPlannerOutput& dataOut) noexcept {
            dataOut.goalReached = state.goalReached;
            dataOut.isFeasible = state.isFeasible;
            dataOut.outOfNodes = state.outOfNodes;
            dataOut.numberOfPerformedIterations = state.numberOfPerformedIterations;
            dataOut.cost = tree.GetRefNode(state.idxSolutionNode).cost;
            dataOut.referencePath.swap(tree.GetRefNode(state.idxSolutionNode).pathBuffer);
            dataOut.trajectory.swap(tree.GetRefNode(state.idxSolutionNode).trajectoryBuffer);
            dataOut.sampletime = parameter.motionPlanner.sampletime;
            dataOut.timestampOfComputationUTC = mpsv::core::GetTimestampUTC();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Prepare helper functions
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Check whether the initial path starts at the initial state.
         * @param[in] dataIn Input data to the motion planner containing the initial path as well as the initial state.
         * @return True if the initial state corresponds to the start of the initial path, false otherwise.
         */
        bool InitialPathStartsAtInitialPose(mpsv::planner::MotionPlannerInput& dataIn) noexcept {
            if(dataIn.initialPath.empty()){
                return false;
            }
            std::array<double,3> delta = mpsv::math::PoseDifference(dataIn.initialPath[0], dataIn.initialStateAndInput);
            if(std::fabs(delta[0] >= epsPosition) || std::fabs(delta[1] >= epsPosition) || std::fabs(delta[2] >= epsAngle)){
                return false;
            }
            return true;
        }

        /**
         * @brief Trim the path to ensure a maximum path length (x,y only).
         * @param[inout] path The path to be trimmed, each pose is given by {x,y,psi}. The path may be trimmed to an interpolated pose.
         * @param[in] maximumPathLength Maximum path length along the path that defines the point at which the path is trimmed.
         * @details The path is trimmed along the path without using angle information.
         */
        void TrimPath(std::vector<std::array<double,3>>& path, double maximumPathLength) noexcept {
            size_t N = path.size();
            if(!N) return;
            double dx, dy, L, lambda;
            for(size_t n = 1; n < N; ++n){
                dx = path[n][0] - path[n - 1][0];
                dy = path[n][1] - path[n - 1][1];
                L = std::sqrt(dx*dx + dy*dy);
                if(L < maximumPathLength){
                    maximumPathLength -= L;
                    continue;
                }
                if(L > 0.0){
                    lambda = maximumPathLength / L;
                    path[n][0] = path[n - 1][0] + lambda * dx;
                    path[n][1] = path[n - 1][1] + lambda * dy;
                    path[n][2] = mpsv::math::SymmetricalAngle(path[n - 1][2] + lambda * mpsv::math::SymmetricalAngle(path[n][2] - path[n - 1][2]));
                }
                path.resize(n + 1);
                break;
            }
        }

        /**
         * @brief Set the initial sample set for motion planning.
         * @param[in] initialPath The initial path (trimmed) from path planning to be included in the sample set.
         * @param[in] originOldToNew Translation between two consecutive problems. If the origin of the previous problem is different to the origin of the new problem, then the old solution must be
         * transformed to be used as initial sample set in the new problem. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
         * @param[in] forceColdStart True if the initial sample set of the motion planner should be initialized based on the input path only. False if the previous solution path should also be included into the sample set.
         * @details The sample set contains the old solution path followed by the initial path (excluding its initial pose). The use of the old solution path is advantageous because the it may have a higher
         * probability of being collision free compared to the initial path of the path planner.
         */
        void SetInitialSampleSet(const std::vector<std::array<double,3>>& initialPath, std::array<double,2> originOldToNew, bool forceColdStart) noexcept {
            state.initialSamplesReversed.clear();
            if(!forceColdStart){
                // Transform tree to new origin and use old solution path as initial sample set
                tree.TransformToNewOrigin(originOldToNew);
                tree.GetPathFromRootToNode(tree.GetRefNode(state.idxSolutionNode).pathBuffer, state.idxSolutionNode);

                // Use old solution path as initial sample set if those poses are inside the new path sampling area
                for(auto&& pose : tree.GetRefNode(state.idxSolutionNode).pathBuffer){
                    if(pathSampler.IsInside(pose)){
                        state.initialSamplesReversed.push_back(pose);
                    }
                }
            }

            // Insert initial path to sample set (exclude first pose, that would be the initial pose)
            state.initialSamplesReversed.insert(state.initialSamplesReversed.end(), initialPath.begin() + 1, initialPath.end());

            // Reverse the order such that the end of the container contains the first poses to be sampled
            std::reverse(state.initialSamplesReversed.begin(), state.initialSamplesReversed.end());
        }

        /**
         * @brief Set the sampling area based on a given path. The area corresponds to the minimum area oriented box enclosing all points of the path, extended by the position sampling range.
         * @param[in] path The path that should be contained in the sampling area.
         * @return The oriented box representing the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(const std::vector<std::array<double,3>>& path) noexcept {
            mpsv::geometry::OrientedBox box;
            constexpr double sqrt2 = std::sqrt(2.0);
            pathSampler.Reset(path, parameter.motionPlanner.samplingRangePosition, parameter.motionPlanner.samplingRangeAngle, parameter.metric.weightPsi);
            box.CreateFromPointCloud(path);
            box.Extend(parameter.motionPlanner.samplingRangePosition * sqrt2);
            boxSampler.SetSize(box);
            return box;
        }

        /**
         * @brief Set eps values (numeric thresholds).
         * @param[in] samplingArea The current sampling area.
         */
        void SetEps(const mpsv::geometry::OrientedBox& samplingArea) noexcept {
            std::array<double,2> dimension = samplingArea.GetDimension();
            double maxDimension = std::max(dimension[0], dimension[1]);

            // Eps for position and angle
            epsPosition = std::numeric_limits<double>::epsilon() * std::max(1.0, maxDimension);
            epsAngle = std::numeric_limits<double>::epsilon() * 6.28318530717959;

            // Eps for distance metric sqrt(dx^2 + dy^2 + (w*dpsi)^2)
            double epsWeightedAngle = std::max(1.0, parameter.metric.weightPsi) * epsAngle;
            epsDistanceMetric = std::sqrt(2.0*epsPosition*epsPosition + epsWeightedAngle*epsWeightedAngle);
        }

        /**
         * @brief Build the 2D-look-up table of the costmap.
         * @param[in] samplingArea The sampling area.
         * @param[in] staticObstacles Static obstacles to be considered for building the map.
         */
        void BuildCostMap(mpsv::geometry::OrientedBox samplingArea, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // The range must be at least the range of the sampling box. This box is further extended by the maximum distance of skeletal points for
            // cost map evaluation and by some safety threshold of two more cells.
            constexpr size_t maxNumCells = 125000000; // limit to 1 GB
            double maxSekeletalPointDistance = 0.0;
            for(auto&& p : parameter.geometry.skeletalPoints){
                maxSekeletalPointDistance = std::max(maxSekeletalPointDistance, std::sqrt(p[0]*p[0] + p[1]*p[1]));
            }
            double additionalBoxExtension = maxSekeletalPointDistance + 2.0 * parameter.costMap.resolution;
            samplingArea.Extend(additionalBoxExtension);
            mpsv::planner::MotionPlannerCostMapArgument args(parameter.costMap.distanceScale, parameter.costMap.distanceDecay, staticObstacles);
            costMap.Build(samplingArea, parameter.costMap.resolution, parameter.costMap.modBreakpoints, args, maxNumCells);
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Iteration helper functions
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Do one iteration of the RRT* motion planning algorithm.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         */
        void Iterate(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Fixed-Node RRT*: ensure that tree is not full by possibly removing a random node
            if((state.outOfNodes = !EnsureNonFullTree())){
                return;
            }

            // Sampling: generate random sample
            mpsv::planner::MotionPlannerTreeNode randomSample = Sampling();

            // Vertex Selection: select nearest node
            int16_t idxNearest = tree.FindNearestNode(randomSample, parameter.metric.weightPsi);

            // Local Planning: steer from nearest sample to random sample
            auto [success, newSample] = LocalPlanning(idxNearest, randomSample, staticObstacles);
            if(!success){
                return;
            }

            // Get near vertices (neighbors of newSample)
            double radius = tree.GetOptimalRadius();
            if(tree.FindNearestNeighbors(idxNeighbors, newSample, radius, parameter.metric.weightPsi, epsDistanceMetric)){
                return; // if the value of the new sample already exists, return
            }

            // Find best parent node from the neighborhood (cost of newSample is updated)
            int16_t idxBestParent = FindBestParent(newSample, idxNearest, staticObstacles);

            // Insert into tree: add new sample to the tree
            int16_t idxAddedSample = tree.Add(idxBestParent, newSample);

            // Rewire edges in the neighborhood around the added sample
            Rewire(idxAddedSample, staticObstacles);

            // Check if the new sample indicates a solution
            double distanceToGoal = mpsv::math::DistanceMetricSE2(tree.GetRefNode(idxAddedSample).pose, state.finalPose, parameter.metric.weightPsi);
            if(distanceToGoal < epsDistanceMetric){
                state.goalReached = true;
                state.closestDistanceToGoal = distanceToGoal;
                state.idxSolutionNode = idxAddedSample;
            }
            if(distanceToGoal < state.closestDistanceToGoal){
                state.closestDistanceToGoal = distanceToGoal;
                state.idxSolutionNode = idxAddedSample;
            }
        }

        /**
         * @brief Ensure that the tree is not full. A random leaf is removed if the tree is full.
         * @return True if tree is not full and iterations may continue, false otherwise.
         */
        bool EnsureNonFullTree(void) noexcept {
            if(tree.IsFull()){
                if(!tree.RemoveRandomLeaf(state.idxSolutionNode)){ // current solution node must not be removed
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Generate a random sample.
         * @return The random sample.
         */
        mpsv::planner::MotionPlannerTreeNode Sampling(void) noexcept {
            mpsv::planner::MotionPlannerTreeNode randomSample;
            if(state.initialSamplesReversed.size()){
                randomSample.pose = state.initialSamplesReversed.back();
                state.initialSamplesReversed.pop_back();
            }
            else{
                if(state.goalReached || state.goalSampleIteration){
                    randomSample.pose = pathSampler.Sample(state.sampleIteration);
                    state.sampleIteration = (state.sampleIteration + 1) % maxNumSamples;
                }
                else{
                    randomSample.pose = state.finalPose;
                }
                state.goalSampleIteration = (state.goalSampleIteration + 1) % parameter.motionPlanner.periodGoalSampling;
            }
            return randomSample;
        }

        /**
         * @brief Perform local planning from a source node to a destination node and create a resulting node.
         * @param[in] idxSource The index to the source node from which to start the local planning.
         * @param[in] destination The destination node to which to steer to from the source node.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         * @return A tuple containing <success, newSample>.
         * <0> True if local planning succeeded (e.g. no collision), false otherwise.
         * <1> The resulting sample of local planning.
         */
        std::tuple<bool, mpsv::planner::MotionPlannerTreeNode> LocalPlanning(const int16_t idxSource, const mpsv::planner::MotionPlannerTreeNode& destination, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            mpsv::planner::MotionPlannerTreeNode& source = tree.GetRefNode(idxSource);

            // Resulting output tuple containing success and newSample
            std::tuple<bool, mpsv::planner::MotionPlannerTreeNode> result;
            bool& success = std::get<0>(result);
            mpsv::planner::MotionPlannerTreeNode& newSample = std::get<1>(result);
            success = false;

            // Direction from source to destination
            std::array<double, 3> direction = mpsv::math::PoseDifference(destination.pose, source.pose);

            // Limit direction to maximum step size
            double z = parameter.metric.weightPsi * direction[2];
            double r = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + z*z);
            double radius = tree.GetOptimalRadius();
            bool stepLimited = false;
            if(r > radius){
                stepLimited = true;
                radius /= r;
                direction[0] *= radius;
                direction[1] *= radius;
                direction[2] *= radius;
            }

            // Do not allow zero movement
            if((std::fabs(direction[0]) <= epsPosition) && (std::fabs(direction[1]) <= epsPosition) && (std::fabs(direction[2]) <= epsAngle)){
                return result;
            }

            // Resulting node is source + limited direction
            newSample.pose[0] = source.pose[0] + direction[0];
            newSample.pose[1] = source.pose[1] + direction[1];
            newSample.pose[2] = mpsv::math::SymmetricalAngle(source.pose[2] + direction[2]);

            // Check feasibility and calculate cost
            std::vector<std::array<double,3>>& path = tree.GetRefNode(idxSource).pathBuffer;
            tree.GetPathFromRootToNode(path, idxSource);
            path.push_back(newSample.pose);
            if(!PathFromRootIsFeasible(path, staticObstacles, tree.GetRefNode(idxSource).trajectoryBuffer)){
                return result;
            }
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, parameter.metric.weightPsi);
            double costHeading = mpsv::math::SwayMetric(source.pose, newSample.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
            double costClearance = costMap.CostAlongLine(source.pose, newSample.pose, parameter.geometry.skeletalPoints);
            newSample.cost = source.cost + costDistance + costHeading + costClearance;

            // If the step was limited during goal sampling, then continue goal sampling in the next iteration
            state.goalSampleIteration -= static_cast<uint32_t>(stepLimited && (1 == state.goalSampleIteration));
            success = true;
            return result;
        }

        /**
         * @brief Find best parent for a new sample.
         * @param[inout] newSample The new sample from local planning for which to find the best parent. The cost of this sample is updated.
         * @param[in] idxNearest Index to the nearest sample of the newSample which is the current parent from local planning.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         * @return Index to the best parent node such that the cost for the new sample becomes minimal and the edge from that parent to the new sample is feasible.
         */
        int16_t FindBestParent(mpsv::planner::MotionPlannerTreeNode& newSample, int16_t idxNearest, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            int16_t idxBestParent = idxNearest;
            for(auto&& index : idxNeighbors){
                // Ignore the nearest node which is already the parent for the new node, check all others
                if(idxNearest == index){
                    continue;
                }

                // Check if cost reduction is actually possible
                mpsv::planner::MotionPlannerTreeNode& possibleParent = tree.GetRefNode(index);
                double resultCost = possibleParent.cost;
                resultCost += mpsv::math::DistanceMetricSE2(possibleParent.pose, newSample.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(possibleParent.pose, newSample.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                if((resultCost + epsDistanceMetric) < newSample.cost){
                    resultCost += costMap.CostAlongLine(possibleParent.pose, newSample.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + epsDistanceMetric) < newSample.cost){
                        // Check if a better connection is feasible and update cost and parent index
                        std::vector<std::array<double,3>>& path = tree.GetRefNode(index).pathBuffer;
                        tree.GetPathFromRootToNode(path, index);
                        path.push_back(newSample.pose);
                        if(!PathFromRootIsFeasible(path, staticObstacles, tree.GetRefNode(index).trajectoryBuffer)){
                            continue;
                        }
                        idxBestParent = index;
                        newSample.cost = resultCost;
                    }
                }
            }
            return idxBestParent;
        }

        /**
         * @brief Rewire the edges of the tree: check if the cost from the added sample to any sample in the neighborhood can be reduced
         * when steering from the added sample to one in the neighborhood. Or: for which sample can the added sample be a better parent.
         * @param[in] idxAddedSample Index to the added sample.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         */
        void Rewire(int16_t idxAddedSample, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Check if the cost to anyone in the neighborhood can be reduced when steering from the best new sample to the neighborhood node
            // Or: for which node can the current best new sample be a better parent
            mpsv::planner::MotionPlannerTreeNode& addedSample = tree.GetRefNode(idxAddedSample);
            for(auto&& index : idxNeighbors){
                // Cannot be a better parent for its own parent or for the root
                if((tree.GetParentIndex(idxAddedSample) == index) || (tree.GetParentIndex(idxAddedSample) < 0)){
                    continue;
                }

                // Check if cost reduction is actually possible
                mpsv::planner::MotionPlannerTreeNode& destination = tree.GetRefNode(index);
                double resultCost = addedSample.cost;
                resultCost += mpsv::math::DistanceMetricSE2(addedSample.pose, destination.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(addedSample.pose, destination.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                if((resultCost + epsDistanceMetric) < destination.cost){
                    resultCost += costMap.CostAlongLine(addedSample.pose, destination.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + epsDistanceMetric) < destination.cost){
                        // get all leaf nodes from [index]
                        std::vector<int16_t> idxLeafNodes = tree.GetAllLeafNodes(index);

                        // change parent of [index] temporarily to idxAddedSample
                        int16_t idxOriginalParent = tree.GetParentIndex(index);
                        tree.GetRefNode(index).idxParent = idxAddedSample;

                        // if original parent only has one child (index) it would become a leaf node
                        if(1 == tree.GetRefNode(idxOriginalParent).idxChilds.size()){
                            idxLeafNodes.push_back(idxOriginalParent);
                        }

                        // check all paths from all leafs to the root
                        bool anythingInfeasible = false;
                        #ifndef MPSV_DONT_USE_OMP
                        #pragma omp parallel for shared(anythingInfeasible, tree, staticObstacles)
                        for(auto&& idxLeaf : idxLeafNodes){
                            std::vector<std::array<double,3>>& path = tree.GetRefNode(idxLeaf).pathBuffer;
                            tree.GetPathFromRootToNode(path, idxLeaf);
                            if(!PathFromRootIsFeasible(path, staticObstacles, tree.GetRefNode(idxLeaf).trajectoryBuffer)){
                                anythingInfeasible = true;
                            }
                        }
                        #else
                        for(auto&& idxLeaf : idxLeafNodes){
                            std::vector<std::array<double,3>>& path = tree.GetRefNode(idxLeaf).pathBuffer;
                            tree.GetPathFromRootToNode(path, idxLeaf);
                            if(!PathFromRootIsFeasible(path, staticObstacles, tree.GetRefNode(idxLeaf).trajectoryBuffer)){
                                anythingInfeasible = true;
                                break;
                            }
                        }
                        #endif

                        // reset parent of [index] to original parent
                        tree.GetRefNode(index).idxParent = idxOriginalParent;

                        // at least one infeasible connection means fail: rewire not saftely possible
                        if(anythingInfeasible){
                            continue;
                        }
                        double deltaCost = resultCost - destination.cost;
                        tree.Rewire(index, idxAddedSample, deltaCost);
                    }
                }
            }
        }

        /**
         * @brief Check whether a given path starting from the root node is feasible. The initial state corresponding to the root node is given in @ref state.
         * @param[in] path The path starting from the root node. This path is used for trajectory generation using a path controller and a nonlinear model of the vehicle.
         * @param[in] staticObstacles Static obstacles to be considered for collision checks.
         * @param[inout] trajectoryBuffer A buffer to be used for trajectory generation.
         * @param[in] minNumSimulationSteps The minimum number of simulation steps to be computed during path exploration (must be at least 1 and at most 0x0000FFFF), defaults to 1.
         * @return True if path is dynamically feasible, e.g. the resulting trajectory is feasible along that path, false otherwise.
         */
        bool PathFromRootIsFeasible(const std::vector<std::array<double,3>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, std::vector<std::array<double,12>>& trajectoryBuffer, uint32_t minNumSimulationSteps = 1) noexcept {
            uint32_t numSteps = vehicleSimulator.ExplorePathToEnd(trajectoryBuffer, state.initialStateAndInput, path, regionOfAttraction, staticObstacles, parameter.motionPlanner.sampletime, minNumSimulationSteps);
            if(numSteps >= 0x0000FFFF){
                return false;
            }
            if(parameter.geometry.vehicleShape.CheckCollisionCurve(staticObstacles, trajectoryBuffer, parameter.geometry.collisionCheckMaxPositionDeviation, parameter.geometry.collisionCheckMaxAngleDeviation)){
                return false;
            }
            return true;
        }

        /**
         * @brief Rebuild all trajectories of the tree. For the solution node, as well as for all leaf nodes, trajectories are generated. All remaining node trajectories are cleared.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration.
         */
        void RebuildAllTreeTrajectories(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            const std::vector<int16_t>& indices = tree.GetAllNodeIndices();
            for(auto&& i : indices){
                tree.GetRefNode(i).trajectoryBuffer.clear();
                if(tree.GetRefNode(i).idxChilds.empty() || (i == state.idxSolutionNode)){
                    GenerateTrajectoryFromRoot(tree.GetRefNode(i).trajectoryBuffer, i, staticObstacles);
                }
            }
        }

        /**
         * @brief Generate trajectory from the root node to a specific node.
         * @param[out] trajectory Final trajectory from the root node to the specified node. Each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted in the trajectory.
         * @param[in] idxNode Index to the node for which to generate the trajectory.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration.
         */
        void GenerateTrajectoryFromRoot(std::vector<std::array<double,12>>& trajectory, int16_t idxNode, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Get the path from the root
            std::vector<std::array<double,3>>& pathBuffer = tree.GetRefNode(idxNode).pathBuffer;
            tree.GetPathFromRootToNode(pathBuffer, idxNode);

            // Simulate trajectory
            (void) vehicleSimulator.ExplorePathToEnd(trajectory, state.initialStateAndInput, pathBuffer, regionOfAttraction, staticObstacles, parameter.motionPlanner.sampletime);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

