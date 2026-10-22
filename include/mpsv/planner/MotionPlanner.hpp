#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/MotionPlannerTree.hpp>
#include <mpsv/planner/MotionPlannerParameterSet.hpp>
#include <mpsv/planner/MotionPlannerState.hpp>
#include <mpsv/planner/MotionPlannerInput.hpp>
#include <mpsv/planner/MotionPlannerOutput.hpp>
#include <mpsv/structures/CostMap.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/sampler/UniformBoxSamplerSE2.hpp>
#include <mpsv/sampler/PathSampler.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/runtime/Time.hpp>
#include <mpsv/common/ErrorCode.hpp>
#include <mpsv/control/VehicleSimulator.hpp>
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/math/Metric.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The motion planning algorithm. Before solving motion planning problems, initialize the planner by calling the @ref Initialize member function.
 * Then apply a parameter set by calling @ref ApplyParameterSet. The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called.
 */
class MotionPlanner {
    private:
        using Node = mpsv::planner::MotionPlannerTreeNode;

    public:
        /**
         * @brief Construct an motion planner and set default values.
         */
        MotionPlanner() noexcept { Initialize(1, 1); }

        /**
         * @brief Destroy the motion planner.
         */
        ~MotionPlanner() noexcept { Terminate(); }

        /**
         * @brief Initialize or re-initialize the motion planner.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance.
         */
        void Initialize(int16_t maxNumNodes, uint32_t maxNumSamples) noexcept {
            Terminate();
            m_idxNeighbors.reserve(maxNumNodes);
            m_tree.Clear(maxNumNodes);
            m_pathSampler.Initialize(maxNumSamples);
        }

        /**
         * @brief Terminate the motion planner. All containers and parameters are cleared. Call this function, if the motion planner is no longer needed.
         */
        void Terminate(void) noexcept {
            m_runSolve = false;
            m_tree.Clear();
            m_costMap.Clear();
            m_state.Clear();
            m_parameter.Clear();
            m_regionOfAttraction.Clear();
            m_vehicleSimulator.ClearController();
            m_vehicleSimulator.ClearModel();
            m_idxNeighbors.clear();
            m_idxNeighbors.shrink_to_fit();
            m_epsDistanceMetric = std::numeric_limits<double>::epsilon();
        }

        /**
         * @brief Apply a parameter set to the motion planner.
         * @param[in] parameter The parameter set to be applied.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code ApplyParameterSet(mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            error_code e;
            if(error_code::NONE != (e = parameter.IsValid())){
                return e;
            }
            m_parameter = parameter;
            if(error_code::NONE != (e = m_vehicleSimulator.SetModel(parameter.model))){
                return e;
            }
            if(error_code::NONE != (e = m_vehicleSimulator.SetController(parameter.motionPlanner.controller))){
                return e;
            }
            if(error_code::NONE != (e = m_regionOfAttraction.SetParameter(parameter.motionPlanner.regionOfAttraction))){
                return e;
            }
            m_state.Clear();
            return error_code::NONE;
        }

        /**
         * @brief Solve the motion planning problem by doing several iterations for an upper computation time limit. Parameters must be set before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[inout] dataIn The input data that represents the motion planning problem to be solved. The data may be modified by this member function, e.g. the static obstacles may be filtered to remove obstacles outside the area of interest.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0. If the simulation time is too large (more than 0x0000FFFF steps), then the problem is infeasible.
         * @param[in] forceColdStart True if the initial sample set of the motion planner should be initialized based on the input path only. False if the previous solution path should also be included into the sample set.
         * @details This member function implements an iteration loop that is termimated if @ref Interrupt is called.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::MotionPlannerOutput& dataOut, mpsv::planner::MotionPlannerInput& dataIn, double maxComputationTime, double minTrajectoryDuration = 0.0, bool forceColdStart = false) noexcept {
            m_runSolve = true;
            mpsv::runtime::PerformanceCounter executionCounter;

            // Prepare
            // Set initial state/input
            m_state.Clear(!forceColdStart); // keep latest solution if no cold start is forced
            m_state.initialStateAndInput = dataIn.initialStateAndInput;

            // Initial path is useless if it does not start at initial pose
            if(!InitialPathStartsAtInitialPose(dataIn)){
                dataIn.initialPath.clear();
            }

            // Ensure at least one pose in the path (e.g. initial pose)
            if(dataIn.initialPath.empty()){
                dataIn.initialPath.push_back({dataIn.initialStateAndInput[0], dataIn.initialStateAndInput[1], dataIn.initialStateAndInput[2]});
            }

            // Trim the solution path by limiting the total length of the path and set the new final pose
            TrimPath(dataIn.initialPath, m_parameter.motionPlanner.maxInputPathLength);
            m_state.finalPose = dataIn.initialPath.back();

            // Set sampling area and initial sample set, build costmap
            mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn.initialPath);
            SetInitialSampleSet(dataIn.initialPath, dataIn.originOldToNew, forceColdStart);
            SetNumericLimits(samplingArea);
            m_tree.SetCSpaceVolume(m_pathSampler.GetVolume());
            BuildCostMap(samplingArea, dataIn.staticObstacles);

            // Reset tree and check feasibility
            m_state.idxSolutionNode = m_tree.Reset(m_state.initialStateAndInput);
            m_state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(m_state.initialStateAndInput, m_state.finalPose, m_parameter.metric.weightPsi);
            m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
            m_state.isFeasible = !m_parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, m_state.initialStateAndInput); // initial pose must not collide for the problem to be feasible

            // Set initial path (initial pose) and trajectory (empty)
            m_tree.GetPathFromRootToNode(m_tree[m_state.idxSolutionNode].pathBuffer, m_state.idxSolutionNode); // set reference path
            m_tree[m_state.idxSolutionNode].trajectoryBuffer.clear();

            // Solve
            if(m_state.isFeasible){
                double maxIterationTime = 0.0; // maximum execution time of one iteration
                while(m_runSolve){
                    ++m_state.numberOfPerformedIterations;
                    mpsv::runtime::PerformanceCounter iterationCounter;
                    Iterate(dataIn.staticObstacles);
                    maxIterationTime = std::max(maxIterationTime, iterationCounter.TimeToStart());
                    if(m_state.outOfNodes || ((maxIterationTime + executionCounter.TimeToStart()) >= maxComputationTime)){
                        break;
                    }
                }
                BuildFinalTrajectory(dataIn.staticObstacles, minTrajectoryDuration);
            }
            AssignOutput(dataOut);
            m_runSolve = false;
        }

        /**
         * @brief Interrupt a running @ref Solve operation of the motion planner.
         */
        void Interrupt(void) noexcept { m_runSolve = false; }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of zero.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept {
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(0.0, std::floor(predictionHorizon / m_parameter.motionPlanner.sampletime)));
            double simulationTime = static_cast<double>(numSimulationSteps) * m_parameter.motionPlanner.sampletime;
            std::array<double,9> finalState = m_vehicleSimulator.PredictMotion(initialStateAndInput, m_parameter.motionPlanner.sampletime, numSimulationSteps);
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
        [[nodiscard]] std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon) const noexcept {
            constexpr uint32_t maxNumSimulationSteps = 0x0000FFFF;
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(1.0, std::floor(predictionHorizon / m_parameter.motionPlanner.sampletime)));
            bool success = m_vehicleSimulator.PredictMotionTrajectory(trajectory, initialStateAndInput, m_parameter.motionPlanner.sampletime, numSimulationSteps, maxNumSimulationSteps);
            double totalSimulationTime = m_parameter.motionPlanner.sampletime * static_cast<double>(trajectory.size());
            return std::make_tuple(success, totalSimulationTime);
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration. All trajectories of the tree are rebuilt. Otherwise the trajectories from tree nodes are undefined.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept { RebuildAllTreeTrajectories(staticObstacles); m_tree.WriteToFile(file, preString + "tree."); m_parameter.WriteToFile(file, preString + "parameter."); m_costMap.WriteToFile(file, preString + "costMap."); }

    protected:
        mpsv::planner::MotionPlannerTree m_tree;                              // Tree for motion planning.
        mpsv::planner::MotionPlannerState m_state;                            // State of the motion planner.
        mpsv::planner::MotionPlannerParameterSet m_parameter;                 // Parameter set for the motion planner.
        mpsv::structures::CostMap m_costMap;                                  // The cost map to be used as additional cost term in the objective function.
        mpsv::sampler::PathSampler m_pathSampler;                             // The path sampler to be used to draw random samples from SE(2).
        std::vector<int16_t> m_idxNeighbors;                                  // Reserved memory for nearest neighbors.
        mpsv::control::VehicleSimulator m_vehicleSimulator;                   // The vehicle simulator to be used for generating dynamically feasible trajectories.
        mpsv::control::RegionOfAttraction m_regionOfAttraction;               // The region of attraction to be used to check if a state enters the region around a pose.
        std::atomic<bool> m_runSolve{false};                                  // True if a solve operation is running, false if not or is requested to terminate.
        double m_epsDistanceMetric{std::numeric_limits<double>::epsilon()};   // Eps (numeric threshold) for distance metric.

        /**
         * @brief Build the final trajectory. This member function calculates the reference path and the trajectory for the current solution node.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds.
         * @details If the trajectory is infeasible, then the isFeasible flag of the @ref m_state is set.
         */
        void BuildFinalTrajectory(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, double minTrajectoryDuration){
            uint32_t minNumSteps = static_cast<uint32_t>(std::ceil(std::max(0.0, minTrajectoryDuration) / m_parameter.motionPlanner.sampletime));
            m_tree.GetPathFromRootToNode(m_tree[m_state.idxSolutionNode].pathBuffer, m_state.idxSolutionNode);
            m_state.isFeasible = PathFromRootIsFeasible(m_tree[m_state.idxSolutionNode].pathBuffer, staticObstacles, m_tree[m_state.idxSolutionNode].trajectoryBuffer, minNumSteps);
        }

        /**
         * @brief Assign output data based on the internal @ref m_state and the @ref m_tree.
         * @param[out] dataOut Output data to be assigned.
         * @details The corresponding path and trajectory buffer of the solution node are swapped with the path and trajectory outputs, respectively.
         */
        void AssignOutput(mpsv::planner::MotionPlannerOutput& dataOut) noexcept {
            dataOut.goalReached = m_state.goalReached;
            dataOut.isFeasible = m_state.isFeasible;
            dataOut.outOfNodes = m_state.outOfNodes;
            dataOut.numberOfPerformedIterations = m_state.numberOfPerformedIterations;
            dataOut.cost = m_tree[m_state.idxSolutionNode].cost;
            dataOut.referencePath.swap(m_tree[m_state.idxSolutionNode].pathBuffer);
            dataOut.trajectory.swap(m_tree[m_state.idxSolutionNode].trajectoryBuffer);
            dataOut.sampletime = m_parameter.motionPlanner.sampletime;
            dataOut.timestampOfComputationUTC = mpsv::runtime::GetTimestampUTC();
        }

        /**
         * @brief Check whether the initial path starts at the initial state.
         * @param[in] dataIn Input data to the motion planner containing the initial path as well as the initial state.
         * @return True if the initial state corresponds to the start of the initial path, false otherwise.
         */
        [[nodiscard]] bool InitialPathStartsAtInitialPose(mpsv::planner::MotionPlannerInput& dataIn) noexcept {
            if(dataIn.initialPath.empty()){
                return false;
            }
            double distance = mpsv::math::DistanceMetricSE2(dataIn.initialPath[0], dataIn.initialStateAndInput, m_parameter.metric.weightPsi);
            if(std::fabs(distance) >= m_epsDistanceMetric){
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
            m_state.initialSamplesReversed.clear();
            if(!forceColdStart){
                // Transform tree to new origin and use old solution path as initial sample set
                m_tree.TransformToNewOrigin(originOldToNew);
                m_tree.GetPathFromRootToNode(m_tree[m_state.idxSolutionNode].pathBuffer, m_state.idxSolutionNode);

                // Use old solution path as initial sample set if those poses are inside the new path sampling area
                for(auto&& pose : m_tree[m_state.idxSolutionNode].pathBuffer){
                    if(m_pathSampler.IsInside(pose)){
                        m_state.initialSamplesReversed.push_back(pose);
                    }
                }
            }

            // Insert initial path to sample set (exclude first pose, that would be the initial pose)
            m_state.initialSamplesReversed.insert(m_state.initialSamplesReversed.end(), initialPath.begin() + 1, initialPath.end());

            // Reverse the order such that the end of the container contains the first poses to be sampled
            std::reverse(m_state.initialSamplesReversed.begin(), m_state.initialSamplesReversed.end());
        }

        /**
         * @brief Set the sampling area based on a given path. The area corresponds to the minimum area oriented box enclosing all points of the path, extended by the position sampling range.
         * @param[in] path The path that should be contained in the sampling area.
         * @return The oriented box representing the rectangular hull of the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(const std::vector<std::array<double,3>>& path) noexcept {
            mpsv::geometry::OrientedBox box;
            const double sqrt2 = std::sqrt(2.0);
            m_pathSampler.Reset(path, m_parameter.motionPlanner.samplingRangePosition, m_parameter.motionPlanner.samplingRangeAngle, m_parameter.metric.weightPsi);
            box.CreateFromPointCloud(path);
            box.Extend(m_parameter.motionPlanner.samplingRangePosition * sqrt2);
            return box;
        }

        /**
         * @brief Set absolute numeric limits. This limit is used to check for equal poses.
         * @param[in] samplingArea The current sampling area.
         */
        void SetNumericLimits(const mpsv::geometry::OrientedBox& samplingArea) noexcept {
            std::array<double,2> dimension = samplingArea.GetDimension();
            double maxDimension = std::max(dimension[0], dimension[1]);

            // Eps for position and angle
            double epsPosition = std::numeric_limits<double>::epsilon() * std::max(1.0, maxDimension);
            double epsAngle = std::numeric_limits<double>::epsilon() * 6.28318530717959;

            // Eps for distance metric sqrt(dx^2 + dy^2 + (w*dpsi)^2)
            double epsWeightedAngle = std::max(1.0, m_parameter.metric.weightPsi) * epsAngle;
            m_epsDistanceMetric = std::sqrt(2.0*epsPosition*epsPosition + epsWeightedAngle*epsWeightedAngle);

            m_tree.SetNumericLimit(m_epsDistanceMetric);
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
            for(auto&& p : m_parameter.geometry.skeletalPoints){
                maxSekeletalPointDistance = std::max(maxSekeletalPointDistance, std::sqrt(p[0]*p[0] + p[1]*p[1]));
            }
            double additionalBoxExtension = maxSekeletalPointDistance + 2.0 * m_parameter.costMap.resolution;
            samplingArea.Extend(additionalBoxExtension);
            mpsv::structures::CostMapArgument args(m_parameter.costMap.distanceScale, m_parameter.costMap.distanceDecay, staticObstacles);
            m_costMap.Build(samplingArea, m_parameter.costMap.resolution, m_parameter.costMap.modBreakpoints, args, maxNumCells);
        }

        /**
         * @brief Do one iteration of the RRT* motion planning algorithm.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         */
        void Iterate(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Fixed-Node RRT*: ensure that tree is not full by possibly removing a random node
            if((m_state.outOfNodes = !EnsureNonFullTree())){
                return;
            }

            // Sampling: generate random sample
            Node randomSample = Sampling();

            // Vertex Selection: select nearest node
            int16_t idxNearest = m_tree.FindNearestNode(randomSample, m_parameter.metric.weightPsi);

            // Local Planning: steer from nearest sample to random sample
            auto [success, newSample] = LocalPlanning(idxNearest, randomSample, staticObstacles);
            if(!success){
                return;
            }

            // Get near vertices (neighbors of newSample)
            double radius = m_tree.GetOptimalRadius();
            if(!m_tree.FindNearestNeighbors(m_idxNeighbors, newSample, radius, m_parameter.metric.weightPsi)){
                return; // if the value of the new sample already exists, return
            }

            // Find best parent node from the neighborhood (cost of newSample is updated)
            int16_t idxBestParent = FindBestParent(newSample, idxNearest, staticObstacles);

            // Insert into tree: add new sample to the tree
            int16_t idxAddedSample = m_tree.Add(idxBestParent, newSample);

            // Rewire edges in the neighborhood around the added sample
            Rewire(idxAddedSample, staticObstacles);

            // Check if the new sample indicates a solution
            double distanceToGoal = mpsv::math::DistanceMetricSE2(m_tree[idxAddedSample].pose, m_state.finalPose, m_parameter.metric.weightPsi);
            if(distanceToGoal < m_epsDistanceMetric){
                m_state.goalReached = true;
                m_state.closestDistanceToGoal = distanceToGoal;
                m_state.idxSolutionNode = idxAddedSample;
            }
            if(distanceToGoal < m_state.closestDistanceToGoal){
                m_state.closestDistanceToGoal = distanceToGoal;
                m_state.idxSolutionNode = idxAddedSample;
            }
        }

        /**
         * @brief Ensure that the tree is not full. A random leaf is removed if the tree is full.
         * @return True if tree is not full and iterations may continue, false otherwise.
         */
        [[nodiscard]] bool EnsureNonFullTree(void) noexcept {
            if(m_tree.IsFull()){
                if(!m_tree.RemoveRandomLeaf(m_state.idxSolutionNode)){ // current solution node must not be removed
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Generate a random sample.
         * @return The random sample.
         */
        Node Sampling(void) noexcept {
            Node randomSample;
            if(m_state.initialSamplesReversed.size()){
                randomSample.pose = m_state.initialSamplesReversed.back();
                m_state.initialSamplesReversed.pop_back();
            }
            else{
                if(m_state.goalReached || m_state.goalSampleIteration){
                    randomSample.pose = m_pathSampler.Sample(m_state.sampleIteration++);
                }
                else{
                    randomSample.pose = m_state.finalPose;
                }
                m_state.goalSampleIteration = (m_state.goalSampleIteration + 1) % m_parameter.motionPlanner.periodGoalSampling;
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
        std::tuple<bool, Node> LocalPlanning(const int16_t idxSource, const Node& destination, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            Node& source = m_tree[idxSource];

            // Resulting output tuple containing success and newSample
            std::tuple<bool, Node> result;
            bool& success = std::get<0>(result);
            Node& newSample = std::get<1>(result);
            success = false;

            // Direction from source to destination
            std::array<double, 3> direction = mpsv::math::PoseDifference(destination.pose, source.pose);

            // Limit direction to maximum step size
            double z = m_parameter.metric.weightPsi * direction[2];
            double r = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + z*z);
            double radius = m_tree.GetOptimalRadius();
            bool stepLimited = false;
            if(r > radius){
                stepLimited = true;
                radius /= r;
                direction[0] *= radius;
                direction[1] *= radius;
                direction[2] *= radius;
            }

            // Resulting node is source + limited direction
            newSample.pose[0] = source.pose[0] + direction[0];
            newSample.pose[1] = source.pose[1] + direction[1];
            newSample.pose[2] = mpsv::math::SymmetricalAngle(source.pose[2] + direction[2]);

            // Check feasibility and calculate cost
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, m_parameter.metric.weightPsi);
            if(costDistance <= m_epsDistanceMetric){
                return result; // do not allow zero movement
            }
            std::vector<std::array<double,3>>& path = m_tree[idxSource].pathBuffer;
            m_tree.GetPathFromRootToNode(path, idxSource);
            path.push_back(newSample.pose);
            if(!PathFromRootIsFeasible(path, staticObstacles, m_tree[idxSource].trajectoryBuffer)){
                return result;
            }
            double costHeading = mpsv::math::SwayMetric(source.pose, newSample.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
            double costClearance = m_costMap.CostAlongLine(source.pose, newSample.pose, m_parameter.geometry.skeletalPoints);
            newSample.cost = source.cost + costDistance + costHeading + costClearance;

            // If the step was limited during goal sampling, then continue goal sampling in the next iteration
            m_state.goalSampleIteration -= static_cast<uint32_t>(stepLimited && (1 == m_state.goalSampleIteration));
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
        int16_t FindBestParent(Node& newSample, int16_t idxNearest, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            int16_t idxBestParent = idxNearest;
            for(auto&& index : m_idxNeighbors){
                // Ignore the nearest node which is already the parent for the new node, check all others
                if(idxNearest == index){
                    continue;
                }

                // Check if cost reduction is actually possible
                Node& possibleParent = m_tree[index];
                double resultCost = possibleParent.cost;
                resultCost += mpsv::math::DistanceMetricSE2(possibleParent.pose, newSample.pose, m_parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(possibleParent.pose, newSample.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < newSample.cost){
                    resultCost += m_costMap.CostAlongLine(possibleParent.pose, newSample.pose, m_parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < newSample.cost){
                        // Check if a better connection is feasible and update cost and parent index
                        std::vector<std::array<double,3>>& path = m_tree[index].pathBuffer;
                        m_tree.GetPathFromRootToNode(path, index);
                        path.push_back(newSample.pose);
                        if(!PathFromRootIsFeasible(path, staticObstacles, m_tree[index].trajectoryBuffer)){
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
            Node& addedSample = m_tree[idxAddedSample];
            for(auto&& index : m_idxNeighbors){
                // Cannot be a better parent for its own parent or for the root
                if((m_tree[idxAddedSample].iParent == index) || (m_tree[idxAddedSample].iParent < 0)){
                    continue;
                }

                // Check if cost reduction is actually possible
                Node& destination = m_tree[index];
                double resultCost = addedSample.cost;
                resultCost += mpsv::math::DistanceMetricSE2(addedSample.pose, destination.pose, m_parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(addedSample.pose, destination.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < destination.cost){
                    resultCost += m_costMap.CostAlongLine(addedSample.pose, destination.pose, m_parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < destination.cost){
                        // get all leaf nodes from [index]
                        std::vector<int16_t> idxLeafNodes;
                        m_tree.GetLeafIndices(idxLeafNodes, index);

                        // change parent of [index] temporarily to idxAddedSample
                        int16_t idxOriginalParent = m_tree[index].iParent;
                        m_tree[index].iParent = idxAddedSample;

                        // if original parent only has one child (index) it would become a leaf node and must be checked for feasibility as well
                        if((m_tree[idxOriginalParent].iFirstChild >= 0) && (m_tree[m_tree[idxOriginalParent].iFirstChild].iNextSibling < 0)){
                            idxLeafNodes.push_back(idxOriginalParent);
                        }

                        // check all paths from all leafs to the root
                        bool anythingInfeasible = false;
                        #pragma omp parallel for shared(anythingInfeasible, m_tree, staticObstacles)
                        for(auto&& idxLeaf : idxLeafNodes){
                            std::vector<std::array<double,3>>& path = m_tree[idxLeaf].pathBuffer;
                            m_tree.GetPathFromRootToNode(path, idxLeaf);
                            if(!PathFromRootIsFeasible(path, staticObstacles, m_tree[idxLeaf].trajectoryBuffer)){
                                anythingInfeasible = true;
                            }
                        }

                        // reset parent of [index] to original parent
                        m_tree[index].iParent = idxOriginalParent;

                        // at least one infeasible connection means fail: rewire not saftely possible
                        if(anythingInfeasible){
                            continue;
                        }
                        double deltaCost = resultCost - destination.cost;
                        m_tree.Rewire(index, idxAddedSample, deltaCost);
                    }
                }
            }
        }

        /**
         * @brief Check whether a given path starting from the root node is feasible. The initial state corresponding to the root node is given in @ref m_state.
         * @param[in] path The path starting from the root node. This path is used for trajectory generation using a path controller and a nonlinear model of the vehicle.
         * @param[in] staticObstacles Static obstacles to be considered for collision checks.
         * @param[inout] trajectoryBuffer A buffer to be used for trajectory generation.
         * @param[in] minNumSimulationSteps The minimum number of simulation steps to be computed during path exploration (must be at least 1 and at most 0x0000FFFF), defaults to 1.
         * @return True if path is dynamically feasible, e.g. the resulting trajectory is feasible along that path, false otherwise.
         */
        [[nodiscard]] bool PathFromRootIsFeasible(const std::vector<std::array<double,3>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, std::vector<std::array<double,12>>& trajectoryBuffer, uint32_t minNumSimulationSteps = 1) noexcept {
            uint32_t numSteps = m_vehicleSimulator.ExplorePathToEnd(trajectoryBuffer, m_state.initialStateAndInput, path, m_regionOfAttraction, staticObstacles, m_parameter.motionPlanner.sampletime, minNumSimulationSteps);
            if(numSteps >= 0x0000FFFF){
                return false;
            }
            if(m_parameter.geometry.vehicleShape.CheckCollisionCurve(staticObstacles, trajectoryBuffer, m_parameter.geometry.collisionCheckMaxPositionDeviation, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
                return false;
            }
            return true;
        }

        /**
         * @brief Rebuild all trajectories of the tree. For the solution node, as well as for all leaf nodes, trajectories are generated. All remaining node trajectories are cleared.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration.
         */
        void RebuildAllTreeTrajectories(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            const std::vector<int16_t>& indices = m_tree.GetActiveIndices();
            for(auto&& i : indices){
                m_tree[i].trajectoryBuffer.clear();
                if((m_tree[i].iFirstChild < 0) || (i == m_state.idxSolutionNode)){
                    GenerateTrajectoryFromRoot(m_tree[i].trajectoryBuffer, i, staticObstacles);
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
            std::vector<std::array<double,3>>& pathBuffer = m_tree[idxNode].pathBuffer;
            m_tree.GetPathFromRootToNode(pathBuffer, idxNode);

            // Simulate trajectory
            (void) m_vehicleSimulator.ExplorePathToEnd(trajectory, m_state.initialStateAndInput, pathBuffer, m_regionOfAttraction, staticObstacles, m_parameter.motionPlanner.sampletime);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
