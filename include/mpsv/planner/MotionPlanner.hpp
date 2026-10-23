#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/MotionPlannerTree.hpp>
#include <mpsv/planner/MotionPlannerParameterSet.hpp>
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
 * @brief The motion planning algorithm. The planner is ready to solve problems after construction and can be reconfigured with @ref Reconfigure.
 * The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called.
 */
class MotionPlanner {
    private:
        using Node = mpsv::planner::MotionPlannerTreeNode;

    public:
        /**
         * @brief Construct a motion planner with the specified capacities.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree. The default value is 200.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance. The default value is 100000.
         */
        explicit MotionPlanner(int16_t maxNumNodes = 200, uint32_t maxNumSamples = 100000) noexcept {
            Reconfigure(maxNumNodes, maxNumSamples);
        }

        /**
         * @brief Reconfigure the motion planner and reset its internal state.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance.
         */
        void Reconfigure(int16_t maxNumNodes, uint32_t maxNumSamples) noexcept {
            m_runSolve = false;
            m_tree.Resize(maxNumNodes);
            m_pathSampler.Generate(maxNumSamples);
            constexpr size_t maxNumCells = 8192 * 8192;
            m_costMap.Resize(maxNumCells);
            m_idxNeighbors.resize(maxNumNodes);
            m_idxNeighbors.shrink_to_fit();
            m_idxNeighbors.clear();
            m_sampleIteration = 0;
            m_goalSampleIteration = 0;
            m_idxSolutionNode = 0;
            m_closestDistanceToGoal = std::numeric_limits<double>::infinity();
            m_epsDistanceMetric = std::numeric_limits<double>::epsilon();
            m_initialSamplesReversed.clear();
            m_regionOfAttraction.Clear();
            m_vehicleSimulator.ClearController();
            m_vehicleSimulator.ClearModel();
        }

        /**
         * @brief Solve the motion planning problem by doing several iterations for an upper computation time limit.
         * @param[out] dataOut The output data where to store results to.
         * @param[inout] dataIn The input data that represents the motion planning problem to be solved. The data may be modified by this member function, e.g. the static obstacles may be filtered to remove obstacles outside the area of interest.
         * @param[in] parameter The parameter set for motion planning.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds, defaults to 0. If the simulation time is too large (more than 0x0000FFFF steps), then the problem is infeasible.
         * @param[in] forceColdStart True if the initial sample set of the motion planner should be initialized based on the input path only. False if the previous solution path should also be included into the sample set.
         * @details This member function implements an iteration loop that is termimated if @ref Interrupt is called.
         */
        void Solve(mpsv::planner::MotionPlannerOutput& dataOut, mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter, double maxComputationTime, double minTrajectoryDuration = 0.0, bool forceColdStart = false) noexcept {
            m_runSolve = true;
            mpsv::runtime::PerformanceCounter executionCounter;

            // Check validity of parameters and input
            m_idxSolutionNode = 0;
            dataOut.Clear();
            dataOut.errorCode = parameter.IsValid();
            if(error_code::NONE == dataOut.errorCode){
                dataOut.errorCode = dataIn.IsValid();
            }

            // Configure model and controller
            if(error_code::NONE == dataOut.errorCode){
                dataOut.errorCode = m_vehicleSimulator.SetModel(parameter.model);
            }
            if(error_code::NONE == dataOut.errorCode){
                dataOut.errorCode = m_vehicleSimulator.SetController(parameter.motionPlanner.controller);
            }
            if(error_code::NONE == dataOut.errorCode){
                dataOut.errorCode = m_regionOfAttraction.SetParameter(parameter.motionPlanner.regionOfAttraction);
            }

            // Prepare the planner by checking for feasibility of the initial pose and performing a warm start if possible
            if(error_code::NONE == dataOut.errorCode){
                if(forceColdStart){
                    m_sampleIteration = 0;
                }
                m_goalSampleIteration = 0;
                m_initialSamplesReversed.clear();
                if(!InitialPathStartsAtInitialPose(dataIn, parameter)){
                    dataIn.initialPath.clear();
                }
                if(dataIn.initialPath.empty()){
                    dataIn.initialPath.push_back({dataIn.initialStateAndInput[0], dataIn.initialStateAndInput[1], dataIn.initialStateAndInput[2]});
                }
                TrimPath(dataIn.initialPath, parameter.motionPlanner.maxInputPathLength);
                mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn.initialPath, parameter);
                SetInitialSampleSet(dataIn.initialPath, dataIn.originOldToNew, forceColdStart);
                SetNumericLimits(samplingArea, parameter);
                m_tree.SetCSpaceVolume(m_pathSampler.GetVolume());
                BuildCostMap(samplingArea, dataIn.staticObstacles, parameter);
                m_idxSolutionNode = m_tree.Reset(dataIn.initialStateAndInput);
                m_tree.GetPathFromRootToNode(m_tree[m_idxSolutionNode].pathBuffer, m_idxSolutionNode); // set path buffer
                m_tree[m_idxSolutionNode].trajectoryBuffer.clear(); // set trajectory buffer
                m_closestDistanceToGoal = mpsv::math::DistanceMetricSE2(dataIn.initialStateAndInput, dataIn.initialPath.back(), parameter.metric.weightPsi);
                if(parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, dataIn.initialStateAndInput)){
                    dataOut.errorCode = mpsv::error_code::MOTIONPLANNER_INITIAL_POSE_COLLISION;
                }
            }

            // Solve the motion planning problem by doing several iterations
            if(error_code::NONE == dataOut.errorCode){
                double maxIterationTime = 0.0;
                while(m_runSolve && (error_code::NONE == dataOut.errorCode) && ((maxIterationTime + executionCounter.TimeToStart()) < maxComputationTime)){
                    mpsv::runtime::PerformanceCounter iterationCounter;
                    dataOut.errorCode = Iterate(dataIn, parameter);
                    dataOut.numberOfPerformedIterations++;
                    maxIterationTime = std::max(maxIterationTime, iterationCounter.TimeToStart());
                }
                if(error_code::NONE == dataOut.errorCode){
                    dataOut.errorCode = BuildFinalTrajectory(dataIn, parameter, minTrajectoryDuration);
                }
            }

            // assign output
            dataOut.goalReached = (m_closestDistanceToGoal < m_epsDistanceMetric);
            dataOut.cost = m_tree[m_idxSolutionNode].cost;
            dataOut.referencePath.swap(m_tree[m_idxSolutionNode].pathBuffer);
            dataOut.trajectory.swap(m_tree[m_idxSolutionNode].trajectoryBuffer);
            dataOut.sampletime = parameter.motionPlanner.sampletime;
            dataOut.timestampOfComputationUTC = mpsv::runtime::GetTimestampUTC();

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
         * @param[in] parameter The parameter set used to solve the motion planning problem.
         * @return [0] The total time that has been simulated.
         * @return [1] The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<double,std::array<double,9>> PredictMotion(const std::array<double,12>& initialStateAndInput, double predictionHorizon, const mpsv::planner::MotionPlannerParameterSet& parameter) const noexcept {
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(0.0, std::floor(predictionHorizon / parameter.motionPlanner.sampletime)));
            double simulationTime = static_cast<double>(numSimulationSteps) * parameter.motionPlanner.sampletime;
            std::array<double,9> finalState = m_vehicleSimulator.PredictMotion(initialStateAndInput, parameter.motionPlanner.sampletime, numSimulationSteps);
            return std::make_tuple(simulationTime, finalState);
        }

        /**
         * @brief Predict the motion trajectory of the vehicle for a given prediction horizon assuming the control input to be constant all the time. At least one simulation step is computed.
         * @param[out] trajectory The prediction trajectory where each entry contains the state and the input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] predictionHorizon The total simulation time for the prediction in seconds. Note that this value is gridded to the internal sampletime parameter. The actual simulation time is returned. This value is clamped to a minimum value of a sampletime.
         * @param[in] parameter The parameter set used to solve the motion planning problem.
         * @return A tuple containing the following values.
         * [0] True if success, false otherwise. If the maximum number of simulation steps (0x0000FFFF) is exceeded, false is returned.
         * [1] The total time that has been simulated.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        [[nodiscard]] std::tuple<bool,double> PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double predictionHorizon, const mpsv::planner::MotionPlannerParameterSet& parameter) const noexcept {
            constexpr uint32_t maxNumSimulationSteps = 0x0000FFFF;
            uint32_t numSimulationSteps = static_cast<uint32_t>(std::max(1.0, std::floor(predictionHorizon / parameter.motionPlanner.sampletime)));
            bool success = m_vehicleSimulator.PredictMotionTrajectory(trajectory, initialStateAndInput, parameter.motionPlanner.sampletime, numSimulationSteps, maxNumSimulationSteps);
            double totalSimulationTime = parameter.motionPlanner.sampletime * static_cast<double>(trajectory.size());
            return std::make_tuple(success, totalSimulationTime);
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration. All trajectories of the tree are rebuilt. Otherwise the trajectories from tree nodes are undefined.
         * @param[in] dataIn Input data used for the latest solve.
         * @param[in] parameter Parameter set used for the latest solve.
         * @note IMPORTANT: The caller of this function must make sure that static obstacles are valid!
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::MotionPlannerInput& dataIn, mpsv::planner::MotionPlannerParameterSet& parameter) noexcept { RebuildAllTreeTrajectories(staticObstacles, dataIn, parameter); m_tree.WriteToFile(file, preString + "tree."); m_costMap.WriteToFile(file, preString + "costMap."); }

    protected:
        mpsv::planner::MotionPlannerTree m_tree;                                   // Tree for motion planning.
        mpsv::sampler::PathSampler m_pathSampler;                                  // The path sampler to be used to draw random samples from SE(2).
        mpsv::structures::CostMap m_costMap;                                       // The cost map to be used as additional cost term in the objective function.
        std::vector<int16_t> m_idxNeighbors;                                       // Reserved memory for nearest neighbors.
        std::atomic<bool> m_runSolve{false};                                       // True if a solve operation is running, false if not or is requested to terminate.
        uint32_t m_sampleIteration{0};                                             // Index of the next random sample.
        uint32_t m_goalSampleIteration{0};                                         // Goal-sampling iteration counter.
        int16_t m_idxSolutionNode{0};                                              // Index to the current solution node.
        double m_closestDistanceToGoal{std::numeric_limits<double>::infinity()};   // Distance from the solution to the goal.
        double m_epsDistanceMetric{std::numeric_limits<double>::epsilon()};        // Eps (numeric threshold) for distance metric.
        std::vector<std::array<double,3>> m_initialSamplesReversed;                // Initial samples, in reverse sampling order.
        mpsv::control::VehicleSimulator m_vehicleSimulator;                        // The vehicle simulator to be used for generating dynamically feasible trajectories.
        mpsv::control::RegionOfAttraction m_regionOfAttraction;                    // The region of attraction to be used to check if a state enters the region around a pose.

        /**
         * @brief Build the final trajectory. This member function calculates the reference path and the trajectory for the current solution node.
         * @param[in] minTrajectoryDuration The minimum duration (>= 0) of the trajectory in seconds.
         * @return mpsv::error_code::NONE if the resulting trajectory is feasible, otherwise mpsv::error_code::MOTIONPLANNER_INFEASIBLE_TRAJECTORY.
         */
        mpsv::error_code BuildFinalTrajectory(const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter, double minTrajectoryDuration){
            uint32_t minNumSteps = static_cast<uint32_t>(std::ceil(std::max(0.0, minTrajectoryDuration) / parameter.motionPlanner.sampletime));
            m_tree.GetPathFromRootToNode(m_tree[m_idxSolutionNode].pathBuffer, m_idxSolutionNode);
            if(!PathFromRootIsFeasible(dataIn.initialStateAndInput, parameter, m_tree[m_idxSolutionNode].pathBuffer, dataIn.staticObstacles, m_tree[m_idxSolutionNode].trajectoryBuffer, minNumSteps)){
                return mpsv::error_code::MOTIONPLANNER_INFEASIBLE_TRAJECTORY;
            }
            return mpsv::error_code::NONE;
        }

        /**
         * @brief Check whether the initial path starts at the initial state.
         * @param[in] dataIn Input data to the motion planner containing the initial path as well as the initial state.
         * @return True if the initial state corresponds to the start of the initial path, false otherwise.
         */
        [[nodiscard]] bool InitialPathStartsAtInitialPose(mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            if(dataIn.initialPath.empty()){
                return false;
            }
            double distance = mpsv::math::DistanceMetricSE2(dataIn.initialPath[0], dataIn.initialStateAndInput, parameter.metric.weightPsi);
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
            m_initialSamplesReversed.clear();
            if(!forceColdStart){
                // Transform tree to new origin and use old solution path as initial sample set
                m_tree.TransformToNewOrigin(originOldToNew);
                m_tree.GetPathFromRootToNode(m_tree[m_idxSolutionNode].pathBuffer, m_idxSolutionNode);

                // Use old solution path as initial sample set if those poses are inside the new path sampling area
                for(auto&& pose : m_tree[m_idxSolutionNode].pathBuffer){
                    if(m_pathSampler.IsInside(pose)){
                        m_initialSamplesReversed.push_back(pose);
                    }
                }
            }

            // Insert initial path to sample set (exclude first pose, that would be the initial pose)
            m_initialSamplesReversed.insert(m_initialSamplesReversed.end(), initialPath.begin() + 1, initialPath.end());

            // Reverse the order such that the end of the container contains the first poses to be sampled
            std::reverse(m_initialSamplesReversed.begin(), m_initialSamplesReversed.end());
        }

        /**
         * @brief Set the sampling area based on a given path. The area corresponds to the minimum area oriented box enclosing all points of the path, extended by the position sampling range.
         * @param[in] path The path that should be contained in the sampling area.
         * @return The oriented box representing the rectangular hull of the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(const std::vector<std::array<double,3>>& path, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            mpsv::geometry::OrientedBox box;
            const double sqrt2 = std::sqrt(2.0);
            m_pathSampler.Reset(path, parameter.motionPlanner.samplingRangePosition, parameter.motionPlanner.samplingRangeAngle, parameter.metric.weightPsi);
            box.CreateFromPointCloud(path);
            box.Extend(parameter.motionPlanner.samplingRangePosition * sqrt2);
            return box;
        }

        /**
         * @brief Set absolute numeric limits. This limit is used to check for equal poses.
         * @param[in] samplingArea The current sampling area.
         */
        void SetNumericLimits(const mpsv::geometry::OrientedBox& samplingArea, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            std::array<double,2> dimension = samplingArea.GetDimension();
            double maxDimension = std::max(dimension[0], dimension[1]);

            // Eps for position and angle
            double epsPosition = std::numeric_limits<double>::epsilon() * std::max(1.0, maxDimension);
            double epsAngle = std::numeric_limits<double>::epsilon() * 6.28318530717959;

            // Eps for distance metric sqrt(dx^2 + dy^2 + (w*dpsi)^2)
            double epsWeightedAngle = std::max(1.0, parameter.metric.weightPsi) * epsAngle;
            m_epsDistanceMetric = std::sqrt(2.0*epsPosition*epsPosition + epsWeightedAngle*epsWeightedAngle);

            m_tree.SetNumericLimit(m_epsDistanceMetric);
        }

        /**
         * @brief Build the 2D-look-up table of the costmap.
         * @param[in] samplingArea The sampling area.
         * @param[in] staticObstacles Static obstacles to be considered for building the map.
         * @param[in] parameter The parameter set for path planning.
         */
        void BuildCostMap(mpsv::geometry::OrientedBox samplingArea, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            double maxSekeletalPointDistance = 0.0;
            for(auto&& p : parameter.geometry.skeletalPoints){
                maxSekeletalPointDistance = std::max(maxSekeletalPointDistance, std::sqrt(p[0]*p[0] + p[1]*p[1]));
            }
            samplingArea.Extend(maxSekeletalPointDistance + 2.0 * parameter.costMap.resolution); // extend by the maximum distance of skeletal points and two more cells as safety threshold
            mpsv::structures::CostMapArgument args(parameter.costMap.distanceScale, parameter.costMap.distanceDecay, staticObstacles);
            m_costMap.Build(samplingArea, parameter.costMap.resolution, parameter.costMap.modBreakpoints, args);
        }

        /**
         * @brief Do one iteration of the RRT* motion planning algorithm.
         * @param[in] dataIn Input data containing the state, path, and obstacles.
         * @param[in] parameter Parameters for motion planning.
         * @return mpsv::error_code::NONE if the iteration was successful, a non-zero error code otherwise.
         */
        mpsv::error_code Iterate(const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            if(m_tree.IsFull() && !m_tree.RemoveRandomLeaf(m_idxSolutionNode)){
                return mpsv::error_code::MOTIONPLANNER_OUT_OF_NODES; // no more nodes available in the tree and no random leaf node could be removed
            }
            Node randomSample = Sampling(dataIn.initialPath.back(), parameter);
            int16_t idxNearest = m_tree.FindNearestNode(randomSample, parameter.metric.weightPsi);
            auto [success, newSample] = LocalPlanning(idxNearest, randomSample, dataIn, parameter);
            if(success){
                if(!m_tree.FindNearestNeighbors(m_idxNeighbors, newSample, m_tree.GetOptimalRadius(), parameter.metric.weightPsi)){
                    int16_t idxBestParent = FindBestParent(newSample, idxNearest, dataIn, parameter);  // updates newSample.cost
                    int16_t idxAddedSample = m_tree.Add(idxBestParent, newSample);
                    Rewire(idxAddedSample, dataIn, parameter);
                    double distanceToGoal = mpsv::math::DistanceMetricSE2(m_tree[idxAddedSample].pose, dataIn.initialPath.back(), parameter.metric.weightPsi);
                    if(distanceToGoal < m_closestDistanceToGoal){
                        m_closestDistanceToGoal = distanceToGoal;
                        m_idxSolutionNode = idxAddedSample;
                    }
                }
            }
            return mpsv::error_code::NONE;
        }

        /**
         * @brief Generate a random sample.
         * @param[in] finalPose The final pose to be used for goal sampling.
         * @param[in] parameter The parameter set for motion planning.
         * @return The random sample.
         */
        Node Sampling(const std::array<double,3>& finalPose, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            Node randomSample;
            if(m_initialSamplesReversed.size()){
                randomSample.pose = m_initialSamplesReversed.back();
                m_initialSamplesReversed.pop_back();
            }
            else{
                if((m_closestDistanceToGoal < m_epsDistanceMetric) || m_goalSampleIteration){
                    randomSample.pose = m_pathSampler.Sample(m_sampleIteration++);
                }
                else{
                    randomSample.pose = finalPose;
                }
                m_goalSampleIteration = (m_goalSampleIteration + 1) % parameter.motionPlanner.periodGoalSampling;
            }
            return randomSample;
        }

        /**
         * @brief Perform local planning from a source node to a destination node and create a resulting node.
         * @param[in] idxSource The index to the source node from which to start the local planning.
         * @param[in] destination The destination node to which to steer to from the source node.
         * @param[in] dataIn Input data containing the state, path, and obstacles.
         * @param[in] parameter Parameters for motion planning.
         * @return A tuple containing <success, newSample>.
         * <0> True if local planning succeeded (e.g. no collision), false otherwise.
         * <1> The resulting sample of local planning.
         */
        std::tuple<bool, Node> LocalPlanning(const int16_t idxSource, const Node& destination, const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            std::tuple<bool, Node> result{};
            auto& [success, newSample] = result;
            Node& source = m_tree[idxSource];

            // Limit step size to optimal radius of the tree and calculate resulting node
            std::array<double, 3> direction = mpsv::math::PoseDifference(destination.pose, source.pose);
            double z = parameter.metric.weightPsi * direction[2];
            double r = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + z*z);
            double radius = m_tree.GetOptimalRadius();
            bool stepLimited = (r > radius);
            if(stepLimited){
                radius /= r;
                direction[0] *= radius;
                direction[1] *= radius;
                direction[2] *= radius;
            }
            newSample.pose[0] = source.pose[0] + direction[0];
            newSample.pose[1] = source.pose[1] + direction[1];
            newSample.pose[2] = mpsv::math::SymmetricalAngle(source.pose[2] + direction[2]);

            // Check feasibility and calculate cost
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, parameter.metric.weightPsi);
            if(costDistance <= m_epsDistanceMetric){
                return result; // do not allow zero movement
            }
            m_tree.GetPathFromRootToNode(source.pathBuffer, idxSource);
            source.pathBuffer.push_back(newSample.pose);
            if(!PathFromRootIsFeasible(dataIn.initialStateAndInput, parameter, source.pathBuffer, dataIn.staticObstacles, source.trajectoryBuffer)){
                return result;
            }
            double costHeading = mpsv::math::SwayMetric(source.pose, newSample.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
            double costClearance = m_costMap.CostAlongLine(source.pose, newSample.pose, parameter.geometry.skeletalPoints);
            newSample.cost = source.cost + costDistance + costHeading + costClearance;

            // If the step was limited during goal sampling, then continue goal sampling in the next iteration
            m_goalSampleIteration -= static_cast<uint32_t>(stepLimited && (1 == m_goalSampleIteration));
            success = true;
            return result;
        }

        /**
         * @brief Find best parent for a new sample.
         * @param[inout] newSample The new sample from local planning for which to find the best parent. The cost of this sample is updated.
         * @param[in] idxNearest Index to the nearest sample of the newSample which is the current parent from local planning.
         * @param[in] dataIn Input data containing the state, path, and obstacles.
         * @param[in] parameter Parameters for motion planning.
         * @return Index to the best parent node such that the cost for the new sample becomes minimal and the edge from that parent to the new sample is feasible.
         */
        int16_t FindBestParent(Node& newSample, int16_t idxNearest, const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            int16_t idxBestParent = idxNearest;
            for(auto&& index : m_idxNeighbors){
                if(idxNearest == index){
                    continue; // the nearest node is already the parent for the new node, check all others
                }

                // Check if cost reduction is actually possible
                Node& possibleParent = m_tree[index];
                double resultCost = possibleParent.cost;
                resultCost += mpsv::math::DistanceMetricSE2(possibleParent.pose, newSample.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(possibleParent.pose, newSample.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < newSample.cost){
                    resultCost += m_costMap.CostAlongLine(possibleParent.pose, newSample.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < newSample.cost){
                        m_tree.GetPathFromRootToNode(m_tree[index].pathBuffer, index);
                        m_tree[index].pathBuffer.push_back(newSample.pose);
                        if(!PathFromRootIsFeasible(dataIn.initialStateAndInput, parameter, m_tree[index].pathBuffer, dataIn.staticObstacles, m_tree[index].trajectoryBuffer)){
                            continue; // infeasible, cannot be a better parent
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
         * @param[in] dataIn Input data containing the state, path, and obstacles.
         * @param[in] parameter Parameters for motion planning.
         */
        void Rewire(int16_t idxAddedSample, const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            Node& addedSample = m_tree[idxAddedSample];
            for(auto&& index : m_idxNeighbors){
                if((m_tree[idxAddedSample].iParent == index) || (m_tree[idxAddedSample].iParent < 0)){
                    continue; // cannot be a better parent for its own parent
                }

                // Check if cost reduction is actually possible
                Node& destination = m_tree[index];
                double resultCost = addedSample.cost;
                resultCost += mpsv::math::DistanceMetricSE2(addedSample.pose, destination.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(addedSample.pose, destination.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < destination.cost){
                    resultCost += m_costMap.CostAlongLine(addedSample.pose, destination.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < destination.cost){
                        std::vector<int16_t> idxLeafNodes;
                        m_tree.GetLeafIndices(idxLeafNodes, index);

                        // change parent temporarily to idxAddedSample
                        int16_t idxOriginalParent = m_tree[index].iParent;
                        m_tree[index].iParent = idxAddedSample;

                        // if original parent only has one child (index) it would become a leaf node and must be checked for feasibility as well
                        if((m_tree[idxOriginalParent].iFirstChild >= 0) && (m_tree[m_tree[idxOriginalParent].iFirstChild].iNextSibling < 0)){
                            idxLeafNodes.push_back(idxOriginalParent);
                        }

                        // check all paths from all leafs to the root
                        bool anythingInfeasible = false;
                        #pragma omp parallel for shared(anythingInfeasible, m_tree, dataIn, parameter)
                        for(auto&& idxLeaf : idxLeafNodes){
                            m_tree.GetPathFromRootToNode(m_tree[idxLeaf].pathBuffer, idxLeaf);
                            if(!PathFromRootIsFeasible(dataIn.initialStateAndInput, parameter, m_tree[idxLeaf].pathBuffer, dataIn.staticObstacles, m_tree[idxLeaf].trajectoryBuffer)){
                                anythingInfeasible = true;
                            }
                        }

                        // reset parent to original parent
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
         * @brief Check whether a given path starting from the root node is feasible.
         * @param[in] initialStateAndInput Initial state/input value from which to start the simulation.
         * @param[in] parameter Parameters for motion planning.
         * @param[in] path The path starting from the root node. This path is used for trajectory generation using a path controller and a nonlinear model of the vehicle.
         * @param[in] staticObstacles Static obstacles to be considered for collision checks.
         * @param[inout] trajectoryBuffer A buffer to be used for trajectory generation.
         * @param[in] minNumSimulationSteps The minimum number of simulation steps to be computed during path exploration (must be at least 1 and at most 0x0000FFFF), defaults to 1.
         * @return True if path is dynamically feasible, e.g. the resulting trajectory is feasible along that path, false otherwise.
         */
        [[nodiscard]] bool PathFromRootIsFeasible(const std::array<double,12>& initialStateAndInput, const mpsv::planner::MotionPlannerParameterSet& parameter, const std::vector<std::array<double,3>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, std::vector<std::array<double,12>>& trajectoryBuffer, uint32_t minNumSimulationSteps = 1) noexcept {
            uint32_t numSteps = m_vehicleSimulator.ExplorePathToEnd(trajectoryBuffer, initialStateAndInput, path, m_regionOfAttraction, staticObstacles, parameter.motionPlanner.sampletime, minNumSimulationSteps);
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
         * @param[in] dataIn Input data used for the latest solve.
         * @param[in] parameter Parameter set used for the latest solve.
         */
        void RebuildAllTreeTrajectories(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            const std::vector<int16_t>& indices = m_tree.GetActiveIndices();
            for(auto&& i : indices){
                m_tree[i].trajectoryBuffer.clear();
                if((m_tree[i].iFirstChild < 0) || (i == m_idxSolutionNode)){
                    GenerateTrajectoryFromRoot(m_tree[i].trajectoryBuffer, i, staticObstacles, dataIn, parameter);
                }
            }
        }

        /**
         * @brief Generate trajectory from the root node to a specific node.
         * @param[out] trajectory Final trajectory from the root node to the specified node. Each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted in the trajectory.
         * @param[in] idxNode Index to the node for which to generate the trajectory.
         * @param[in] staticObstacles List of static obstacles to be used for the guidance law of path exploration.
         * @param[in] dataIn Input data used for the latest solve.
         * @param[in] parameter Parameter set used for the latest solve.
         */
        void GenerateTrajectoryFromRoot(std::vector<std::array<double,12>>& trajectory, int16_t idxNode, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::MotionPlannerInput& dataIn, const mpsv::planner::MotionPlannerParameterSet& parameter) noexcept {
            // Get the path from the root
            std::vector<std::array<double,3>>& pathBuffer = m_tree[idxNode].pathBuffer;
            m_tree.GetPathFromRootToNode(pathBuffer, idxNode);

            // Simulate trajectory
            (void) m_vehicleSimulator.ExplorePathToEnd(trajectory, dataIn.initialStateAndInput, pathBuffer, m_regionOfAttraction, staticObstacles, parameter.motionPlanner.sampletime);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
