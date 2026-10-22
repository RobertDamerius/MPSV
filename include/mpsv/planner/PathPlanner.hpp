#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/PathPlannerParameterSet.hpp>
#include <mpsv/planner/PathPlannerState.hpp>
#include <mpsv/planner/PathPlannerInput.hpp>
#include <mpsv/planner/PathPlannerOutput.hpp>
#include <mpsv/structures/CostMap.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/sampler/UniformBoxSamplerSE2.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/runtime/Time.hpp>
#include <mpsv/common/ErrorCode.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/math/Metric.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The path planning algorithm. Before solving path planning problems, initialize the planner by calling the @ref Initialize member function.
 * The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called.
 */
class PathPlanner {
    private:
        using Node = mpsv::structures::FixedSizeTreeSE2Node;

    public:
        /**
         * @brief Construct a path planner and set default values.
         */
        PathPlanner() noexcept { Initialize(1, 1); }

        /**
         * @brief Destroy the path planner.
         */
        ~PathPlanner() noexcept { Terminate(); }

        /**
         * @brief Initialize or re-initialize the path planner.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance.
         */
        void Initialize(int16_t maxNumNodes, uint32_t maxNumSamples) noexcept {
            Terminate();
            m_idxNeighbors.reserve(maxNumNodes);
            m_tree.Clear(maxNumNodes);
            m_boxSampler.Initialize(maxNumSamples);
        }

        /**
         * @brief Terminate the path planner.
         */
        void Terminate(void) noexcept {
            m_runSolve = false;
            m_tree.Clear();
            m_costMap.Clear();
            m_state.Clear();
            m_boxSampler.Terminate();
            m_idxNeighbors.clear();
            m_idxNeighbors.shrink_to_fit();
            m_epsDistanceMetric = std::numeric_limits<double>::epsilon();
        }

        /**
         * @brief Solve the path planning problem by doing several iterations for an upper computation time limit.
         * @param[out] dataOut The output data where to store results to.
         * @param[inout] dataIn The input data that represents the path planning problem to be solved. The data may be modified by this member function, e.g. the static obstacles may be filtered to remove obstacles outside the area of interest.
         * @param[in] parameter The parameter set for path planning.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @param[in] forceColdStart True if no warm start should be performed and the path planner starts with an empty tree, false otherwise. The default value is false.
         * @details This member function implements an iteration loop that is termimated if @ref Interrupt is called.
         */
        void Solve(mpsv::planner::PathPlannerOutput& dataOut, mpsv::planner::PathPlannerInput& dataIn, const mpsv::planner::PathPlannerParameterSet& parameter, double maxComputationTime, bool forceColdStart = false) noexcept {
            m_runSolve = true;
            mpsv::runtime::PerformanceCounter executionCounter;

            // Check validity of parameters and input
            dataOut.Clear();
            dataOut.errorCode = parameter.IsValid();
            if(error_code::NONE == dataOut.errorCode){
                dataOut.errorCode = dataIn.IsValid();
            }

            // Prepare
            if(mpsv::error_code::NONE == dataOut.errorCode){
                m_state.Clear(!forceColdStart); // keep latest solution if no cold start is forced
                mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn);
                SetNumericLimits(samplingArea, parameter);
                m_tree.SetCSpaceVolume(samplingArea.GetVolume(parameter.metric.weightPsi));
                BuildCostMap(samplingArea, dataIn.staticObstacles, parameter);
                RemoveObstaclesOutsideAreaOfInterest(dataIn.staticObstacles, samplingArea, parameter);
                if(parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, dataIn.initialPose)){
                    dataOut.errorCode = mpsv::error_code::PATHPLANNER_INITIAL_POSE_COLLISION;
                    m_state.idxSolutionNode = m_tree.Reset(dataIn.initialPose);
                    m_state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(dataIn.initialPose, dataIn.finalPose, parameter.metric.weightPsi);
                    m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
                }
                else{
                    if(forceColdStart || !WarmStart(dataIn, parameter)){
                        m_state.idxSolutionNode = m_tree.Reset(dataIn.initialPose);
                        m_state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(dataIn.initialPose, dataIn.finalPose, parameter.metric.weightPsi);
                        m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
                    }
                }
            }

            // Solve
            if(mpsv::error_code::NONE == dataOut.errorCode){
                double maxIterationTime = 0.0; // maximum execution time of one iteration
                while(m_runSolve){
                    ++m_state.numberOfPerformedIterations;
                    mpsv::runtime::PerformanceCounter iterationCounter;
                    Iterate(dataIn, parameter);
                    maxIterationTime = std::max(maxIterationTime, iterationCounter.TimeToStart());
                    if(m_state.outOfNodes){
                        dataOut.errorCode = mpsv::error_code::PATHPLANNER_OUT_OF_NODES;
                        break;
                    }
                    if((maxIterationTime + executionCounter.TimeToStart()) >= maxComputationTime){
                        break;
                    }
                }
            }

            // assign output
            dataOut.goalReached = m_state.goalReached;
            dataOut.numberOfPerformedIterations = m_state.numberOfPerformedIterations;
            dataOut.cost = m_tree[m_state.idxSolutionNode].cost;
            m_tree.GetPathFromRootToNode(dataOut.path, m_state.idxSolutionNode);
            dataOut.timestampOfComputationUTC = mpsv::runtime::GetTimestampUTC();

            m_runSolve = false;
        }

        /**
         * @brief Interrupt a running @ref Solve operation of the path planner.
         */
        void Interrupt(void) noexcept { m_runSolve = false; }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept { m_tree.WriteToFile(file, preString + "tree."); m_costMap.WriteToFile(file, preString + "costMap."); }

    protected:
        mpsv::structures::FixedSizeTreeSE2<Node> m_tree;                      // Tree for path planning.
        mpsv::planner::PathPlannerState m_state;                              // State of the path planner.
        mpsv::structures::CostMap m_costMap;                                  // The cost map to be used as additional cost term in the objective function.
        mpsv::sampler::UniformBoxSamplerSE2 m_boxSampler;                     // The box sampler to be used to draw random samples from SE(2).
        std::vector<int16_t> m_idxNeighbors;                                  // Reserved memory for nearest neighbors.
        std::atomic<bool> m_runSolve{false};                                  // True if a solve operation is running, false if not or is requested to terminate.
        double m_epsDistanceMetric{std::numeric_limits<double>::epsilon()};   // Eps (numeric threshold) for distance metric.

        /**
         * @brief Set the sampling area.
         * @param[in] dataIn The input data containing the sampling box center pose and dimensions.
         * @return The oriented box representing the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(const mpsv::planner::PathPlannerInput& dataIn) noexcept {
            mpsv::geometry::OrientedBox box;
            box.Create(dataIn.samplingBoxCenterPose, dataIn.samplingBoxDimension);
            m_boxSampler.SetSize(box);
            return box;
        }

        /**
         * @brief Set absolute numeric limits. This limit is used to check for equal poses.
         * @param[in] samplingArea The current sampling area.
         * @param[in] parameter The parameter set for path planning.
         */
        void SetNumericLimits(const mpsv::geometry::OrientedBox& samplingArea, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
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
         * @brief Remove obstacles that are completely outside the new configuration space (sampling box + farest vehicle shape vertex).
         * @param[inout] staticObstacles Static obstacles to be considered. The content of the container may change.
         * @param[in] samplingArea The sampling area.
         * @param[in] parameter The parameter set for path planning.
         */
        void RemoveObstaclesOutsideAreaOfInterest(std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, mpsv::geometry::OrientedBox samplingArea, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            double boxExtension = parameter.geometry.vehicleShape.GetFarthestVertexDistance();
            samplingArea.Extend(boxExtension);
            mpsv::geometry::ConvexPolygon boxPolygon = samplingArea.ExportAsConvexPolygon();
            staticObstacles.erase(std::remove_if(staticObstacles.begin(), staticObstacles.end(), [&boxPolygon](mpsv::geometry::ConvexPolygon& p){ return !boxPolygon.Overlap(p); } ), staticObstacles.end());
        }

        /**
         * @brief Build the 2D-look-up table of the cost map.
         * @param[in] samplingArea The sampling area.
         * @param[in] staticObstacles Static obstacles to be considered for building the costmap.
         * @param[in] parameter The parameter set for path planning.
         */
        void BuildCostMap(mpsv::geometry::OrientedBox samplingArea, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            // The range must be at least the range of the sampling box. This box is further extended by the maximum distance of skeletal points for
            // cost map evaluation and by some safety threshold of two more cells.
            constexpr size_t maxNumCells = 125000000; // limit to about 1 GB
            double maxSekeletalPointDistance = 0.0;
            for(auto&& p : parameter.geometry.skeletalPoints){
                maxSekeletalPointDistance = std::max(maxSekeletalPointDistance, std::sqrt(p[0]*p[0] + p[1]*p[1]));
            }
            double additionalBoxExtension = maxSekeletalPointDistance + 2.0 * parameter.costMap.resolution;
            samplingArea.Extend(additionalBoxExtension);
            mpsv::structures::CostMapArgument args(parameter.costMap.distanceScale, parameter.costMap.distanceDecay, staticObstacles);
            m_costMap.Build(samplingArea, parameter.costMap.resolution, parameter.costMap.modBreakpoints, args, maxNumCells);
        }

        /**
         * @brief Warm start the path planner by transforming the tree to a new origin and connecting the initial pose with the old solution path.
         * @param[in] dataIn The input data containing the path planning problem.
         * @param[in] parameter The parameter set for path planning.
         * @return True if warm start was successful, false otherwise. If this method fails, a cold start must be performed.
         */
        [[nodiscard]] bool WarmStart(const mpsv::planner::PathPlannerInput& dataIn, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            // Step 1: Transform tree to new origin
            m_tree.TransformToNewOrigin(dataIn.originOldToNew);

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 2: Connect initial pose with old solution path
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Get the current solution branch
            std::vector<int16_t> idxSolutionPathReverse;
            int16_t node = m_state.idxSolutionNode;
            while(node >= 0){
                idxSolutionPathReverse.push_back(node);
                node = m_tree[node].iParent;
            }

            // If there's no path (only one node or less), then we can make a cold start
            if(idxSolutionPathReverse.size() <= 1) return false;

            // Calculate the cummulative cost for all nodes along the given solution path based on the updated costmap
            // idxSolutionPathReverse starts with the solution node and ends with the root node.
            m_tree[idxSolutionPathReverse.back()].cost = 0.0;
            for(size_t k = idxSolutionPathReverse.size()-1; k > 0; --k){
                auto& a = m_tree[idxSolutionPathReverse[k]];
                auto& b = m_tree[idxSolutionPathReverse[k - 1]];
                double costDistance = mpsv::math::DistanceMetricSE2(a.pose, b.pose, parameter.metric.weightPsi);
                double costHeading = mpsv::math::SwayMetric(a.pose, b.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                double costClearance = m_costMap.CostAlongLine(a.pose, b.pose, parameter.geometry.skeletalPoints);
                b.cost = a.cost + (costDistance + costHeading + costClearance);
            }

            // Find best collision-free connection from initial pose to solution path: find that connection, that is collision-free and reduces the cost.
            int16_t idxBestConnection = -1;
            double bestTotalCost = std::numeric_limits<double>::infinity();
            for(auto&& i : idxSolutionPathReverse){
                auto& node = m_tree[i];
                if(!parameter.geometry.vehicleShape.CheckCollisionLine(dataIn.staticObstacles, node.pose, dataIn.initialPose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                    // The total cost for this connection is the cost from the initial pose to the connection point added to the cummulative cost from the connection point to the solution (old solution)
                    double costDistance = mpsv::math::DistanceMetricSE2(dataIn.initialPose, node.pose, parameter.metric.weightPsi);
                    double costHeading = mpsv::math::SwayMetric(dataIn.initialPose, node.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                    double costClearance = m_costMap.CostAlongLine(dataIn.initialPose, node.pose, parameter.geometry.skeletalPoints);
                    double costToGoal = m_tree[idxSolutionPathReverse[0]].cost - node.cost;
                    double totalCost = (costDistance + costHeading + costClearance) + costToGoal;
                    if(totalCost < bestTotalCost){
                        bestTotalCost = totalCost;
                        idxBestConnection = i;
                    }
                }
            }

            // If no connection could be found, then we can make a cold start
            if(idxBestConnection < 0) return false;


            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 3: Change the root of the tree
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // The tree starts from initialPose and connects to idxBestConnection
            int16_t idxRoot = m_tree.ChangeRoot(dataIn.initialPose, idxBestConnection, parameter.metric.weightPsi);
            if(idxRoot < 0) return false;


            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 4: Check feasibility and recalculate cost of all edges and prune the tree (feasible if no collision and inside new sampling area)
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            std::vector<int16_t> traversalStack;
            traversalStack.push_back(idxRoot);
            m_tree[idxRoot].cost = 0.0;
            while(!traversalStack.empty()){
                int16_t current = traversalStack.back();
                traversalStack.pop_back();
                int16_t child = m_tree[current].iFirstChild;
                while(child >= 0){
                    int16_t nextChild = m_tree[child].iNextSibling;
                    auto& a = m_tree[current];
                    auto& b = m_tree[child];
                    if(m_boxSampler.IsInside(b.pose) && !parameter.geometry.vehicleShape.CheckCollisionLine(dataIn.staticObstacles, b.pose, a.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                        if(m_tree[child].iFirstChild >= 0){
                            traversalStack.push_back(child);
                        }
                        b.cost = a.cost;
                        b.cost += mpsv::math::DistanceMetricSE2(a.pose, b.pose, parameter.metric.weightPsi);
                        b.cost += mpsv::math::SwayMetric(a.pose, b.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
                        b.cost += m_costMap.CostAlongLine(a.pose, b.pose, parameter.geometry.skeletalPoints);
                    }
                    else{
                        m_tree.DeleteSubtree(child);
                    }
                    child = nextChild;
                }
            }


            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 5: Find closest node to new goal
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Set default state values
            m_state.outOfNodes = false;
            m_state.closestDistanceToGoal = std::numeric_limits<double>::infinity();
            m_state.idxSolutionNode = 0;

            // Find closest distance to goal
            const std::vector<int16_t>& indices = m_tree.GetActiveIndices();
            double distance;
            for(auto&& i : indices){
                distance = mpsv::math::DistanceMetricSE2(m_tree[i].pose, dataIn.finalPose, parameter.metric.weightPsi);
                if(distance < m_state.closestDistanceToGoal){
                    m_state.closestDistanceToGoal = distance;
                    m_state.idxSolutionNode = i;
                }
            }

            // Check if solution was found
            m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
            return true;
        }

        /**
         * @brief Do one iteration of the RRT* path planning algorithm.
         * @param[in] dataIn The input data containing the path planning problem.
         * @param[in] parameter The parameter set for path planning.
         */
        void Iterate(const mpsv::planner::PathPlannerInput& dataIn, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            if((m_state.outOfNodes = (m_tree.IsFull() && !m_tree.RemoveRandomLeaf(m_state.idxSolutionNode)))){
                return; // no more nodes available in the tree and no random leaf node could be removed
            }
            Node randomSample = Sampling(dataIn.finalPose, parameter);
            int16_t idxNearest = m_tree.FindNearestNode(randomSample, parameter.metric.weightPsi);
            Node& nearestSample = m_tree[idxNearest];
            auto [success, newSample] = LocalPlanning(nearestSample, randomSample, dataIn.staticObstacles, parameter);
            if(!success){
                return; // local planning failed (e.g. collision)
            }
            if(m_tree.FindNearestNeighbors(m_idxNeighbors, newSample, m_tree.GetOptimalRadius(), parameter.metric.weightPsi)){
                return; // value of the new sample already exists
            }
            int16_t idxBestParent = FindBestParent(newSample, idxNearest, dataIn.staticObstacles, parameter); // updates newSample.cost
            int16_t idxAddedSample = m_tree.Add(idxBestParent, newSample);
            Rewire(idxAddedSample, dataIn.staticObstacles, parameter);

            // Check if the new sample indicates a solution
            double distanceToGoal = mpsv::math::DistanceMetricSE2(m_tree[idxAddedSample].pose, dataIn.finalPose, parameter.metric.weightPsi);
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
         * @brief Generate a random sample.
         * @param[in] parameter The parameter set for path planning.
         * @return The random sample.
         */
        Node Sampling(const std::array<double,3>& finalPose, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            Node randomSample;
            if(m_state.goalReached || m_state.goalSampleIteration){
                randomSample.pose = m_boxSampler.Sample(m_state.sampleIteration++);
            }
            else{
                randomSample.pose = finalPose;
            }
            m_state.goalSampleIteration = (m_state.goalSampleIteration + 1) % parameter.pathPlanner.periodGoalSampling;
            return randomSample;
        }

        /**
         * @brief Perform local planning from a source node to a destination node and create a resulting node.
         * @param[in] source The source node from which to start the local planning.
         * @param[in] destination The destination node to which to steer to from the source node.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         * @param[in] parameter The parameter set for path planning.
         * @return A tuple containing <success, newSample>.
         * <0> True if local planning succeeded (e.g. no collision), false otherwise.
         * <1> The resulting sample of local planning.
         */
        std::tuple<bool, Node> LocalPlanning(const Node& source, const Node& destination, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            // Output tuple containing success and newSample
            std::tuple<bool, Node> result{};
            auto& [success, newSample] = result;

            // Limit direction to maximum step size
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

            // Resulting node is source + limited direction
            newSample.pose[0] = source.pose[0] + direction[0];
            newSample.pose[1] = source.pose[1] + direction[1];
            newSample.pose[2] = mpsv::math::SymmetricalAngle(source.pose[2] + direction[2]);

            // Check collision and calculate cost
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, parameter.metric.weightPsi);
            if(costDistance <= m_epsDistanceMetric){
                return result; // do not allow zero movement
            }
            if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, source.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                return result;
            }
            double costHeading = mpsv::math::SwayMetric(source.pose, newSample.pose, parameter.metric.weightSway, parameter.metric.weightReverseScale, parameter.metric.weightReverseDecay);
            double costClearance = m_costMap.CostAlongLine(source.pose, newSample.pose, parameter.geometry.skeletalPoints);
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
         * @param[in] parameter The parameter set for path planning.
         * @return Index to the best parent node such that the cost for the new sample becomes minimal and the edge from that parent to the new sample is feasible.
         */
        int16_t FindBestParent(Node& newSample, int16_t idxNearest, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
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
                        if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, possibleParent.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                            continue; // collision detected, cannot be a better parent
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
         * @param[in] parameter The parameter set for path planning.
         */
        void Rewire(int16_t idxAddedSample, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            Node& addedSample = m_tree[idxAddedSample];
            for(auto&& index : m_idxNeighbors){
                if(m_tree[idxAddedSample].iParent == index){
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
                        if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, destination.pose, addedSample.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                            continue; // collision detected, cannot be a better parent
                        }
                        double deltaCost = resultCost - destination.cost;
                        m_tree.Rewire(index, idxAddedSample, deltaCost);
                    }
                }
            }
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
