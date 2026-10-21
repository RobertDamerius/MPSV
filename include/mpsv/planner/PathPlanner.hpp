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
 * Then apply a parameter set by calling @ref ApplyParameterSet. To solve a planning problem, call the two member functions @ref Prepare and
 * @ref Solve. The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called.
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
            m_interruptFlag = false;
            m_isSolving = false;
            m_tree.Clear();
            m_costMap.Clear();
            m_state.Clear();
            m_parameter.Clear();
            m_boxSampler.Terminate();
            m_idxNeighbors.clear();
            m_idxNeighbors.shrink_to_fit();
            m_epsDistanceMetric = std::numeric_limits<double>::epsilon();
        }

        /**
         * @brief Apply a parameter set to the path planner.
         * @param[in] parameterSet The parameter set to be applied.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         * @details Do not call this function between @ref Prepare and @ref Solve calls, otherwise the parameters used by
         * both member functions may be inconsistent! If new parameters are to be applied, always apply them BEFORE a prepare-solve-step.
         */
        error_code ApplyParameterSet(mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            error_code e = parameter.IsValid();
            if(error_code::NONE != e){
                return e;
            }
            m_parameter = parameter;
            m_state.Clear(true);
            return error_code::NONE;
        }

        /**
         * @brief Prepare the path planner before solving the path planning problem. A parameter set has to be applied using the @ref ApplyParameterSet member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[inout] dataIn The input data containing initial and final pose as well as static obstacles. All static obstacles that are outside the area of interest are removed! The input argument is changed.
         * @param[in] forceColdStart True if no warm start should be performed and the path planner starts with an empty tree, false otherwise. The default value is false.
         * @details This member function copies the initial and final pose to internal variables. The sampling area and cost look-up table are generated during this call. A warmstart is performed to use as much
         * information as possible from a previous solution.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Prepare(mpsv::planner::PathPlannerOutput& dataOut, mpsv::planner::PathPlannerInput& dataIn, bool forceColdStart = false) noexcept {
            // During the prepare step of the path planning algorithm, the following steps are performed:
            //
            // 1. SET SAMPLING AREA
            //    The 2-dimensional range for x- and y-positions to be sampled has to be defined. This sampling area should include the
            //    initial and final pose. The line between two poses defines the major axis of the sampling box. The actual dimension of
            //    the box is then calculated by the specified sampling box extension parameters from the current parameter set.
            //    The tree uses that box to calculate the volume of the C space which is required to calculate the optimal radius for near
            //    neighbor searches based on the cardinality of the tree.
            //
            // 2. BUILD COST MAP
            //    Samples are only drawn from the sampling area set during step 1. Thus, the area of the cost map can be reduced to an area
            //    of interest that is at least the size of the sampling area. Because the cost map is not only evaluated at one pose
            //    but also at two additional poses with distances L1, L2, the sampling area is extended to get the area for the cost map.
            //
            // 3. REMOVE STATIC OBSTACLES
            //    In order to increase performance, all static obstacles that are completely outside the area of interest, are removed.
            //    The area of interest is limited by the sampling area and the size of the static vehicle shape.
            //
            // 4. FEASIBILITY CHECK
            //    If the static vehicle shape at the initial pose already collides with static obstacles, no solution can be computed
            //    and the problem is infeasible. In addition, the initial and final pose must lie inside the sampling area.
            //
            // 5. WARM START / COLD START
            //    A warm start is performed to initialize the tree and keep as much nodes as possible from the previous tree. If a warm
            //    start is not possible, a cold start is performed. In this case the tree is reset and the initial pose is set as the
            //    root node.
            //
            // 6. ASSIGN OUTPUT
            //    The output of the path planner is updated. If a warm start was successfull, the solution path may contain a good initial
            //    path that should be visible in the output. However, the path is not smoothed.
            m_state.Clear(!forceColdStart); // keep latest solution if no cold start is forced
            m_state.initialPose = dataIn.initialPose;
            m_state.finalPose = dataIn.finalPose;
            mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn.samplingBoxCenterPose, dataIn.samplingBoxDimension);
            SetNumericLimits(samplingArea);
            m_tree.SetCSpaceVolume(samplingArea.GetVolume(m_parameter.metric.weightPsi));
            BuildCostMap(samplingArea, dataIn.staticObstacles);
            RemoveObstaclesOutsideAreaOfInterest(dataIn.staticObstacles, samplingArea);
            m_state.isFeasible = !m_parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, m_state.initialPose) && samplingArea.IsInside(m_state.initialPose) && samplingArea.IsInside(m_state.finalPose);
            if(!m_state.isFeasible){
                m_state.idxSolutionNode = m_tree.Reset(m_state.initialPose);
                m_state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(m_state.initialPose, m_state.finalPose, m_parameter.metric.weightPsi);
                m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
            }
            else{
                if(forceColdStart || !WarmStart(dataIn.originOldToNew, dataIn.staticObstacles)){
                    m_state.idxSolutionNode = m_tree.Reset(m_state.initialPose);
                    m_state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(m_state.initialPose, m_state.finalPose, m_parameter.metric.weightPsi);
                    m_state.goalReached = (m_state.closestDistanceToGoal < m_epsDistanceMetric);
                }
            }
            AssignOutput(dataOut);
        }

        /**
         * @brief Solve the path planning problem by doing several iterations for an upper computation time limit (the limit is set as parameter when calling this function). Parameters must be set and the planner must be prepared before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[in] dataIn The input data containing static obstacles. Only the static obstacles are used. All remaining data has already been processed by the @ref Prepare member function.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @details This member function implements an iteration loop. This loop is stopped, if the @ref m_interruptFlag is set via @ref Interrupt. The iteration loop is stopped
         * if the upper computation time limit is exceeded or if an error occurs. There are two types of errors: not feasible and out of nodes.
         * The problem is not feasible, if the inital pose already collides with obstacles. In this case no tree is built and the solution path would be
         * an empty set. However, the idxSolution always points to the root, thus the path returned in the output is non-empty (it contains at least the
         * root by definition). The second error outOfNodes is set, if no more nodes are available in the memory for more iterations. This only happens if
         * all nodes belong to the solution path. The number of nodes is limited. If the tree is full, a random node which must not belong to the solution path
         * is removed. If this is not possible, the internal memory of the tree is out of nodes.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::PathPlannerOutput& dataOut, mpsv::planner::PathPlannerInput& dataIn, double maxComputationTime) noexcept {
            m_isSolving = true;
            if(m_state.isFeasible){
                mpsv::runtime::PerformanceCounter executionCounter, iterationCounter;
                double maxIterationTime = 0.0; // maximum execution time of one iteration
                executionCounter.Start();
                while(!m_interruptFlag){
                    ++m_state.numberOfPerformedIterations;
                    iterationCounter.Start();
                    Iterate(dataIn.staticObstacles);
                    maxIterationTime = std::max(maxIterationTime, iterationCounter.TimeToStart());
                    if(m_state.outOfNodes || ((maxIterationTime + executionCounter.TimeToStart()) >= maxComputationTime)){
                        break;
                    }
                }
            }
            AssignOutput(dataOut);
            m_isSolving = false;
            m_interruptFlag = false;
        }

        /**
         * @brief Interrupt a running @ref Solve operation of the path planner.
         * @note The internal @ref m_interruptFlag is automatically reset at the end of a @ref Solve operation.
         */
        void Interrupt(void) noexcept { m_interruptFlag.store(m_isSolving); }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept { m_tree.WriteToFile(file, preString + "tree."); m_parameter.WriteToFile(file, preString + "parameter."); m_costMap.WriteToFile(file, preString + "costMap."); }

    protected:
        mpsv::structures::FixedSizeTreeSE2<Node> m_tree;                      // Tree for path planning.
        mpsv::planner::PathPlannerState m_state;                              // State of the path planner.
        mpsv::planner::PathPlannerParameterSet m_parameter;                   // Parameter set for the path planner.
        mpsv::structures::CostMap m_costMap;                                  // The cost map to be used as additional cost term in the objective function.
        mpsv::sampler::UniformBoxSamplerSE2 m_boxSampler;                     // The box sampler to be used to draw random samples from SE(2).
        std::vector<int16_t> m_idxNeighbors;                                  // Reserved memory for nearest neighbors.
        std::atomic<bool> m_interruptFlag{false};                             // If set to true, no more iterations are performed and the @ref Solve member function returns.
        std::atomic<bool> m_isSolving{false};                                 // True if the path planner performs a @ref Solve step, false otherwise.
        double m_epsDistanceMetric{std::numeric_limits<double>::epsilon()};   // Eps (numeric threshold) for distance metric.

        /**
         * @brief Assign output data based on the internal @ref m_state and the @ref m_tree.
         * @param[out] dataOut Output data to be assigned.
         */
        void AssignOutput(mpsv::planner::PathPlannerOutput& dataOut) noexcept {
            dataOut.goalReached = m_state.goalReached;
            dataOut.isFeasible = m_state.isFeasible;
            dataOut.outOfNodes = m_state.outOfNodes;
            dataOut.numberOfPerformedIterations = m_state.numberOfPerformedIterations;
            dataOut.cost = m_tree[m_state.idxSolutionNode].cost;
            m_tree.GetPathFromRootToNode(dataOut.path, m_state.idxSolutionNode);
            dataOut.timestampOfComputationUTC = mpsv::runtime::GetTimestampUTC();
        }

        /**
         * @brief Set the sampling area.
         * @param[in] samplingBoxCenterPose Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
         * @param[in] samplingBoxDimension Dimension of the sampling box along major and minor axes of the box.
         * @return The oriented box representing the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(std::array<double,3> samplingBoxCenterPose, std::array<double,2> samplingBoxDimension) noexcept {
            mpsv::geometry::OrientedBox box;
            box.Create(samplingBoxCenterPose, samplingBoxDimension);
            m_boxSampler.SetSize(box);
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
         * @brief Remove obstacles that are completely outside the new configuration space (sampling box + farest vehicle shape vertex).
         * @param[inout] staticObstacles Static obstacles to be considered. The content of the container may change.
         * @param[in] samplingArea The sampling area.
         */
        void RemoveObstaclesOutsideAreaOfInterest(std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, mpsv::geometry::OrientedBox samplingArea) noexcept {
            double boxExtension = m_parameter.geometry.vehicleShape.GetFarthestVertexDistance();
            samplingArea.Extend(boxExtension);
            mpsv::geometry::ConvexPolygon boxPolygon = samplingArea.ExportAsConvexPolygon();
            staticObstacles.erase(std::remove_if(staticObstacles.begin(), staticObstacles.end(), [&boxPolygon](mpsv::geometry::ConvexPolygon& p){ return !boxPolygon.Overlap(p); } ), staticObstacles.end());
        }

        /**
         * @brief Build the 2D-look-up table of the cost map.
         * @param[in] samplingArea The sampling area.
         * @param[in] staticObstacles Static obstacles to be considered for building the costmap.
         */
        void BuildCostMap(mpsv::geometry::OrientedBox samplingArea, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // The range must be at least the range of the sampling box. This box is further extended by the maximum distance of skeletal points for
            // cost map evaluation and by some safety threshold of two more cells.
            constexpr size_t maxNumCells = 125000000; // limit to about 1 GB
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
         * @brief Prepare the algorithm with a warm start. This member function is called by the @ref Prepare memnber function.
         * @param[in] originOldToNew Translation between two consecutive problems. If the origin of the previous problem is different
         * to the origin of the new problem, then the tree must be transformed during a warm start. This vector specifies the position
         * of the new origin with respect to the old origin (vector from old origin to new origin).
         * @param[in] staticObstacles List of static obstacles to be taken into account for path planning.
         * @return True if warm start was successful, false otherwise. If this method fails, a cold start must be performed.
         */
        bool WarmStart(std::array<double,2> originOldToNew, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Step 1: Transform tree to new origin
            m_tree.TransformToNewOrigin(originOldToNew);

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
                double costDistance = mpsv::math::DistanceMetricSE2(a.pose, b.pose, m_parameter.metric.weightPsi);
                double costHeading = mpsv::math::SwayMetric(a.pose, b.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                double costClearance = m_costMap.CostAlongLine(a.pose, b.pose, m_parameter.geometry.skeletalPoints);
                b.cost = a.cost + (costDistance + costHeading + costClearance);
            }

            // Find best collision-free connection from initial pose to solution path: find that connection, that is collision-free and reduces the cost.
            int16_t idxBestConnection = -1;
            double bestTotalCost = std::numeric_limits<double>::infinity();
            for(auto&& i : idxSolutionPathReverse){
                auto& node = m_tree[i];
                if(!m_parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, node.pose, m_state.initialPose, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
                    // The total cost for this connection is the cost from the initial pose to the connection point added to the cummulative cost from the connection point to the solution (old solution)
                    double costDistance = mpsv::math::DistanceMetricSE2(m_state.initialPose, node.pose, m_parameter.metric.weightPsi);
                    double costHeading = mpsv::math::SwayMetric(m_state.initialPose, node.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                    double costClearance = m_costMap.CostAlongLine(m_state.initialPose, node.pose, m_parameter.geometry.skeletalPoints);
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
            int16_t idxRoot = m_tree.ChangeRoot(m_state.initialPose, idxBestConnection, m_parameter.metric.weightPsi);
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
                    if(m_boxSampler.IsInside(b.pose) && !m_parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, b.pose, a.pose, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
                        if(m_tree[child].iFirstChild >= 0){
                            traversalStack.push_back(child);
                        }
                        b.cost = a.cost;
                        b.cost += mpsv::math::DistanceMetricSE2(a.pose, b.pose, m_parameter.metric.weightPsi);
                        b.cost += mpsv::math::SwayMetric(a.pose, b.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                        b.cost += m_costMap.CostAlongLine(a.pose, b.pose, m_parameter.geometry.skeletalPoints);
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
                distance = mpsv::math::DistanceMetricSE2(m_tree[i].pose, m_state.finalPose, m_parameter.metric.weightPsi);
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
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         */
        void Iterate(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            if((m_state.outOfNodes = (m_tree.IsFull() && !m_tree.RemoveRandomLeaf(m_state.idxSolutionNode)))){
                return; // no more nodes available in the tree and no random leaf node could be removed
            }
            Node randomSample = Sampling();
            int16_t idxNearest = m_tree.FindNearestNode(randomSample, m_parameter.metric.weightPsi);
            Node& nearestSample = m_tree[idxNearest];
            auto [success, newSample] = LocalPlanning(nearestSample, randomSample, staticObstacles);
            if(!success){
                return; // local planning failed (e.g. collision)
            }
            if(m_tree.FindNearestNeighbors(m_idxNeighbors, newSample, m_tree.GetOptimalRadius(), m_parameter.metric.weightPsi)){
                return; // value of the new sample already exists
            }
            int16_t idxBestParent = FindBestParent(newSample, idxNearest, staticObstacles); // updates newSample.cost
            int16_t idxAddedSample = m_tree.Add(idxBestParent, newSample);
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
         * @brief Generate a random sample.
         * @return The random sample.
         */
        Node Sampling(void) noexcept {
            Node randomSample;
            if(m_state.goalReached || m_state.goalSampleIteration){
                randomSample.pose = m_boxSampler.Sample(m_state.sampleIteration++);
            }
            else{
                randomSample.pose = m_state.finalPose;
            }
            m_state.goalSampleIteration = (m_state.goalSampleIteration + 1) % m_parameter.pathPlanner.periodGoalSampling;
            return randomSample;
        }

        /**
         * @brief Perform local planning from a source node to a destination node and create a resulting node.
         * @param[in] source The source node from which to start the local planning.
         * @param[in] destination The destination node to which to steer to from the source node.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         * @return A tuple containing <success, newSample>.
         * <0> True if local planning succeeded (e.g. no collision), false otherwise.
         * <1> The resulting sample of local planning.
         */
        std::tuple<bool, Node> LocalPlanning(const Node& source, const Node& destination, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Output tuple containing success and newSample
            std::tuple<bool, Node> result{};
            auto& [success, newSample] = result;

            // Limit direction to maximum step size
            std::array<double, 3> direction = mpsv::math::PoseDifference(destination.pose, source.pose);
            double z = m_parameter.metric.weightPsi * direction[2];
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
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, m_parameter.metric.weightPsi);
            if(costDistance <= m_epsDistanceMetric){
                return result; // do not allow zero movement
            }
            if(m_parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, source.pose, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
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
                if(idxNearest == index){
                    continue; // the nearest node is already the parent for the new node, check all others
                }

                // Check if cost reduction is actually possible
                Node& possibleParent = m_tree[index];
                double resultCost = possibleParent.cost;
                resultCost += mpsv::math::DistanceMetricSE2(possibleParent.pose, newSample.pose, m_parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(possibleParent.pose, newSample.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < newSample.cost){
                    resultCost += m_costMap.CostAlongLine(possibleParent.pose, newSample.pose, m_parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < newSample.cost){
                        if(m_parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, possibleParent.pose, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
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
         */
        void Rewire(int16_t idxAddedSample, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            Node& addedSample = m_tree[idxAddedSample];
            for(auto&& index : m_idxNeighbors){
                if(m_tree[idxAddedSample].iParent == index){
                    continue; // cannot be a better parent for its own parent
                }

                // Check if cost reduction is actually possible
                Node& destination = m_tree[index];
                double resultCost = addedSample.cost;
                resultCost += mpsv::math::DistanceMetricSE2(addedSample.pose, destination.pose, m_parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(addedSample.pose, destination.pose, m_parameter.metric.weightSway, m_parameter.metric.weightReverseScale, m_parameter.metric.weightReverseDecay);
                if((resultCost + m_epsDistanceMetric) < destination.cost){
                    resultCost += m_costMap.CostAlongLine(addedSample.pose, destination.pose, m_parameter.geometry.skeletalPoints);
                    if((resultCost + m_epsDistanceMetric) < destination.cost){
                        if(m_parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, destination.pose, addedSample.pose, m_parameter.geometry.collisionCheckMaxAngleDeviation)){
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
