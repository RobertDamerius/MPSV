#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_planner_PathPlannerTree.hpp>
#include <mpsv_planner_PathPlannerParameterSet.hpp>
#include <mpsv_planner_PathPlannerState.hpp>
#include <mpsv_planner_PathPlannerInput.hpp>
#include <mpsv_planner_PathPlannerOutput.hpp>
#include <mpsv_planner_PathPlannerCostMap.hpp>
#include <mpsv_geometry_OrientedBox.hpp>
#include <mpsv_geometry_StaticObstacle.hpp>
#include <mpsv_geometry_ConvexPolygon.hpp>
#include <mpsv_sampler_UniformBoxSamplerSE2.hpp>
#include <mpsv_core_DataLogFile.hpp>
#include <mpsv_core_PerformanceCounter.hpp>
#include <mpsv_core_Time.hpp>
#include <mpsv_math_Additional.hpp>
#include <mpsv_math_Metric.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief The path planning algorithm. Before solving path planning problems, initialize the planner by calling the @ref Initialize member function.
 * Then apply a parameter set by calling @ref ApplyParameterSet. To solve a planning problem, call the two member functions @ref Prepare and
 * @ref Solve. The internal iteration loop of the @ref Solve operation is interrupted if @ref Interrupt is called. If no @ref Solve operation is running,
 * then the next one is to be interrupted.
 */
class PathPlanner {
    public:
        /**
         * @brief Construct a path planner and set default values.
         */
        PathPlanner() noexcept { Terminate(); }

        /**
         * @brief Destroy the path planner.
         */
        ~PathPlanner() noexcept { Terminate(); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initialization / Termination
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Initialize the path planner. This initializes the memory used by the path planner. Call this member function before calling any other function.
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
            if(!tree.Initialize(maxNumNodes) || !boxSampler.Initialize(maxNumSamples)){
                Terminate();
                return false;
            }
            return true;
        }

        /**
         * @brief Terminate the path planner. All containers and parameters are cleared. Call this function, if the path planner is no longer needed.
         */
        void Terminate(void) noexcept {
            interruptFlag = false;
            tree.Terminate();
            state.Clear();
            parameter.Clear();
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
         * @brief Apply a parameter set to the path planner.
         * @param[in] parameterSet The parameter set to be applied.
         * @return True if the input parameter set is valid and parameters have been applied, false otherwise.
         * @details Call this function after a successfull initialization (using the @ref Initialize member function). Do not call this function between @ref Prepare and @ref Solve calls, otherwise the parameters used by
         * both member functions may be inconsistent! If new parameters are to be applied, always apply them BEFORE a prepare-solve-step.
         */
        bool ApplyParameterSet(const mpsv::planner::PathPlannerParameterSet& parameter) noexcept {
            if(!parameter.IsValid()){
                return false;
            }
            this->parameter = parameter;
            state.Clear(true);
            return true;
        }

        /**
         * @brief Prepare the path planner before solving the path planning problem. The planner must be initialized using the @ref Initialize member function and a parameter set has to be applied using the @ref ApplyParameterSet member function.
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
            //    and the problem is infeasible. The prepare function returns.
            //
            // 5. WARM START / COLD START
            //    A warm start is performed to initialize the tree and keep as much nodes as possible from the previous tree. If a warm
            //    start is not possible, a cold start is performed. In this case the tree is reset and the initial pose is set as the
            //    root node.
            //
            // 6. ASSIGN OUTPUT
            //    The output of the path planner is updated. If a warm start was successfull, the solution path may contain a good initial
            //    path that should be visible in the output. However, the path is not smoothed.
            state.Clear(!forceColdStart); // keep latest solution if no cold start is forced
            state.initialPose = dataIn.initialPose;
            state.finalPose = dataIn.finalPose;
            mpsv::geometry::OrientedBox samplingArea = SetSamplingArea(dataIn.samplingBoxCenterPose, dataIn.samplingBoxDimension);
            SetEps(samplingArea);
            tree.SetCSpaceVolume(samplingArea, parameter.metric.weightPsi);
            BuildCostMap(samplingArea, dataIn.staticObstacles);
            RemoveObstaclesOutsideAreaOfInterest(dataIn.staticObstacles, samplingArea);
            state.isFeasible = !parameter.geometry.vehicleShape.CheckCollisionPose(dataIn.staticObstacles, state.initialPose); // initial pose must not collide for the problem to be feasible
            if(!state.isFeasible){
                state.idxSolutionNode = tree.ClearAndSetRoot(mpsv::planner::PathPlannerTreeNode(state.initialPose));
                state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(state.initialPose, state.finalPose, parameter.metric.weightPsi);
            }
            else{
                if(forceColdStart || !WarmStart(dataIn.originOldToNew, dataIn.staticObstacles)){
                    state.idxSolutionNode = tree.ClearAndSetRoot(mpsv::planner::PathPlannerTreeNode(state.initialPose));
                    state.closestDistanceToGoal = mpsv::math::DistanceMetricSE2(state.initialPose, state.finalPose, parameter.metric.weightPsi);
                    tree.ResetRandomNumberCounter();
                }
            }
            AssignOutput(dataOut);
        }

        /**
         * @brief Solve the path planning problem by doing several iterations for an upper computation time limit (the limit is set as parameter when calling this function). The planner must be initialized, parameters must be set and the planner must be prepared before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[in] dataIn The input data containing static obstacles. Only the static obstacles are used. All remaining data has already been processed by the @ref Prepare member function.
         * @param[in] maxComputationTime The maximum computation time in seconds allowed before leaving the iteration loop.
         * @details This member function implements an iteration loop. This loop is stopped, if the @ref interruptFlag is set via @ref Interrupt. The iteration loop is stopped
         * if the upper computation time limit is exceeded or if an error occurs. There are two types of errors: not feasible and out of nodes.
         * The problem is not feasible, if the inital pose already collides with obstacles. In this case no tree is built and the solution path would be
         * an empty set. However, the idxSolution always points to the root, thus the path returned in the output is non-empty (it contains at least the
         * root by definition). The second error outOfNodes is set, if no more nodes are available in the memory for more iterations. This only happens if
         * all nodes belong to the solution path. The number of nodes is limited. If the tree is full, a random node which must not belong to the solution path
         * is removed. If this is not possible, the internal memory of the tree is out of nodes.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::PathPlannerOutput& dataOut, mpsv::planner::PathPlannerInput& dataIn, double maxComputationTime) noexcept {
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
            }
            AssignOutput(dataOut);
            interruptFlag = false;
        }

        /**
         * @brief Solve the path planning problem by doing a fixed number of iterations. The planner must be initialized, parameters must be set and the planner must be prepared before calling this member function.
         * @param[out] dataOut The output data where to store results to.
         * @param[in] dataIn The input data containing static obstacles. Only the static obstacles are used. All remaining data has already been processed by the @ref Prepare member function.
         * @param[in] maxIterations The maximum number of iterations to be performed.
         * @details This member function implements an iteration loop. This loop is stopped before the maximum number of iterations is reached,  or
         * if the @ref interruptFlag is set via @ref Interrupt. In addition, the iteration loop is stopped before the maximum number of iterations if an error occurs.
         * There are two types of errors: not feasible and out of nodes. The problem is not feasible, if the inital pose already collides with
         * obstacles. In this case no tree is built and the solution path would be an empty set. However, the idxSolution always points to the
         * root, thus the path returned in the output is non-empty (it contains at least the root by definition). The second error outOfNodes
         * is set, if no more nodes are available in the memory for more iterations. This only happens if all nodes belong to the solution path.
         * The number of nodes is limited. If the tree is full, a random node which must not belong to the solution path is removed. If this
         * is not possible, the internal memory of the tree is out of nodes.
         * @note IMPORTANT: The caller of this function must make sure that all input values are valid!
         */
        void Solve(mpsv::planner::PathPlannerOutput& dataOut, mpsv::planner::PathPlannerInput& dataIn, uint32_t maxIterations) noexcept {
            if(state.isFeasible){
                for(uint32_t i = 0; i < maxIterations; ++i){
                    ++state.numberOfPerformedIterations;
                    Iterate(dataIn.staticObstacles);
                    if(interruptFlag || state.outOfNodes){
                        break;
                    }
                }
            }
            AssignOutput(dataOut);
            interruptFlag = false;
        }

        /**
         * @brief Interrupt a running @ref Solve operation of the path planner. If the @ref Solve operation is not running, then the next one
         * is to be interrupted.
         * @note The internal @ref interruptFlag is automatically reset at the end of a @ref Solve operation.
         */
        void Interrupt(void) noexcept { interruptFlag = true; }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Data Recording / Debugging
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept { tree.WriteToFile(file, preString + "tree."); parameter.WriteToFile(file, preString + "parameter."); costMap.WriteToFile(file, preString + "costMap."); }

    protected:
        mpsv::planner::PathPlannerTree tree;                // Tree for path planning.
        mpsv::planner::PathPlannerState state;              // State of the path planner.
        mpsv::planner::PathPlannerParameterSet parameter;   // Parameter set for the path planner.
        mpsv::planner::PathPlannerCostMap costMap;          // The cost map to be used as additional cost term in the objective function.
        mpsv::sampler::UniformBoxSamplerSE2 boxSampler;     // The box sampler to be used to draw random samples from SE(2).
        uint32_t maxNumSamples;                             // The maximum number of samples set by @ref Initialize.
        std::vector<int16_t> idxNeighbors;                  // Reserved memory for nearest neighbors.
        std::atomic<bool> interruptFlag;                    // If set to true, no more iterations are performed and the @ref Solve member function returns.
        double epsPosition;                                 // Eps (numeric threshold) for position.
        double epsAngle;                                    // Eps (numeric threshold) for angle.
        double epsDistanceMetric;                           // Eps (numeric threshold) for distance metric.

        /**
         * @brief Assign output data based on the internal @ref state and the @ref tree.
         * @param[out] dataOut Output data to be assigned.
         */
        void AssignOutput(mpsv::planner::PathPlannerOutput& dataOut) noexcept {
            dataOut.goalReached = state.goalReached;
            dataOut.isFeasible = state.isFeasible;
            dataOut.outOfNodes = state.outOfNodes;
            dataOut.numberOfPerformedIterations = state.numberOfPerformedIterations;
            dataOut.cost = tree.GetRefNode(state.idxSolutionNode).cost;
            tree.GetPathFromRootToNode(dataOut.path, state.idxSolutionNode);
            dataOut.timestampOfComputationUTC = mpsv::core::GetTimestampUTC();
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Prepare helper functions
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Set the sampling area.
         * @param[in] samplingBoxCenterPose Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
         * @param[in] samplingBoxDimension Dimension of the sampling box along major and minor axes of the box.
         * @return The oriented box representing the sampling area.
         */
        mpsv::geometry::OrientedBox SetSamplingArea(std::array<double,3> samplingBoxCenterPose, std::array<double,2> samplingBoxDimension) noexcept {
            mpsv::geometry::OrientedBox box;
            box.Create(samplingBoxCenterPose, samplingBoxDimension);
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
         * @brief Remove obstacles that are completely outside the new configuration space (sampling box + farest vehicle shape vertex).
         * @param[inout] staticObstacles Static obstacles to be considered. The content of the container may change.
         * @param[in] samplingArea The sampling area.
         */
        void RemoveObstaclesOutsideAreaOfInterest(std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, mpsv::geometry::OrientedBox samplingArea) noexcept {
            double boxExtension = parameter.geometry.vehicleShape.GetFarthestVertexDistance();
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
            for(auto&& p : parameter.geometry.skeletalPoints){
                maxSekeletalPointDistance = std::max(maxSekeletalPointDistance, std::sqrt(p[0]*p[0] + p[1]*p[1]));
            }
            double additionalBoxExtension = maxSekeletalPointDistance + 2.0 * parameter.costMap.resolution;
            samplingArea.Extend(additionalBoxExtension);
            mpsv::planner::PathPlannerCostMapArgument args(parameter.costMap.distanceScale, parameter.costMap.distanceDecay, staticObstacles);
            costMap.Build(samplingArea, parameter.costMap.resolution, parameter.costMap.modBreakpoints, args, maxNumCells);
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Warm start helper functions
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Prepare the algorithm with a warm start. This member function is called by the @ref Prepare memnber function.
         * @param[in] originOldToNew Translation between two consecutive problems. If the origin of the previous problem is different to the origin of the new problem, then the tree must be transformed during a warm start. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
         * @param[in] staticObstacles List of static obstacles to be taken into account for path planning.
         * @return True if warm start was successful, false otherwise. If this method fails, a cold start must be performed.
         */
        bool WarmStart(std::array<double,2> originOldToNew, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Step 1: Transform tree to new origin
            tree.TransformToNewOrigin(originOldToNew);

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 2: Connect initial pose with old solution path
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Get the current solution branch
            if(state.idxSolutionNode < 0) return false;
            std::vector<int16_t> idxSolutionPath = tree.GetBranchToRoot(state.idxSolutionNode);

            // If there's no path (at least two nodes), then we can make a cold start
            if(idxSolutionPath.size() < 2) return false;

            // Calculate the cummulative cost for all nodes along the given solution path based on the updated costmap
            // idxSolutionPath starts with the solution node and ends with the root node.
            tree.GetRefNode(idxSolutionPath.back()).tmp = 0.0;
            for(int32_t k = static_cast<int32_t>(idxSolutionPath.size())-1; k > 0; --k){
                auto& a = tree.GetRefNode(idxSolutionPath[k]);
                auto& b = tree.GetRefNode(idxSolutionPath[k - 1]);
                double costDistance = mpsv::math::DistanceMetricSE2(a.pose, b.pose, parameter.metric.weightPsi);
                double costHeading = mpsv::math::SwayMetric(a.pose, b.pose, parameter.metric.weightSway);
                double costClearance = costMap.CostAlongLine(a.pose, b.pose, parameter.geometry.skeletalPoints);
                b.tmp = a.tmp + (costDistance + costHeading + costClearance);
            }

            // Find best collision-free connection from initial pose to solution path: find that connection, that is collision-free and reduces the cost.
            int16_t idxBestConnection = -1;
            double bestTotalCost = std::numeric_limits<double>::infinity();
            for(auto&& i : idxSolutionPath){
                auto& node = tree.GetRefNode(i);
                if(!parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, node.pose, state.initialPose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                    // The total cost for this connection is the cost from the initial pose to the connection point added to the cost from the connection point to the solution (old solution)
                    double costDistance = mpsv::math::DistanceMetricSE2(state.initialPose, node.pose, parameter.metric.weightPsi);
                    double costHeading = mpsv::math::SwayMetric(state.initialPose, node.pose, parameter.metric.weightSway);
                    double costClearance = costMap.CostAlongLine(state.initialPose, node.pose, parameter.geometry.skeletalPoints);
                    double totalCost = (costDistance + costHeading + costClearance) + (tree.GetRefNode(idxSolutionPath[0]).tmp - node.tmp);
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
            int16_t idxRoot = tree.ChangeRoot(mpsv::planner::PathPlannerTreeNode(state.initialPose), idxBestConnection, epsPosition, epsAngle);
            if(idxRoot < 0) return false;


            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 4: Check feasibility of all edges and prune the tree (feasible if no collision and inside new sampling area)
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Set all flags to true and set the flag of the root node to false
            tree.SetFlags(true);
            tree.GetRefNode(idxRoot).flag = false;

            // Check all branches recursively and set flags to false for feasible edges, then remove all marked nodes
            RecursiveFeasibilityCheck(idxRoot, staticObstacles);
            tree.RemoveMarkedNodes();

            // Recalculate the cost for the whole tree
            // First the cost for all lines is calculated separately and stored in the cost value of a child. Then the RecursiveCostCalculation accumulates them along all branches.
            const std::vector<int16_t>& indices = tree.GetAllNodeIndices();
            if(indices.empty()) return false;
            #ifndef MPSV_DONT_USE_OMP
            #pragma omp parallel for if (indices.size() > 100)
            for(auto&& i : indices){
                auto& node = tree.GetRefNode(i);
                for(auto&& c : node.idxChilds){
                    auto& child = tree.GetRefNode(c);
                    child.cost = mpsv::math::DistanceMetricSE2(node.pose, child.pose, parameter.metric.weightPsi);
                    child.cost += mpsv::math::SwayMetric(node.pose, child.pose, parameter.metric.weightSway);
                    child.cost += costMap.CostAlongLine(node.pose, child.pose, parameter.geometry.skeletalPoints);
                }
            }
            #else
            for(auto&& i : indices){
                auto& node = tree.GetRefNode(i);
                for(auto&& c : node.idxChilds){
                    auto& child = tree.GetRefNode(c);
                    child.cost = mpsv::math::DistanceMetricSE2(node.pose, child.pose, parameter.metric.weightPsi);
                    child.cost += mpsv::math::SwayMetric(node.pose, child.pose, parameter.metric.weightSway);
                    child.cost += costMap.CostAlongLine(node.pose, child.pose, parameter.geometry.skeletalPoints);
                }
            }
            #endif
            tree.GetRefNode(idxRoot).cost = 0.0;
            RecursiveCostCalculation(idxRoot);


            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Step 5: Find closest node to new goal
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Set default state values
            state.goalReached = false;
            state.outOfNodes = false;
            state.closestDistanceToGoal = std::numeric_limits<double>::infinity();
            state.idxSolutionNode = 0;

            // Find closest distance to goal
            double distance;
            for(auto&& i : indices){
                distance = mpsv::math::DistanceMetricSE2(tree.GetRefNode(i).pose, state.finalPose, parameter.metric.weightPsi);
                if(distance < state.closestDistanceToGoal){
                    state.closestDistanceToGoal = distance;
                    state.idxSolutionNode = i;
                }
            }

            // Check if solution was found
            if(state.closestDistanceToGoal < epsDistanceMetric){
                state.goalReached = true;
            }
            return true;
        }

        /**
         * @brief Warm start helper function to check the feasibility of edges recursively.
         * @param[in] idxNode Node index from which to start the feasibility check.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         * @details An edge is feasible if no collision occurrs and the edge stays inside the new sampling area.
         */
        void RecursiveFeasibilityCheck(int16_t idxNode, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            auto& node = tree.GetRefNode(idxNode);
            std::vector<int16_t> childsToKeep;
            for(auto&& i : node.idxChilds){
                if(boxSampler.IsInside(tree.GetRefNode(i).pose) && !parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, tree.GetRefNode(i).pose, node.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                    childsToKeep.push_back(i);
                    tree.GetRefNode(i).flag = false;
                    RecursiveFeasibilityCheck(i, staticObstacles);
                }
            }
            node.idxChilds.swap(childsToKeep);
        }

        /**
         * @brief Warm start helper function to recalculate the cost of the whole tree recursively.
         * @param[in] idxNode Node index from which to start the cost calculation, e.g. the root node.
         */
        void RecursiveCostCalculation(int16_t idxNode) noexcept {
            auto& node = tree.GetRefNode(idxNode);
            for(auto&& i : node.idxChilds){
                auto& child = tree.GetRefNode(i);
                child.cost += node.cost;
                RecursiveCostCalculation(i);
            }
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Iteration helper functions
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Do one iteration of the RRT* path planning algorithm.
         * @param[in] staticObstacles The set of static obstacles to be used for collision checks.
         */
        void Iterate(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Fixed-Node RRT*: ensure that tree is not full by possibly removing a random node
            if((state.outOfNodes = !EnsureNonFullTree())){
                return;
            }

            // Sampling: generate random sample
            mpsv::planner::PathPlannerTreeNode randomSample = Sampling();

            // Vertex Selection: select nearest node
            int16_t idxNearest = tree.FindNearestNode(randomSample, parameter.metric.weightPsi);
            mpsv::planner::PathPlannerTreeNode& nearestSample = tree.GetRefNode(idxNearest);

            // Local Planning: steer from nearest sample to random sample
            auto [success, newSample] = LocalPlanning(nearestSample, randomSample, staticObstacles);
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
        mpsv::planner::PathPlannerTreeNode Sampling(void) noexcept {
            mpsv::planner::PathPlannerTreeNode randomSample;
            if(state.goalReached || state.goalSampleIteration){
                randomSample.pose = boxSampler.Sample(state.sampleIteration);
                state.sampleIteration = (state.sampleIteration + 1) % maxNumSamples;
            }
            else{
                randomSample.pose = state.finalPose;
            }
            state.goalSampleIteration = (state.goalSampleIteration + 1) % parameter.pathPlanner.periodGoalSampling;
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
        std::tuple<bool, mpsv::planner::PathPlannerTreeNode> LocalPlanning(const mpsv::planner::PathPlannerTreeNode& source, const mpsv::planner::PathPlannerTreeNode& destination, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            // Resulting output tuple containing success and newSample
            std::tuple<bool, mpsv::planner::PathPlannerTreeNode> result;
            bool& success = std::get<0>(result);
            mpsv::planner::PathPlannerTreeNode& newSample = std::get<1>(result);
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

            // Check collision and calculate cost
            if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, source.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                return result;
            }
            double costDistance = mpsv::math::DistanceMetricSE2(source.pose, newSample.pose, parameter.metric.weightPsi);
            double costHeading = mpsv::math::SwayMetric(source.pose, newSample.pose, parameter.metric.weightSway);
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
        int16_t FindBestParent(mpsv::planner::PathPlannerTreeNode& newSample, int16_t idxNearest, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            int16_t idxBestParent = idxNearest;
            for(auto&& index : idxNeighbors){
                // Ignore the nearest node which is already the parent for the new node, check all others
                if(idxNearest == index){
                    continue;
                }

                // Check if cost reduction is actually possible
                mpsv::planner::PathPlannerTreeNode& possibleParent = tree.GetRefNode(index);
                double resultCost = possibleParent.cost;
                resultCost += mpsv::math::DistanceMetricSE2(possibleParent.pose, newSample.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(possibleParent.pose, newSample.pose, parameter.metric.weightSway);
                if((resultCost + epsDistanceMetric) < newSample.cost){
                    resultCost += costMap.CostAlongLine(possibleParent.pose, newSample.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + epsDistanceMetric) < newSample.cost){
                        // Check if a better connection is collision-free and update cost and parent index
                        if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, newSample.pose, possibleParent.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
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
            mpsv::planner::PathPlannerTreeNode& addedSample = tree.GetRefNode(idxAddedSample);
            for(auto&& index : idxNeighbors){
                // Cannot be a better parent for its own parent
                if(tree.GetParentIndex(idxAddedSample) == index){
                    continue;
                }

                // Check if cost reduction is actually possible
                mpsv::planner::PathPlannerTreeNode& destination = tree.GetRefNode(index);
                double resultCost = addedSample.cost;
                resultCost += mpsv::math::DistanceMetricSE2(addedSample.pose, destination.pose, parameter.metric.weightPsi);
                resultCost += mpsv::math::SwayMetric(addedSample.pose, destination.pose, parameter.metric.weightSway);
                if((resultCost + epsDistanceMetric) < destination.cost){
                    resultCost += costMap.CostAlongLine(addedSample.pose, destination.pose, parameter.geometry.skeletalPoints);
                    if((resultCost + epsDistanceMetric) < destination.cost){
                        // Check if a better connection is collision-free and rewire
                        if(parameter.geometry.vehicleShape.CheckCollisionLine(staticObstacles, destination.pose, addedSample.pose, parameter.geometry.collisionCheckMaxAngleDeviation)){
                            continue;
                        }
                        double deltaCost = resultCost - destination.cost;
                        tree.Rewire(index, idxAddedSample, deltaCost);
                    }
                }
            }
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

