#pragma once


#include <cstdint>
#include <array>
#include <mpsv_mpsv.hpp>


/**
 * @brief This union defines the input data for the path planner wrapper class.
 */
#pragma pack(push, 1)
union SerializationPathPlannerInputUnion {
    struct SerializationPathPlannerInputStruct {
        struct {
            struct {
                int32_t modBreakpoints;                                                    // A modulo factor (> 0) that indicates when to calculate the cost using the cost function and when to do linear interpolation.
                double resolution;                                                         // Resolution (> 1e-3) of the grid map (dimension of one cell).
                double distanceScale;                                                      // Scale factor (>= 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
                double distanceDecay;                                                      // Decay factor (> 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
            } costMap;
            struct {
                double collisionCheckMaxPositionDeviation;                                 // Maximum position deviation for path subdivision during collision checking. Must be at least 0.01 meters.
                double collisionCheckMaxAngleDeviation;                                    // Maximum angle deviation for path subdivision during collision checking. Must be at least 1 degree.
                uint8_t numSkeletalPoints;                                                 // Number of skeletal points in range [1,10].
                std::array<std::array<double,2>,10> skeletalPoints;                        // Skeletal points (b-frame) at which the cost map is to be evaluated. All points must be inside the vehicle shape.
                uint8_t numPolygonsVehicleShape;                                           // Number of convex polygons that represent the vehicle shape in range [1,10].
                std::array<uint8_t,10> numVerticesVehicleShape;                            // Number of vertices of a convex polygon of the vehicle shape in range [3,20].
                std::array<std::array<std::array<double,2>,20>,10> verticesVehicleShape;   // Vertex data of the vehicle shape.
            } geometry;
            struct {
                double weightPsi;                                                          // Weighting for yaw angle in distance metric function.
                double weightSway;                                                         // Weighting for sway movement (heading angle with respect to perpenticular direction of movement).
            } metric;
            struct {
                uint32_t periodGoalSampling;                                               // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
                double maxComputationTime;                                                 // The maximum computation time in seconds allowed before leaving the iteration loop.
            } pathPlanner;
        } parameter;
        std::array<double,3> initialPose;                                                  // Initial pose given as {x,y,psi}.
        std::array<double,3> finalPose;                                                    // Final pose given as {x,y,psi}.
        std::array<double,2> originOldToNew;                                               // Translation between two consecutive problems. If the origin of the previous problem is different to the origin of the new problem, then the tree must be transformed during a warm start. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
        std::array<double,3> samplingBoxCenterPose;                                        // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double,2> samplingBoxDimension;                                         // Dimension of the sampling box along major and minor axes of the box.
        uint16_t numStaticObstacles;                                                       // The number of static obstacles in range [0,1000].
        std::array<uint8_t,1000> numVerticesPerStaticObstacle;                             // The number of vertices for each static obstacle in range [3,20].
        std::array<std::array<std::array<double,2>,20>,1000> verticesStaticObstacles;      // Vertex data of the static obstacles.
    } data;
    uint8_t bytes[sizeof(SerializationPathPlannerInputUnion::SerializationPathPlannerInputStruct)];
};
#pragma pack(pop)


/**
 * @brief This union defines the output data for the path planner wrapper class.
 */
#pragma pack(push, 1)
union SerializationPathPlannerOutputUnion {
    struct SerializationPathPlannerOutputStruct {
        uint8_t goalReached:1;                        // True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
        uint8_t isFeasible:1;                         // True if problem is feasible, false otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
        uint8_t outOfNodes:1;                         // True if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
        uint8_t invalidInput:1;                       // True if input data is invalid, false otherwise.
        uint8_t reserved:4;                           // Reserved bits, unused, set to zero.
        uint32_t numberOfPerformedIterations;         // The total number of iterations that have been performed since the latest prepare step.
        double timestampOfComputationUTC;             // Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
        double cost;                                  // The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        uint16_t numPoses;                            // The total number of poses that represent the actual path.
        std::array<std::array<double,3>,1000> path;   // Resulting path, where each pose is given as {x,y,psi}. The first element always corresponds to the initial pose. If the goal is reached, then the final pose corresponds to the desired final pose of the path planning problem.
    } data;
    uint8_t bytes[sizeof(SerializationPathPlannerOutputUnion::SerializationPathPlannerOutputStruct)];
};
#pragma pack(pop)


/**
 * @brief This class represents a wrapper for the path planner. It uses a custom IO data structure that is used to exchange data to simulink.
 */
class MPSV_WrapperPathPlanner {
    public:
        /**
         * @brief Construct a new wrapper object.
         */
        MPSV_WrapperPathPlanner(){ initializationOK = false; maxComputationTime = 0.0; }

        /**
         * @brief Destroy the wrapper object.
         * @details Calls the @ref Terminate member function.
         */
        ~MPSV_WrapperPathPlanner(){ Terminate(); }

        /**
         * @brief Initialize the wrapper. The memory for the internal path planner is allocated.
         * @param[in] maxNumNodes Maximum number of nodes to be used in the tree. A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
         * @param[in] maxNumSamples Maximum number of random samples to be generated in advance. A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
         */
        void Initialize(int16_t maxNumNodes, uint32_t maxNumSamples);

        /**
         * @brief Terminate the wrapper. The memory for the internal path planner is freed.
         */
        void Terminate(void);

        /**
         * @brief Perform one step of the wrapper. The parameters are applied to the internal path planner and the problem is solved using the input data.
         * @param[out] output Output where to store the results of the solved path planning problem.
         * @param[in] input Input data defining the path planning problem to be solved.
         */
        void Step(SerializationPathPlannerOutputUnion* output, SerializationPathPlannerInputUnion* input);

    protected:
        bool initializationOK;                                         // True if initialization was OK, false otherwise.
        mpsv::planner::PathPlanner pathPlanner;                        // The internal path planner.
        mpsv::planner::PathPlannerInput pathPlannerInput;              // The input data to the path planner.
        mpsv::planner::PathPlannerOutput pathPlannerOutput;            // The output data from the path planner.
        mpsv::planner::PathPlannerParameterSet pathPlannerParameter;   // The parameter set for the path planner.
        double maxComputationTime;                                     // The maximum computation time in seconds allowed before leaving the iteration loop.

        /**
         * @brief Assign the input data to the internal path planner data.
         * @param[in] input Input data defining the path planning problem to be solved.
         * @return True if the input data is valid, false otherwise.
         */
        bool AssignInput(SerializationPathPlannerInputUnion* input);

        /**
         * @brief Assign the output data from the internal path planner data.
         * @param[out] output Output where to store the results of the solved path planning problem.
         */
        void AssignOutput(SerializationPathPlannerOutputUnion* output);

        /**
         * @brief Clear the output data by setting default values.
         * @param[inout] output Output where to store the results of the solved path planning problem.
         * @param[in] validInput True if the input data is valid, false otherwise.
         */
        void ClearOutput(SerializationPathPlannerOutputUnion* output, bool validInput);
};

