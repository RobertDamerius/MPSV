#pragma once


#include <cstdint>
#include <array>
#include <mpsv_mpsv.hpp>


/**
 * @brief This union defines the input data for the asynchronous online planner wrapper class.
 */
#pragma pack(push, 1)
union SerializationAsynchronousOnlinePlannerInputUnion {
    struct SerializationAsynchronousOnlinePlannerInputStruct {
        uint8_t enable;                                                                 // Non-zero if the asynchronous online planner should be enabled, zero if the planner should go into standby mode.
        uint8_t reset;                                                                  // Non-zero if the asynchronous online planner should be reset. A reset is ensured to be performed before the next internal solve operation of the planner.
        double timestamp;                                                               // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint corresponding to @ref initialStateAndInput.
        std::array<double,3> originLLA;                                                 // Geographical origin to which this input belongs to, given as {lat,lon,alt}.
        std::array<double,12> initialStateAndInput;                                     // Initial state and input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
        std::array<double,3> finalPose;                                                 // The final pose given as {x,y,psi}.
        std::array<double,3> samplingBoxCenterPose;                                     // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double,2> samplingBoxDimension;                                      // Dimension of the sampling box along major and minor axes of the box.
        uint16_t numStaticObstacles;                                                    // The number of static obstacles in range [0,1000].
        std::array<uint8_t,1000> numVerticesPerStaticObstacle;                          // The number of vertices for each static obstacle in range [3,20].
        std::array<std::array<std::array<double,2>,20>,1000> verticesStaticObstacles;   // Vertex data of the static obstacles.
    } data;
    uint8_t bytes[sizeof(SerializationAsynchronousOnlinePlannerInputUnion::SerializationAsynchronousOnlinePlannerInputStruct)];
};
#pragma pack(pop)


/**
 * @brief This union defines the parameter data for the asynchronous online planner wrapper class.
 */
#pragma pack(push, 1)
union SerializationAsynchronousOnlinePlannerParameterUnion {
    struct SerializationAsynchronousOnlinePlannerParameterStruct {
        double timestamp;                                                                  // The user-defined timestamp that indicates the parameters to be used. This parameter set is only applied, if this value changes.
        double timeoutInput;                                                               // Timeout in seconds for the input data.
        struct {
            struct {
                double collisionCheckMaxPositionDeviation;                                 // [Geometry] Maximum position deviation for path subdivision during collision checking. Must be at least 0.01 meters.
                double collisionCheckMaxAngleDeviation;                                    // [Geometry] Maximum angle deviation for path subdivision during collision checking. Must be at least 1 degree.
                uint8_t numPolygonsVehicleShape;                                           // [Geometry] Number of convex polygons that represent the vehicle shape in range [1,10].
                std::array<uint8_t,10> numVerticesVehicleShape;                            // [Geometry] Number of vertices of a convex polygon of the vehicle shape in range [3,20].
                std::array<std::array<std::array<double,2>,20>,10> verticesVehicleShape;   // [Geometry] Vertex data of the vehicle shape.
                uint8_t numSkeletalPoints;                                                 // [Geometry] Number of skeletal points in range [1,10].
                std::array<std::array<double,2>,10> skeletalPoints;                        // [Geometry] Skeletal points (b-frame) at which the cost map is to be evaluated. All points must be inside the vehicle shape.
            } geometry;
            struct {
                int32_t modBreakpoints;                                                    // [CostMap] A modulo factor (> 0) that indicates when to calculate the cost using the objective function and when to do bilinear interpolation.
                double resolution;                                                         // [CostMap] Resolution (> 1e-3) of the grid map (dimension of one cell).
                double distanceScale;                                                      // [CostMap] Scale factor (>= 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
                double distanceDecay;                                                      // [CostMap] Decay factor (> 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
            } costMap;
            struct {
                double weightPsi;                                                          // [Metric] Weighting for heading angle (psi) in distance metric function.
                double weightSway;                                                         // [Metric] Weighting for sway movement (heading angle with respect to perpenticular direction of movement).
            } metric;
            struct {
                std::array<double,36> matF;                                                // [Model] 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
                std::array<double,9> matB;                                                 // [Model] 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
                std::array<double,3> vecTimeconstantsXYN;                                  // [Model] Timeconstants {TX, TY, TN} for input force dynamics.
                std::array<double,3> vecTimeconstantsInput;                                // [Model] Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
                std::array<double,3> satXYN;                                               // [Model] Absolute elliptical saturation value for input vector u of model nu_dot = A*nu + f(nu) + B*tau.
            } model;
            struct {
                uint32_t periodGoalSampling;                                               // [PathPlanner] Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
            } pathPlanner;
            struct {
                double samplingRangePosition;                                              // [MotionPlanner] Range in meters for sampling the position around a given path.
                double samplingRangeAngle;                                                 // [MotionPlanner] Range in radians for sampling the angle around a given path.
                uint32_t periodGoalSampling;                                               // [MotionPlanner] Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
                double sampletime;                                                         // [MotionPlanner] Sampletime to be used for fixed-step trajectory simulation.
                double maxPositionOvershoot;                                               // [MotionPlanner] Maximum position overshoot due to dynamic motion between two states. This value is used to remove obstacles outside the area of interest.
                double maxInputPathLength;                                                 // [MotionPlanner] Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
                struct {
                    double maxRadiusX;                                                     // [MotionPlanner / Controller] Maximum look-ahead distance for longitudinal distance during pose control.
                    double maxRadiusY;                                                     // [MotionPlanner / Controller] Maximum look-ahead distance for lateral distance during pose control.
                    double maxRadiusPsi;                                                   // [MotionPlanner / Controller] Maximum look-ahead distance for angular distance during pose control.
                    double minRadiusPosition;                                              // [MotionPlanner / Controller] Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
                    std::array<double,9> vecTimeconstantsFlatStates;                       // [MotionPlanner / Controller] Timeconstants for flat states {Tu, Tv, Tr, Tu_dot, Tv_dot, Tr_dot, Tu_dotdot, Tv_dotdot, Tr_dotdot}.
                    std::array<double,36> matK;                                            // [MotionPlanner / Controller] 3-by-12 control gain matrix (row-major order) for pose control (state controller using underlying velocity controller based on feedback-linearization).
                    std::array<double,3> satUVR;                                           // [MotionPlanner / Controller] Absolute elliptical saturation value for velocity commands (u,v,r) from pose controller to underlying velocity controller.
                } controller;
                struct {
                    std::array<double,3> rangePose;                                        // [MotionPlanner / RegionOfAttraction] Pose box constraints for the region of attraction.
                    std::array<double,3> rangeUVR;                                         // [MotionPlanner / RegionOfAttraction] Velocity box constraints for the region of attraction.
                    std::array<double,3> rangeXYN;                                         // [MotionPlanner / RegionOfAttraction] The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.
                } regionOfAttraction;
            } motionPlanner;
        } sequentialPlanner;
        struct {
            uint8_t predictInitialStateOnReset;                                            // [OnlinePlanner] Non-zero if the initial state should be predicted by the expected computation time after a reset of the online planner (e.g. the first solve).
            double maxComputationTimePathOnReset;                                          // [OnlinePlanner] Maximum computation time in seconds for path planning after a reset.
            double maxComputationTimeMotionOnReset;                                        // [OnlinePlanner] Maximum computation time in seconds for motion planning after a reset.
            double maxComputationTimePath;                                                 // [OnlinePlanner] Maximum computation time in seconds for path planning.
            double maxComputationTimeMotion;                                               // [OnlinePlanner] Maximum computation time in seconds for motion planning.
            double additionalAheadPlanningTime;                                            // [OnlinePlanner] Additional time added to the estimated computation time in seconds to obtain the future timepoint from where to start the next planning problem. This value must be greater than zero.
            double additionalTrajectoryDuration;                                           // [OnlinePlanner] Additional time added to the ahead planning time (= execution time + additional ahead planning time) to obtain the minimum time duration of a trajectory. This value must be greater than zero.
            double timeKeepPastTrajectory;                                                 // [OnlinePlanner] Time in seconds to keep from a past trajectory. The past data of the previous trajectory is inserted at the beginning of a new solution. Inserting past data helps to handle imperfect time synchronization between this trajectory generator and the user of the trajectory data.
        } onlinePlanner;
    } data;
    uint8_t bytes[sizeof(SerializationAsynchronousOnlinePlannerParameterUnion::SerializationAsynchronousOnlinePlannerParameterStruct)];
};
#pragma pack(pop)


/**
 * @brief This union defines the output data for the asynchronous online planner wrapper class.
 */
#pragma pack(push, 1)
union SerializationAsynchronousOnlinePlannerOutputUnion {
    struct SerializationAsynchronousOnlinePlannerOutputStruct {
        double timestampInput;                                     // The user-defined timestamp of the corresponding input data that has been used to compute the solution. The default value is quiet_NaN.
        double timestampParameter;                                 // The user-defined timestamp of the corresponding parameter data that has been used to compute the solution. The default value is quiet_NaN.
        uint8_t timeoutInput;                                      // Non-zero if the given input data timed out, zero otherwise.
        uint8_t validInput;                                        // Non-zero if the given input data is valid, zero otherwise.
        uint8_t validParameter;                                    // Non-zero if the given parameter data is valid, zero otherwise.
        uint8_t threadState;                                       // The state of the planning thread (0: offline, 1: standby, 2: running).
        double timestamp;                                          // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint of the @ref trajectory.
        std::array<double,3> originLLA;                            // Geographical origin to which this outputs belongs to, given as {lat,lon,alt}.
        uint32_t numTrajectoryPoints;                              // The number of points inside the trajectory.
        std::array<std::array<double,12>,1000> trajectory;         // Resulting trajectory, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted. The actual length of the trajectory is given by numTrajectoryPoints.
        double sampletime;                                         // The sampletime of the trajectory data in seconds.
        uint8_t performedReset;                                    // Non-zero if reset has been performed, zero otherwise.
        uint8_t error;                                             // Non-zero if path or motion planner reported an error, e.g. not feasible or out of nodes, zero otherwise. This value is equal to (!pathPlanner.isFeasible || pathPlanner.outOfNodes || !motionPlanner.isFeasible || motionPlanner.outOfNodes).
        struct {
            uint32_t numPoses;                                     // [PathPlanner] The number of poses representing the path, e.g. the length of the path.
            std::array<std::array<double,3>,1000> path;            // [PathPlanner] Resulting path of the internal path planning problem, where each pose is given as {x,y,psi}. The actual length of the path is given by numPoses.
            uint8_t goalReached;                                   // [PathPlanner] Non-zero if goal is reached, zero otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            uint8_t isFeasible;                                    // [PathPlanner] Non-zero if problem is feasible, zero otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            uint8_t outOfNodes;                                    // [PathPlanner] Non-zero if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            uint32_t numberOfPerformedIterations;                  // [PathPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                      // [PathPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                           // [PathPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } pathPlanner;
        struct {
            uint32_t numPoses;                                     // [MotionPlanner] The number of poses representing the reference path, e.g. the length of the reference path.
            std::array<std::array<double,3>,1000> referencePath;   // [MotionPlanner] Resulting reference path of the internal motion planning problem, where each pose is given as {x,y,psi}. The actual length of the reference path is given by numPoses.
            double startingTimepoint;                              // [MotionPlanner] The starting timepoint of the internal motion planner trajectory (arbitrary time origin defined by the user, same as @ref timestamp).
            uint8_t goalReached;                                   // [MotionPlanner] Non-zero if goal is reached, zero otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            uint8_t isFeasible;                                    // [MotionPlanner] Non-zero if problem is feasible, zero otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            uint8_t outOfNodes;                                    // [MotionPlanner] Non-zero if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            uint32_t numberOfPerformedIterations;                  // [MotionPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                      // [MotionPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                           // [MotionPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } motionPlanner;
    } data;
    uint8_t bytes[sizeof(SerializationAsynchronousOnlinePlannerOutputUnion::SerializationAsynchronousOnlinePlannerOutputStruct)];
};
#pragma pack(pop)


/**
 * @brief This class represents a wrapper for the asynchronous online planner. It uses a custom IO data structure that is used to exchange data to simulink.
 */
class MPSV_WrapperAsynchronousOnlinePlanner {
    public:
        /**
         * @brief Construct a new wrapper object.
         */
        MPSV_WrapperAsynchronousOnlinePlanner();

        /**
         * @brief Destroy the wrapper object.
         * @details Calls the @ref Terminate member function.
         */
        ~MPSV_WrapperAsynchronousOnlinePlanner();

        /**
         * @brief Initialize the wrapper. The memory for the internal planner is allocated.
         * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planning). A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
         * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planning). A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
         * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planning). A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
         * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planning). A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
         * @param[in] threadPriority The priority to be set for the thread from 1 (low) to 99 (high).
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
         */
        void Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic);

        /**
         * @brief Terminate the wrapper. The memory for the internal planner is freed.
         */
        void Terminate(void);

        /**
         * @brief Perform one step of the wrapper. All data is forwarded to a separate thread that manages the actual planning.
         * @param[out] output Output where to store the latest results of the planning problem.
         * @param[in] input Input data defining the motion planning problem to be solved.
         * @param[in] parameter Parameter data to be set for the asynchronous online planner.
         */
        void Step(SerializationAsynchronousOnlinePlannerOutputUnion* output, SerializationAsynchronousOnlinePlannerInputUnion* input, SerializationAsynchronousOnlinePlannerParameterUnion* parameter);

    protected:
        mpsv::planner::AsyncOnlinePlanner planner;                        // The internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerInput plannerInput;              // Stores the input data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerParameterSet plannerParameter;   // Stores the parameter data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerOutput plannerOutput;            // Stores the output of the asynchronous online planner.

        /**
         * @brief Read the input data and assign it to @ref plannerInput.
         * @param[in] input Input data defining the motion planning problem to be solved.
         */
        void ReadFromInput(SerializationAsynchronousOnlinePlannerInputUnion* input);

        /**
         * @brief Read the parameter data and assign it to @ref plannerParameter.
         * @param[in] parameter Parameter data to be set for the asynchronous online planner.
         */
        void ReadFromParameter(SerializationAsynchronousOnlinePlannerParameterUnion* parameter);

        /**
         * @brief Write the @ref plannerOutput to the output data.
         * @param[out] output The output data to which to write the asynchronous online planner output.
         */
        void WriteToOutput(SerializationAsynchronousOnlinePlannerOutputUnion* output);
};

