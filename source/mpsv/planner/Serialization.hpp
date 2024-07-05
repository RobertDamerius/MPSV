#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/AsyncOnlinePlannerInput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerOutput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerParameterSet.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This union defines a constant-size input data for an asynchronous online planner.
 */
#pragma pack(push, 1)
union SerializationAsyncOnlinePlannerInputUnion {
    struct SerializationAsyncOnlinePlannerInputStruct {
        double timestamp;                                                               // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint corresponding to @ref initialStateAndInput.
        uint8_t enable:1;                                                               // Bit 0: non-zero if the asynchronous online planner should be enabled, zero if the planner should go into standby mode.
        uint8_t reset:1;                                                                // Bit 1: non-zero if the asynchronous online planner should be reset. A reset is ensured to be performed before the next internal solve operation of the planner.
        uint8_t reserved:6;                                                             // Bit 2-7: reserved, always zero.
        std::array<double,3> originLLA;                                                 // Geographical origin to which this input belongs to, given as {lat,lon,alt}.
        std::array<double,12> initialStateAndInput;                                     // Initial state and input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
        std::array<double,3> finalPose;                                                 // The final pose given as {x,y,psi}.
        std::array<double,3> samplingBoxCenterPose;                                     // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double,2> samplingBoxDimension;                                      // Dimension of the sampling box along major and minor axes of the box.
        uint16_t numStaticObstacles;                                                    // The number of static obstacles in range [0,400].
        std::array<uint8_t,400> numVerticesPerStaticObstacle;                           // The number of vertices for each static obstacle in range [3,20].
        std::array<std::array<std::array<float,2>,20>,400> verticesStaticObstacles;     // Vertex data of the static obstacles.
    } data;
    uint8_t bytes[sizeof(mpsv::planner::SerializationAsyncOnlinePlannerInputUnion::SerializationAsyncOnlinePlannerInputStruct)];
};
#pragma pack(pop)


/**
 * @brief This union defines a constant-size parameter data for an asynchronous online planner.
 */
#pragma pack(push, 1)
union SerializationAsyncOnlinePlannerParameterUnion {
    struct SerializationAsyncOnlinePlannerParameterStruct {
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
                double weightReverseScale;                                                 // [Metric] Weighting for sway and reverse movement (heading angle with respect to line angle).
                double weightReverseDecay;                                                 // [Metric] Decay factor (> 0) for the weighting function that weights sway and reverse movement.
            } metric;
            struct {
                std::array<double,36> matF;                                                // [Model] 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
                std::array<double,9> matB;                                                 // [Model] 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
                std::array<double,3> vecTimeconstantsXYN;                                  // [Model] Timeconstants {TX, TY, TN} for input force dynamics.
                std::array<double,3> vecTimeconstantsInput;                                // [Model] Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
                std::array<double,3> lowerLimitXYN;                                        // [Model] Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
                std::array<double,3> upperLimitXYN;                                        // [Model] Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
            } model;
            struct {
                uint32_t periodGoalSampling;                                               // [PathPlanner] Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
            } pathPlanner;
            struct {
                double samplingRangePosition;                                              // [MotionPlanner] Range in meters for sampling the position around a given path.
                double samplingRangeAngle;                                                 // [MotionPlanner] Range in radians for sampling the angle around a given path.
                uint32_t periodGoalSampling;                                               // [MotionPlanner] Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
                double sampletime;                                                         // [MotionPlanner] Sampletime to be used for fixed-step trajectory simulation.
                double maxInputPathLength;                                                 // [MotionPlanner] Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
                struct {
                    double maxRadiusX;                                                     // [MotionPlanner / Controller] Maximum look-ahead distance for longitudinal distance during pose control.
                    double maxRadiusY;                                                     // [MotionPlanner / Controller] Maximum look-ahead distance for lateral distance during pose control.
                    double maxRadiusPsi;                                                   // [MotionPlanner / Controller] Maximum look-ahead distance for angular distance during pose control.
                    double minRadiusPosition;                                              // [MotionPlanner / Controller] Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
                    std::array<double,36> matK;                                            // [MotionPlanner / Controller] 3-by-12 control gain matrix (row-major order) for pose control.
                } controller;
                struct {
                    std::array<double,3> rangePose;                                        // [MotionPlanner / RegionOfAttraction] Pose box constraints for the region of attraction.
                    std::array<double,3> rangeUVR;                                         // [MotionPlanner / RegionOfAttraction] Velocity box constraints for the region of attraction.
                    std::array<double,3> rangeXYN;                                         // [MotionPlanner / RegionOfAttraction] The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.
                } regionOfAttraction;
            } motionPlanner;
        } sequentialPlanner;
        struct {
            uint8_t predictInitialStateOnReset:1;                                          // [OnlinePlanner] Bit 0: non-zero if the initial state should be predicted by the expected computation time after a reset of the online planner (e.g. the first solve).
            uint8_t reserved:7;                                                            // [OnlinePlanner] Bit 1-7: reserved, always zero.
            double maxComputationTimePathOnReset;                                          // [OnlinePlanner] Maximum computation time in seconds for path planning after a reset.
            double maxComputationTimeMotionOnReset;                                        // [OnlinePlanner] Maximum computation time in seconds for motion planning after a reset.
            double maxComputationTimePath;                                                 // [OnlinePlanner] Maximum computation time in seconds for path planning.
            double maxComputationTimeMotion;                                               // [OnlinePlanner] Maximum computation time in seconds for motion planning.
            double additionalAheadPlanningTime;                                            // [OnlinePlanner] Additional time added to the estimated computation time in seconds to obtain the future timepoint from where to start the next planning problem. This value must be greater than zero.
            double additionalTrajectoryDuration;                                           // [OnlinePlanner] Additional time added to the ahead planning time (= execution time + additional ahead planning time) to obtain the minimum time duration of a trajectory. This value must be greater than zero.
            double timeKeepPastTrajectory;                                                 // [OnlinePlanner] Time in seconds to keep from a past trajectory. The past data of the previous trajectory is inserted at the beginning of a new solution. Inserting past data helps to handle imperfect time synchronization between this trajectory generator and the user of the trajectory data.
        } onlinePlanner;
    } data;
    uint8_t bytes[sizeof(mpsv::planner::SerializationAsyncOnlinePlannerParameterUnion::SerializationAsyncOnlinePlannerParameterStruct)];
};
#pragma pack(pop)


/**
 * @brief This union defines the constant-size output data for an asynchronous online planner.
 */
#pragma pack(push, 1)
union SerializationAsyncOnlinePlannerOutputUnion {
    struct SerializationAsyncOnlinePlannerOutputStruct {
        double timestamp;                                          // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint of the @ref trajectory.
        double timestampInput;                                     // The user-defined timestamp of the corresponding input data that has been used to compute the solution. The default value is quiet_NaN.
        double timestampParameter;                                 // The user-defined timestamp of the corresponding parameter data that has been used to compute the solution. The default value is quiet_NaN.
        uint8_t threadState:2;                                     // Bit 0-1: the state of the planning thread (0: offline, 1: standby, 2: running).
        uint8_t timeoutInput:1;                                    // Bit 2: non-zero if the given input data timed out, zero otherwise.
        uint8_t validInput:1;                                      // Bit 3: non-zero if the given input data is valid, zero otherwise.
        uint8_t validParameter:1;                                  // Bit 4: non-zero if the given parameter data is valid, zero otherwise.
        uint8_t performedReset:1;                                  // Bit 5: non-zero if reset has been performed, zero otherwise.
        uint8_t error:1;                                           // Bit 6: non-zero if path or motion planner reported an error, e.g. not feasible or out of nodes, zero otherwise. This value is equal to (!pathPlanner.isFeasible || pathPlanner.outOfNodes || !motionPlanner.isFeasible || motionPlanner.outOfNodes).
        uint8_t trajectoryShrinked:1;                              // Bit 7: non-zero if trajectory has been shrinked to fit to the memory size.
        std::array<double,3> originLLA;                            // Geographical origin to which this outputs belongs to, given as {lat,lon,alt}.
        uint16_t numTrajectoryPoints;                              // The number of points inside the trajectory.
        std::array<std::array<double,12>,500> trajectory;          // Resulting trajectory, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted. The actual length of the trajectory is given by numTrajectoryPoints.
        double sampletime;                                         // The sampletime of the trajectory data in seconds.
        struct {
            uint16_t numPoses;                                     // [PathPlanner] The number of poses representing the path, e.g. the length of the path.
            std::array<std::array<double,3>,100> path;             // [PathPlanner] Resulting path of the internal path planning problem, where each pose is given as {x,y,psi}. The actual length of the path is given by numPoses.
            uint8_t goalReached:1;                                 // [PathPlanner] Bit 0: non-zero if goal is reached, zero otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            uint8_t isFeasible:1;                                  // [PathPlanner] Bit 1: non-zero if problem is feasible, zero otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            uint8_t outOfNodes:1;                                  // [PathPlanner] Bit 2: non-zero if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            uint8_t pathShrinked:1;                                // [PathPlanner] Bit 3: non-zero if path has been shrinked to fit to the memory size.
            uint8_t reserved:4;                                    // [PathPlanner] Bit 4-7: reserved, always zero.
            uint32_t numberOfPerformedIterations;                  // [PathPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                      // [PathPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                           // [PathPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } pathPlanner;
        struct {
            uint16_t numPoses;                                     // [MotionPlanner] The number of poses representing the reference path, e.g. the length of the reference path.
            std::array<std::array<double,3>,100> referencePath;    // [MotionPlanner] Resulting reference path of the internal motion planning problem, where each pose is given as {x,y,psi}. The actual length of the reference path is given by numPoses.
            uint8_t goalReached:1;                                 // [MotionPlanner] Bit 0: non-zero if goal is reached, zero otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            uint8_t isFeasible:1;                                  // [MotionPlanner] Bit 1: non-zero if problem is feasible, zero otherwise. The problem is not feasible, if the initial pose already collides with static obstacles.
            uint8_t outOfNodes:1;                                  // [MotionPlanner] Bit 2: non-zero if all nodes are within the solution path and no new nodes can be sampled and added to the tree.
            uint8_t referencePathShrinked:1;                       // [MotionPlanner] Bit 3: non-zero if referencePath has been shrinked to fit to the memory size.
            uint8_t reserved:4;                                    // [MotionPlanner] Bit 4-7: reserved, always zero.
            uint32_t numberOfPerformedIterations;                  // [MotionPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC;                      // [MotionPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost;                                           // [MotionPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } motionPlanner;
    } data;
    uint8_t bytes[sizeof(mpsv::planner::SerializationAsyncOnlinePlannerOutputUnion::SerializationAsyncOnlinePlannerOutputStruct)];
};
#pragma pack(pop)


/**
 * @brief Deserialize input data for the asynchronous online planner.
 * @param[out] plannerInput Input structure for the asynchronous online planner.
 * @param[in] input Serialized input data.
 */
inline void Deserialize(mpsv::planner::AsyncOnlinePlannerInput& plannerInput, const mpsv::planner::SerializationAsyncOnlinePlannerInputUnion* input){
    plannerInput.enable                = input->data.enable;
    plannerInput.reset                 = input->data.reset;
    plannerInput.timestamp             = input->data.timestamp;
    plannerInput.originLLA             = input->data.originLLA;
    plannerInput.initialStateAndInput  = input->data.initialStateAndInput;
    plannerInput.finalPose             = input->data.finalPose;
    plannerInput.samplingBoxCenterPose = input->data.samplingBoxCenterPose;
    plannerInput.samplingBoxDimension  = input->data.samplingBoxDimension;
    plannerInput.staticObstacles.clear();
    bool validStaticObstacles = (input->data.numStaticObstacles <= input->data.numVerticesPerStaticObstacle.size());
    for(size_t p = 0; (p < static_cast<size_t>(input->data.numStaticObstacles)) && validStaticObstacles; ++p){
        validStaticObstacles &= (input->data.numVerticesPerStaticObstacle[p] > 2) && (input->data.numVerticesPerStaticObstacle[p] <= input->data.verticesStaticObstacles[p].size());
        std::vector<std::array<double,2>> vertices(input->data.numVerticesPerStaticObstacle[p]);
        std::transform(input->data.verticesStaticObstacles[p].begin(), input->data.verticesStaticObstacles[p].begin() + input->data.numVerticesPerStaticObstacle[p], vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
        plannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
        validStaticObstacles &= plannerInput.staticObstacles.back().EnsureCorrectVertexOrder();
    }

    // if there're errors make whole input invalid (set timestampt to NaN)
    if(!validStaticObstacles){
        plannerInput.timestamp = std::numeric_limits<double>::quiet_NaN();
    }
}


/**
 * @brief Deserialize parameter data for the asynchronous online planner.
 * @param[out] plannerParameter Parameter structure for the asynchronous online planner.
 * @param[in] parameter Serialized parameter data.
 */
inline void Deserialize(mpsv::planner::AsyncOnlinePlannerParameterSet& plannerParameter, const mpsv::planner::SerializationAsyncOnlinePlannerParameterUnion* parameter){
    plannerParameter.timestamp                                                             = parameter->data.timestamp;
    plannerParameter.timeoutInput                                                          = parameter->data.timeoutInput;
    plannerParameter.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation         = parameter->data.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation;
    plannerParameter.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation            = parameter->data.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation;

    // vehicle shape
    plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
    bool validVehicleShape = (parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape > 0) && (parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape <= parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape.size());
    if(validVehicleShape){
        for(uint8_t p = 0; p < parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape; ++p){
            validVehicleShape &= (parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p] > 2) && (parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p] <= parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].size());
            if(!validVehicleShape){
                break;
            }
            for(uint8_t v = 0; v < parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p]; ++v){
                std::vector<std::array<double,2>> vertices(parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].begin(), parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].begin() + parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p]);
                plannerParameter.sequentialPlanner.geometry.vehicleShape.Add(vertices);
            }
        }
        validVehicleShape &= plannerParameter.sequentialPlanner.geometry.vehicleShape.EnsureCorrectVertexOrder();
    }

    // skeletal points
    plannerParameter.sequentialPlanner.geometry.skeletalPoints.clear();
    bool validSkeletalPoints = (parameter->data.sequentialPlanner.geometry.numSkeletalPoints > 0) && (parameter->data.sequentialPlanner.geometry.numSkeletalPoints <= parameter->data.sequentialPlanner.geometry.skeletalPoints.size());
    if(validSkeletalPoints){
        plannerParameter.sequentialPlanner.geometry.skeletalPoints.resize(parameter->data.sequentialPlanner.geometry.numSkeletalPoints);
        for(uint8_t k = 0; (k < parameter->data.sequentialPlanner.geometry.numSkeletalPoints); ++k){
            plannerParameter.sequentialPlanner.geometry.skeletalPoints[k] = parameter->data.sequentialPlanner.geometry.skeletalPoints[k];
        }
    }

    plannerParameter.sequentialPlanner.costMap.modBreakpoints                              = parameter->data.sequentialPlanner.costMap.modBreakpoints;
    plannerParameter.sequentialPlanner.costMap.resolution                                  = parameter->data.sequentialPlanner.costMap.resolution;
    plannerParameter.sequentialPlanner.costMap.distanceScale                               = parameter->data.sequentialPlanner.costMap.distanceScale;
    plannerParameter.sequentialPlanner.costMap.distanceDecay                               = parameter->data.sequentialPlanner.costMap.distanceDecay;
    plannerParameter.sequentialPlanner.metric.weightPsi                                    = parameter->data.sequentialPlanner.metric.weightPsi;
    plannerParameter.sequentialPlanner.metric.weightSway                                   = parameter->data.sequentialPlanner.metric.weightSway;
    plannerParameter.sequentialPlanner.metric.weightReverseScale                           = parameter->data.sequentialPlanner.metric.weightReverseScale;
    plannerParameter.sequentialPlanner.metric.weightReverseDecay                           = parameter->data.sequentialPlanner.metric.weightReverseDecay;
    plannerParameter.sequentialPlanner.model.matF                                          = parameter->data.sequentialPlanner.model.matF;
    plannerParameter.sequentialPlanner.model.matB                                          = parameter->data.sequentialPlanner.model.matB;
    plannerParameter.sequentialPlanner.model.vecTimeconstantsXYN                           = parameter->data.sequentialPlanner.model.vecTimeconstantsXYN;
    plannerParameter.sequentialPlanner.model.vecTimeconstantsInput                         = parameter->data.sequentialPlanner.model.vecTimeconstantsInput;
    plannerParameter.sequentialPlanner.model.lowerLimitXYN                                 = parameter->data.sequentialPlanner.model.lowerLimitXYN;
    plannerParameter.sequentialPlanner.model.upperLimitXYN                                 = parameter->data.sequentialPlanner.model.upperLimitXYN;
    plannerParameter.sequentialPlanner.pathPlanner.periodGoalSampling                      = parameter->data.sequentialPlanner.pathPlanner.periodGoalSampling;
    plannerParameter.sequentialPlanner.motionPlanner.samplingRangePosition                 = parameter->data.sequentialPlanner.motionPlanner.samplingRangePosition;
    plannerParameter.sequentialPlanner.motionPlanner.samplingRangeAngle                    = parameter->data.sequentialPlanner.motionPlanner.samplingRangeAngle;
    plannerParameter.sequentialPlanner.motionPlanner.periodGoalSampling                    = parameter->data.sequentialPlanner.motionPlanner.periodGoalSampling;
    plannerParameter.sequentialPlanner.motionPlanner.sampletime                            = parameter->data.sequentialPlanner.motionPlanner.sampletime;
    plannerParameter.sequentialPlanner.motionPlanner.maxInputPathLength                    = parameter->data.sequentialPlanner.motionPlanner.maxInputPathLength;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusX                 = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusX;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusY                 = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusY;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusPsi               = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusPsi;
    plannerParameter.sequentialPlanner.motionPlanner.controller.minRadiusPosition          = parameter->data.sequentialPlanner.motionPlanner.controller.minRadiusPosition;
    plannerParameter.sequentialPlanner.motionPlanner.controller.matK                       = parameter->data.sequentialPlanner.motionPlanner.controller.matK;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangePose          = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangePose;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR           = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN           = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN;
    plannerParameter.onlinePlanner.predictInitialStateOnReset                              = parameter->data.onlinePlanner.predictInitialStateOnReset;
    plannerParameter.onlinePlanner.maxComputationTimePathOnReset                           = parameter->data.onlinePlanner.maxComputationTimePathOnReset;
    plannerParameter.onlinePlanner.maxComputationTimeMotionOnReset                         = parameter->data.onlinePlanner.maxComputationTimeMotionOnReset;
    plannerParameter.onlinePlanner.maxComputationTimePath                                  = parameter->data.onlinePlanner.maxComputationTimePath;
    plannerParameter.onlinePlanner.maxComputationTimeMotion                                = parameter->data.onlinePlanner.maxComputationTimeMotion;
    plannerParameter.onlinePlanner.additionalAheadPlanningTime                             = parameter->data.onlinePlanner.additionalAheadPlanningTime;
    plannerParameter.onlinePlanner.additionalTrajectoryDuration                            = parameter->data.onlinePlanner.additionalTrajectoryDuration;
    plannerParameter.onlinePlanner.timeKeepPastTrajectory                                  = parameter->data.onlinePlanner.timeKeepPastTrajectory;

    // if there're errors make whole parameter invalid (remove vehicle shape and skeletal points)
    if(!validVehicleShape || !validSkeletalPoints){
        plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
        plannerParameter.sequentialPlanner.geometry.skeletalPoints.clear();
    }
}


/**
 * @brief Serialize output of the asynchronous online planner.
 * @param[out] output Serialized output data.
 * @param[in] plannerOutput Output structure of the asynchronous online planner to be serialized.
 * @details The trajectory, path and referencePath containers may be shrinked to fit the constant-size serialization data.
 */
inline void Serialize(mpsv::planner::SerializationAsyncOnlinePlannerOutputUnion* output, const mpsv::planner::AsyncOnlinePlannerOutput& plannerOutput){
    output->data.timestamp                                 = plannerOutput.timestamp;
    output->data.timestampInput                            = plannerOutput.timestampInput;
    output->data.timestampParameter                        = plannerOutput.timestampParameter;
    output->data.threadState                               = static_cast<uint8_t>(plannerOutput.threadState);
    output->data.timeoutInput                              = static_cast<uint8_t>(plannerOutput.timeoutInput);
    output->data.validInput                                = static_cast<uint8_t>(plannerOutput.validInput);
    output->data.validParameter                            = static_cast<uint8_t>(plannerOutput.validParameter);
    output->data.performedReset                            = static_cast<uint8_t>(plannerOutput.performedReset);
    output->data.error                                     = static_cast<uint8_t>(plannerOutput.error);
    output->data.originLLA                                 = plannerOutput.originLLA;

    // trajectory
    output->data.numTrajectoryPoints                       = static_cast<uint16_t>(std::min(plannerOutput.trajectory.size(), output->data.trajectory.size()));
    output->data.trajectoryShrinked                        = static_cast<uint8_t>(plannerOutput.trajectory.size() > output->data.trajectory.size());
    output->data.sampletime                                = plannerOutput.sampletime;
    for(uint16_t k = 0; k < output->data.numTrajectoryPoints; ++k){
        output->data.trajectory[k] = plannerOutput.trajectory[k];
    }

    // pathPlanner.path
    output->data.pathPlanner.numPoses                      = static_cast<uint16_t>(std::min(plannerOutput.pathPlanner.path.size(), output->data.pathPlanner.path.size()));
    output->data.pathPlanner.pathShrinked                  = static_cast<uint8_t>(plannerOutput.pathPlanner.path.size() > output->data.pathPlanner.path.size());
    for(uint16_t k = 0; k < output->data.pathPlanner.numPoses; ++k){
        output->data.pathPlanner.path[k] = plannerOutput.pathPlanner.path[k];
    }
    output->data.pathPlanner.goalReached                   = static_cast<uint8_t>(plannerOutput.pathPlanner.goalReached);
    output->data.pathPlanner.isFeasible                    = static_cast<uint8_t>(plannerOutput.pathPlanner.isFeasible);
    output->data.pathPlanner.outOfNodes                    = static_cast<uint8_t>(plannerOutput.pathPlanner.outOfNodes);
    output->data.pathPlanner.numberOfPerformedIterations   = plannerOutput.pathPlanner.numberOfPerformedIterations;
    output->data.pathPlanner.timestampOfComputationUTC     = plannerOutput.pathPlanner.timestampOfComputationUTC;
    output->data.pathPlanner.cost                          = plannerOutput.pathPlanner.cost;

    // motionPlanner.referencePath
    output->data.motionPlanner.numPoses                    = static_cast<uint16_t>(std::min(plannerOutput.motionPlanner.referencePath.size(), output->data.motionPlanner.referencePath.size()));
    output->data.motionPlanner.referencePathShrinked       = static_cast<uint8_t>(plannerOutput.motionPlanner.referencePath.size() > output->data.motionPlanner.referencePath.size());
    for(uint16_t k = 0; k < output->data.motionPlanner.numPoses; ++k){
        output->data.motionPlanner.referencePath[k] = plannerOutput.motionPlanner.referencePath[k];
    }
    output->data.motionPlanner.goalReached                 = static_cast<uint8_t>(plannerOutput.motionPlanner.goalReached);
    output->data.motionPlanner.isFeasible                  = static_cast<uint8_t>(plannerOutput.motionPlanner.isFeasible);
    output->data.motionPlanner.outOfNodes                  = static_cast<uint8_t>(plannerOutput.motionPlanner.outOfNodes);
    output->data.motionPlanner.numberOfPerformedIterations = plannerOutput.motionPlanner.numberOfPerformedIterations;
    output->data.motionPlanner.timestampOfComputationUTC   = plannerOutput.motionPlanner.timestampOfComputationUTC;
    output->data.motionPlanner.cost                        = plannerOutput.motionPlanner.cost;
}


} /* namespace: planner */


} /* namespace: mpsv */

