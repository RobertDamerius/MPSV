#pragma once


#include <cstdint>
#include <array>
#include <vector>
#include <mpsv_mpsv.hpp>


/**
 * @brief Constant-size input data for the asynchronous online planner wrapper.
 */
struct __attribute__((packed)) serialization_asynchronous_online_planner_input {
    double timestamp;
    uint8_t enable:1;
    uint8_t reset:1;
    uint8_t reserved:6;
    std::array<double,3> originLLA;
    std::array<double,12> initialStateAndInput;
    std::array<double,3> finalPose;
    std::array<double,3> samplingBoxCenterPose;
    std::array<double,2> samplingBoxDimension;
    std::array<std::array<float,2>,8000> verticesStaticObstacles;
};


/**
 * @brief Constant-size parameter data for the asynchronous online planner wrapper.
 */
struct __attribute__((packed)) serialization_asynchronous_online_planner_parameter {
    double timestamp;
    double timeoutInput;
    struct __attribute__((packed)) {
        struct __attribute__((packed)) {
            double collisionCheckMaxPositionDeviation;
            double collisionCheckMaxAngleDeviation;
            std::array<std::array<float,2>,100> verticesVehicleShape;
            std::array<std::array<float,2>,10> skeletalPoints;
        } geometry;
        struct __attribute__((packed)) {
            int32_t modBreakpoints;
            double resolution;
            double distanceScale;
            double distanceDecay;
        } costMap;
        struct __attribute__((packed)) {
            double weightPsi;
            double weightSway;
            double weightReverseScale;
            double weightReverseDecay;
        } metric;
        struct __attribute__((packed)) {
            std::array<double,36> matF;
            std::array<double,9> matB;
            std::array<double,3> vecTimeconstantsXYN;
            std::array<double,3> vecTimeconstantsInput;
            std::array<double,3> lowerLimitXYN;
            std::array<double,3> upperLimitXYN;
        } model;
        struct __attribute__((packed)) {
            uint32_t periodGoalSampling;
        } pathPlanner;
        struct __attribute__((packed)) {
            double samplingRangePosition;
            double samplingRangeAngle;
            uint32_t periodGoalSampling;
            double sampletime;
            double maxInputPathLength;
            struct __attribute__((packed)) {
                double maxRadiusX;
                double maxRadiusY;
                double maxRadiusPsi;
                double minRadiusPosition;
                std::array<double,36> matK;
            } controller;
            struct __attribute__((packed)) {
                std::array<double,3> rangePose;
                std::array<double,3> rangeUVR;
                std::array<double,3> rangeXYN;
            } regionOfAttraction;
        } motionPlanner;
    } sequentialPlanner;
    struct __attribute__((packed)) {
        uint8_t reserved;
        double maxComputationTimePathOnReset;
        double maxComputationTimeMotionOnReset;
        double maxComputationTimePath;
        double maxComputationTimeMotion;
        double additionalAheadPlanningTime;
        double additionalTrajectoryDuration;
        double timeKeepPastTrajectory;
    } onlinePlanner;
};


/**
 * @brief Constant-size output data for the asynchronous online planner wrapper.
 */
struct __attribute__((packed)) serialization_asynchronous_online_planner_output {
    double timestamp;
    double timestampInput;
    double timestampParameter;
    uint8_t inputError;
    uint8_t parameterError;
    uint8_t threadState:2;
    uint8_t performedReset:1;
    uint8_t error:1;
    uint8_t trajectoryShrinked:1;
    uint8_t reserved:3;
    std::array<double,3> originLLA;
    uint16_t numTrajectoryPoints;
    std::array<std::array<double,12>,500> trajectory;
    double sampletime;
    struct __attribute__((packed)) {
        uint16_t numPoses;
        std::array<std::array<double,3>,100> path;
        uint8_t goalReached:1;
        uint8_t isFeasible:1;
        uint8_t outOfNodes:1;
        uint8_t pathShrinked:1;
        uint8_t reserved:4;
        uint32_t numberOfPerformedIterations;
        double timestampOfComputationUTC;
        double cost;
    } pathPlanner;
    struct __attribute__((packed)) {
        uint16_t numPoses;
        std::array<std::array<double,3>,100> referencePath;
        uint8_t goalReached:1;
        uint8_t isFeasible:1;
        uint8_t outOfNodes:1;
        uint8_t referencePathShrinked:1;
        uint8_t reserved:4;
        uint32_t numberOfPerformedIterations;
        double timestampOfComputationUTC;
        double cost;
    } motionPlanner;
};


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
         * @param[in] maxNumCells Maximum number of cells in the cost maps.
         * @param[in] threadPriority The priority to be set for the thread from 1 (low) to 99 (high).
         * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero!
         * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored.
         * @param[in] cpuCoreIDs (linux-only) List of CPU cores on which the worker thread is eligible to run. Only IDs in range and available to the worker are used.
         */
        void Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs);

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
        void Step(serialization_asynchronous_online_planner_output* output, serialization_asynchronous_online_planner_input* input, serialization_asynchronous_online_planner_parameter* parameter);

    protected:
        mpsv::planner::AsyncOnlinePlanner m_planner;                        // The internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerInput m_plannerInput;              // Stores the input data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerParameterSet m_plannerParameter;   // Stores the parameter data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerOutput m_plannerOutput;            // Stores the output of the asynchronous online planner.

        /**
         * @brief Deserialize parameter data for the internal asynchronous online planner.
         * @param[in] parameter Serialized parameter data.
         */
        void DeserializeParameter(serialization_asynchronous_online_planner_parameter* parameter);

        /**
         * @brief Deserialize input data for the internal asynchronous online planner.
         * @param[in] input Serialized input data.
         */
        void DeserializeInput(serialization_asynchronous_online_planner_input* input);

        /**
         * @brief Serialize the current output of the internal asynchronous online planner.
         * @param[out] output Serialized output data.
         */
        void SerializeOutput(serialization_asynchronous_online_planner_output* output);
};
