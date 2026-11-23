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
    std::uint8_t enable:1;
    std::uint8_t reset:1;
    std::uint8_t reserved:6;
    std::array<double, 3U> originLLA;
    std::array<double, 12U> initialStateAndInput;
    std::array<double, 3U> finalPose;
    std::array<double, 3U> samplingBoxCenterPose;
    std::array<double, 2U> samplingBoxDimension;
    std::array<std::array<float, 2U>, 8000U> verticesStaticObstacles;
};


/**
 * @brief Constant-size parameter data for the asynchronous online planner wrapper.
 */
struct __attribute__((packed)) serialization_asynchronous_online_planner_parameter {
    struct __attribute__((packed)) {
        struct __attribute__((packed)) {
            double collisionCheckMaxPositionDeviation;
            double collisionCheckMaxAngleDeviation;
            std::array<std::array<float, 2U>, 100U> verticesVehicleShape;
            std::array<std::array<float, 2U>, 10U> skeletalPoints;
        } geometry;
        struct __attribute__((packed)) {
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
            std::array<double, 36U> matF;
            std::array<double, 9U> matB;
            std::array<double, 3U> vecTimeconstantsXYN;
            std::array<double, 3U> vecTimeconstantsInput;
            std::array<double, 3U> lowerLimitXYN;
            std::array<double, 3U> upperLimitXYN;
        } model;
        struct __attribute__((packed)) {
            std::uint32_t periodGoalSampling;
        } pathPlanner;
        struct __attribute__((packed)) {
            double samplingRangePosition;
            double samplingRangeAngle;
            std::uint32_t periodGoalSampling;
            double sampletime;
            double maxInputPathLength;
            struct __attribute__((packed)) {
                double maxRadiusX;
                double maxRadiusY;
                double maxRadiusPsi;
                double minRadiusPosition;
                std::array<double, 36U> matK;
            } controller;
            struct __attribute__((packed)) {
                std::array<double, 3U> rangePose;
                std::array<double, 3U> rangeUVR;
                std::array<double, 3U> rangeXYN;
            } regionOfAttraction;
        } motionPlanner;
    } sequentialPlanner;
    struct __attribute__((packed)) {
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
    std::uint8_t errorCode;
    std::uint8_t threadState:2;
    std::uint8_t performedReset:1;
    std::uint8_t trajectoryTruncated:1;
    std::uint8_t reserved:4;
    std::array<double, 3U> originLLA;
    std::int32_t numTrajectoryPoints;
    std::array<std::array<double, 12U>, 500U> trajectory;
    double sampletime;
    struct __attribute__((packed)) {
        std::int32_t numPoses;
        std::array<std::array<double, 3U>, 100U> path;
        std::uint8_t goalReached:1;
        std::uint8_t pathTruncated:1;
        std::uint8_t reserved:6;
        std::uint32_t numberOfPerformedIterations;
        double timestampOfComputationUTC;
        double cost;
    } pathPlanner;
    struct __attribute__((packed)) {
        std::int32_t numPoses;
        std::array<std::array<double, 3U>, 100U> referencePath;
        std::uint8_t goalReached:1;
        std::uint8_t referencePathTruncated:1;
        std::uint8_t reserved:6;
        std::uint32_t numberOfPerformedIterations;
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
         * @details Calls the @ref Stop member function.
         */
        ~MPSV_WrapperAsynchronousOnlinePlanner();

        /**
         * @brief Start the wrapper. The memory for the internal planner is allocated.
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
        void Start(std::int16_t pathMaxNumNodes, std::uint32_t pathMaxNumSamples, std::int16_t motionMaxNumNodes, std::uint32_t motionMaxNumSamples, std::uint32_t maxNumCells, std::int32_t threadPriority, std::int32_t ompNumThreads, std::int32_t ompDynamic, std::vector<std::int32_t> cpuCoreIDs);

        /**
         * @brief Stop the wrapper. The memory for the internal planner is freed.
         */
        void Stop();

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
