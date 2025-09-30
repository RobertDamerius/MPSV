#pragma once


#include <cstdint>
#include <array>
#include <vector>
#include <mpsv_mpsv.hpp>


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
         * @param[in] cpuCoreIDs (linux-only) List of CPU cores on which the worker thread is eligible to run. If no or invalid IDs are specified, no thread affinity is set.
         */
        void Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs);

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
        void Step(mpsv::planner::serialization_output* output, mpsv::planner::serialization_input* input, mpsv::planner::serialization_parameter* parameter);

    protected:
        mpsv::planner::AsyncOnlinePlanner planner;                        // The internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerInput plannerInput;              // Stores the input data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerParameterSet plannerParameter;   // Stores the parameter data for the internal asynchronous online planner.
        mpsv::planner::AsyncOnlinePlannerOutput plannerOutput;            // Stores the output of the asynchronous online planner.
};

