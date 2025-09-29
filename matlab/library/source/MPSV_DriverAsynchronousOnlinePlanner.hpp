#pragma once


#include <cstdint>


/**
 * @brief Initialize the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 * @param[in] pathMaxNumNodes Maximum number of nodes to be used in the tree (path planning). A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
 * @param[in] pathMaxNumSamples Maximum number of random samples to be generated in advance (path planning). A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
 * @param[in] motionMaxNumNodes Maximum number of nodes to be used in the tree (motion planning). A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
 * @param[in] motionMaxNumSamples Maximum number of random samples to be generated in advance (motion planning). A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
 * @param[in] threadPriority The priority to be set for the thread from 1 (low) to 99 (high).
 * @param[in] ompNumThreads The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
 * @param[in] ompDynamic Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
 * @param[in] cpuCoreIDs (linux-only) List of CPU cores on which the worker thread is eligible to run. If no or invalid IDs are specified, no thread affinity is set.
 * @param[in] numCPUCoreIDs The actual number of CPU core IDs in the cpuCoreIDs list.
 */
extern void MPSV_AsynchronousOnlinePlannerInitialize(void** workVector, int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, int32_t* cpuCoreIDs, uint32_t numCPUCoreIDs);

/**
 * @brief Terminate the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 */
extern void MPSV_AsynchronousOnlinePlannerTerminate(void* workVector);

/**
 * @brief Perform one step of the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 * @param[out] serializedOutput Serialized data (bytes) representing the output data.
 * @param[in] serializedInput Serialized data (bytes) representing the input data.
 * @param[in] serializedParameter Serialized data (bytes) representing the parameter data.
 */
extern void MPSV_AsynchronousOnlinePlannerStep(void* workVector, uint8_t* serializedOutput, uint8_t* serializedInput, uint8_t* serializedParameter);

