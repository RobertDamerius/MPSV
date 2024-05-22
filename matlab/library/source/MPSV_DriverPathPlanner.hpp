#pragma once


#include <cstdint>


/**
 * @brief Initialize the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 * @param[in] maxNumNodes Maximum number of nodes to be used in the tree. A fixed-node RRT* planner is used. If the tree contains the maximum number of nodes, random leaf nodes are removed. If all nodes belong to the solution and can't be removed, then the outOfNodes error is set in the output.
 * @param[in] maxNumSamples Maximum number of random samples to be generated in advance. A circular buffer of random samples is generated for fast sampling. The buffer can be quite large to ensure, that no duplicated samples are drawn within one solve operation.
 */
extern void MPSV_PathPlannerInitialize(void** workVector, int16_t maxNumNodes, uint32_t maxNumSamples);

/**
 * @brief Terminate the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 */
extern void MPSV_PathPlannerTerminate(void* workVector);

/**
 * @brief Perform one step of the driver.
 * @param[in] workVector The simulink work vector storing the pointer to the actual path planner object.
 * @param[out] serializedDataOut Serialized data (bytes) representing the output data.
 * @param[in] serializedDataIn Serialized data (bytes) representing the input data.
 */
extern void MPSV_PathPlannerStep(void* workVector, uint8_t* serializedDataOut, uint8_t* serializedDataIn);

