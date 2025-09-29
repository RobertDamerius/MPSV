#pragma once


#include <Common.hpp>


/**
 * @brief Represents the planner configuration.
 */
class PlannerConfiguration {
    public:
        int16_t pathMaxNumNodes;              // Maximum number of nodes to be used in the tree (path planner).
        uint32_t pathMaxNumSamples;           // Maximum number of random samples to be generated in advance (path planner).
        int16_t motionMaxNumNodes;            // Maximum number of nodes to be used in the tree (motion planner).
        uint32_t motionMaxNumSamples;         // Maximum number of random samples to be generated in advance (motion planner).
        int32_t threadPriorityPlanner;        // The priority to be set for the planner thread (1: low, 99: high).
        int32_t threadPriorityUpdater;        // The priority to be set for the output update thread (1: low, 99: high).
        int32_t ompNumThreads;                // The number of threads to be used for parallel computing (OMP_NUM_THREADS). This value is only set if it's greater than zero! This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
        int32_t ompDynamic;                   // Greater than zero if dynamic adjustment of the number of threads should be enabled for parallel computing or zero to disable dynamic adjustment (OMP_DYNAMIC). If this value is set less than 0, this parameter is ignored. This parameter has no effect if the software compiled with MPSV_DONT_USE_OMP set.
        std::vector<int32_t> cpuCoreIDs;      // (linux-only) List of CPU core IDs on which the worker thread is eligible to run. If no or invalid IDs are specified, no thread affinity is set.
        double updatePeriod;                  // Update period in seconds for the periodic output updater thread.

        /**
         * @brief Construct a new planner configuration object and set default values.
         */
        PlannerConfiguration(){ Clear(); }

        /**
         * @brief Clear configuration and set default values.
         */
        void Clear(void){
            pathMaxNumNodes = 0;
            pathMaxNumSamples = 0;
            motionMaxNumNodes = 0;
            motionMaxNumSamples = 0;
            threadPriorityPlanner = 0;
            threadPriorityUpdater = 0;
            ompNumThreads = 0;
            ompDynamic = 0;
            cpuCoreIDs.clear();
            updatePeriod = 0.0;
        }
};

