#pragma once


#include <mpsv/mpsv.hpp>
#include <PlannerConfiguration.hpp>
#include <PeriodicTimer.hpp>


/**
 * @brief Represents the planner that solves sequential planning problems in a separate thread.
 */
class Planner: protected mpsv::planner::AsyncOnlinePlanner {
    public:
        /* make some member function public again */
        using mpsv::planner::AsyncOnlinePlanner::SetParameter;
        using mpsv::planner::AsyncOnlinePlanner::SetInput;

        /**
         * @brief Start the planner.
         * @return True if success, false otherwise.
         */
        bool Start(const PlannerConfiguration plannerConf);

        /**
         * @brief Stop the planner.
         */
        void Stop(void);

    protected:
        /**
         * @brief This function is called from inside the worker thread of the planner, if a planning problem has been solved and new
         * output data is available. The actual data is obtained via @ref GetOutput.
         */
        void CallbackNewOutputAvailable(void);

    private:
        std::thread periodicThread;    // Thread that periodically checks the planner output and sends messages via the network manager.
        PeriodicTimer periodicTimer;   // Timer for periodic events.

        /**
         * @brief Thread function of the periodic update thread.
         */
        void PeriodicUpdateThread(void);
};

