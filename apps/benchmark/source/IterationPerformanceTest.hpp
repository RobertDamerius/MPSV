#pragma once


#include <string>
#include <vector>
#include <mpsv/mpsv.hpp>
#include <SceneDataFile.hpp>


class IterationPerformanceTest {
    public:
        /**
         * @brief Run the iteration performance test.
         */
        static void Run(void);

    private:
        static SceneDataFile sceneData;   // Scene data file containing scenarios of realistic planning problems.

        /**
         * @brief Load scene data file.
         * @return True if success, false otherwise.
         */
        static bool LoadSceneData(void);

        /**
         * @brief Get the sequential planner parameter set for testing.
         */
        static mpsv::planner::SequentialPlannerParameterSet GetParameterSet(void);

        /**
         * @brief Get the sequential planner input data based on the current scene data.
         * @param[in] scene The scene for which to get the sequential planner input data.
         * @return Sequential planner input data.
         */
        static mpsv::planner::SequentialPlannerInput GetInput(Scene& scene);

        /**
         * @brief Solve the sequential planning problem for given scene data.
         * @param[in] scene The scene data for which to solve the problem.
         * @param[in] recreateFile True if the output data log file should be recreated.
         */
        static void Solve(Scene& scene, bool recreateFile);
};

