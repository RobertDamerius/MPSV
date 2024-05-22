#pragma once


#include <string>
#include <vector>
#include <mpsv/mpsv.hpp>


/* Version Settings */
extern const std::string strOS;
extern const std::string strVersion;
extern const std::string strCompilerVersion;
extern const std::string strBuilt;


/**
 * @brief This class is used for benchmark tests.
 */
class Benchmark {
    public:
        /**
         * @brief Run the main entry function for benchmark tests.
         */
        static void Run(void);

    protected:
        static std::vector<mpsv::geometry::ConvexPolygon> convexPolygons;   // List of randomly generated polygons for benchmark tests.
        static mpsv::control::VehicleSimulator vehicleSimulator;            // Vehicle simulator to be used for benchmark tests.
        static double preventOptimization;                                  // Temporary value used to prevent the compiler from optimizing out certain code sections.

        /**
         * @brief Generate a specific number of polygons.
         * @param[in] numPolygons Number of polygons to be generated.
         * @details Random triangles are generated.
         */
        static void GeneratePolygons(const size_t numPolygons);

        /**
         * @brief Initialize the vehicle simulator by setting model and controller parameters.
         */
        static void InitializeVehicleSimulator(void);

        /**
         * @brief Idle some time between benchmark tests.
         */
        static void Idle(void);

        /**
         * @brief Run the polygon overlap benchmark test.
         * @return Computation time of the benchmark test.
         */
        static double PolygonOverlap(void);

        /**
         * @brief Run the cost map benchmark in single threading mode.
         * @return Computation time of the benchmark test.
         */
        static double CostMapSingleThread(void);

        /**
         * @brief Run the cost map benchmark in multi threading mode.
         * @return Computation time of the benchmark test.
         */
        static double CostMapMultiThread(void);

        /**
         * @brief Cost function to be used for generating data of the cost map benchmarks.
         * @param[in] x X position.
         * @param[in] y Y position.
         * @return Resulting cost value at {x,y}, that is, a*exp(-b*d^2(x,y)), where d^(x,y) is the closest squared distance to the edges of all convex polygons.
         */
        static double CostFunction(double x, double y);

        /**
         * @brief Run the forward simulation benchmark.
         * @return Computation time of the benchmark test.
         */
        static double ForwardSimulation(void);

        /**
         * @brief Run the path planning benchmark.
         * @return Computation time of the benchmark test.
         */
        static double PathPlanning(void);

        /**
         * @brief Run the motion planning benchmark.
         * @return Computation time of the benchmark test.
         */
        static double MotionPlanning(void);

        /**
         * @brief Print header of benchmark result table.
         */
        static void PrintResultHeader(void);

        /**
         * @brief Print result of a benchmark test.
         * @param[in] name Name of the benchmark test.
         * @param[in] times List of computation times of the same benchmark.
         */
        static void PrintResultData(std::string name, std::vector<double>& times);

        /**
         * @brief Print footer of benchmark result table.
         */
        static void PrintResultFooter(void);

        /**
         * @brief Print system information to the output.
         */
        static void PrintSystemInfo(void);

        /**
         * @brief Generate a list of static obstacles for path and motion planning benchmarks.
         * @param[out] staticObstacles List of static obstacles.
         */
        static void GenerateObstaclesForPlanning(std::vector<mpsv::geometry::StaticObstacle>& staticObstacles);
};

