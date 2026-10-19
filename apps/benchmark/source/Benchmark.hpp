#pragma once


#include <string>
#include <vector>
#include <mpsv/mpsv.hpp>


/* Version Settings */
#if __linux__
inline constexpr std::string_view strOS{"Linux"};
#elif __FreeBSD__
inline constexpr std::string_view strOS{"FreeBSD"};
#elif __ANDROID__
inline constexpr std::string_view strOS{"Android"};
#elif __APPLE__
inline constexpr std::string_view strOS{"macOS"};
#elif _WIN32
inline constexpr std::string_view strOS{"Windows"};
#else
inline constexpr std::string_view strOS{"unknown"};
#endif
inline constexpr std::string_view strVersion{"1.0.0"};
inline constexpr std::string_view strCompilerVersion{__VERSION__};
inline constexpr std::string_view strBuilt{__DATE__ " " __TIME__};


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
         * @return Number of iterations of the benchmark test.
         */
        static uint32_t PathPlanning(void);

        /**
         * @brief Run the motion planning benchmark.
         * @return Number of iterations of the benchmark test.
         */
        static uint32_t MotionPlanning(void);

        /**
         * @brief Print header of benchmark result table.
         */
        static void PrintResultHeaderTimes(void);
        static void PrintResultHeaderIterations(void);

        /**
         * @brief Print result of a benchmark test.
         * @param[in] name Name of the benchmark test.
         * @param[in] times List of computation times of the same benchmark.
         */
        static void PrintResultDataTimes(std::string name, std::vector<double>& times);

        /**
         * @brief Print result of a benchmark test.
         * @param[in] name Name of the benchmark test.
         * @param[in] iterations Number of iterations of the same benchmark.
         */
        static void PrintResultDataIterations(std::string name, std::vector<uint32_t>& iterations);

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
