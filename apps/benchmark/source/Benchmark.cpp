#include <Benchmark.hpp>
#include <omp.h>
#include <iostream>
#include <chrono>
/* OS depending */
// Windows System
#ifdef _WIN32
#include <windows.h>
#include <Iphlpapi.h>
// Unix System
#elif __linux__
#include <sys/utsname.h>
#else
// Other
#endif
using namespace mpsv::core;
using namespace mpsv::control;
using namespace mpsv::geometry;
using namespace mpsv::sampler;


#if __linux__
const std::string strOS("Linux");
#elif __FreeBSD__
const std::string strOS("FreeBSD");
#elif __ANDROID__
const std::string strOS("Android");
#elif __APPLE__
const std::string strOS("macOS");
#elif _WIN32
const std::string strOS("Windows");
#else
const std::string strOS("unknown");
#endif
const std::string strVersion("20231027");
const std::string strCompilerVersion(__VERSION__);
const std::string strBuilt(__DATE__ " " __TIME__);


std::vector<mpsv::geometry::ConvexPolygon> Benchmark::convexPolygons;
mpsv::control::VehicleSimulator Benchmark::vehicleSimulator;
double Benchmark::preventOptimization = 0.0;


void Benchmark::Run(void){
    // Benchmark print info
    PrintSystemInfo();

    // Generate a test set of polygons
    printf("[BENCHMARK] Initializing benchmark data\n");
    GeneratePolygons(10000);
    InitializeVehicleSimulator();

    // Benchmark: Polygon Overlap
    Idle();
    printf("[BENCHMARK] (1/6) Polygon Overlap ...\n");
    std::vector<double> t1;
    for(int i = 0; i < 10; ++i) t1.push_back(PolygonOverlap());

    // Benchmark: Cost Map (Single Thread)
    Idle();
    printf("[BENCHMARK] (2/6) Cost Map (Single Thread) ...\n");
    std::vector<double> t2;
    for(int i = 0; i < 10; ++i) t2.push_back(CostMapSingleThread());

    // Benchmark: Cost Map (Multi Thread)
    Idle();
    printf("[BENCHMARK] (3/6) Cost Map (Multi Thread) ...\n");
    std::vector<double> t3;
    for(int i = 0; i < 10; ++i) t3.push_back(CostMapMultiThread());

    // Benchmark: Forward Simulation
    Idle();
    printf("[BENCHMARK] (4/6) Forward Simulation ...\n");
    std::vector<double> t4;
    for(int i = 0; i < 10; ++i) t4.push_back(ForwardSimulation());

    // Benchmark: Path Planning
    Idle();
    printf("[BENCHMARK] (5/6) Path Planning ...\n");
    std::vector<double> t5;
    for(int i = 0; i < 10; ++i) t5.push_back(PathPlanning());

    // Benchmark: Motion Planning
    Idle();
    printf("[BENCHMARK] (6/6) Motion Planning ...\n");
    std::vector<double> t6;
    for(int i = 0; i < 10; ++i) t6.push_back(MotionPlanning());

    // Print results
    PrintResultHeader();
    PrintResultData("Convex Polygon Overlap", t1);
    PrintResultData("Cost Map (Single Thread)", t2);
    PrintResultData("Cost Map (Multi Thread)", t3);
    PrintResultData("Forward Simulation", t4);
    PrintResultData("Path Planning", t5);
    PrintResultData("Motion Planning", t6);
    PrintResultFooter();
}

void Benchmark::Idle(void){
    printf("[BENCHMARK] Idle for 5 seconds ...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void Benchmark::GeneratePolygons(const size_t numPolygons){
    // Clear polygon container and reserve memory
    convexPolygons.clear();
    convexPolygons.reserve(numPolygons);

    // Initialize a sampler for random numbers in range [{0,0}, {100,100}].
    HaltonSequence sampler;
    if(!sampler.Initialize(2, 3 * numPolygons, 100.0)){
        fprintf(stderr,"ERROR: Failed to initialize the halton sequence!\n");
        return;
    }

    // Generate triangles
    for(size_t p = 0, k = 0; p < numPolygons; ++p){
        double x1 = sampler.Sample(0,k);
        double y1 = sampler.Sample(1,k++);
        double x2 = sampler.Sample(0,k);
        double y2 = sampler.Sample(1,k++);
        double x3 = sampler.Sample(0,k);
        double y3 = sampler.Sample(1,k++);
        convexPolygons.push_back(ConvexPolygon({{x1,y1},{x2,y2},{x3,y3}}));
        if(!convexPolygons.back().EnsureCorrectVertexOrder()){
            fprintf(stderr,"ERROR: Failed to build convex polygon for test set!\n");
        }
    }
    sampler.Terminate();
}

void Benchmark::InitializeVehicleSimulator(void){
    std::array<double,36> matF                      = {-0.031604977390302, 0.0, 0.0, 0.233741474945168, 0.0, 0.0, 0.0, 0.0, 0.133462292221887, -0.020792892088185, 0.0, 0.0, 0.0, -0.041708610292712, 0.017242263124379, 0.0, -0.251243228165994, -0.000476044356140, 0.0, 0.0, 0.005297185500344, 0.0, -0.167981638498434, 0.497442136687157, 0.0, 0.000017966020615, -0.023528504195578, 0.0, 0.000108223241795, 0.000649602175186, 0.0, 0.0, -0.000002281767319, 0.0, 0.000072358238719, -0.678801229030825};
    std::array<double,9> matB                       = {0.000237948834266, -0.000004551592718, 0.000010003488944, -0.000009313932115, 0.000215194147058, -0.000024957572224, -0.000002202124158, -0.000002930260852, 0.000043018345190};
    std::array<double,3> vecTimeconstantsXYN        = {0.2, 0.2, 0.2};
    std::array<double,3> vecTimeconstantsInput      = {0.5, 0.5, 0.5};
    std::array<double,3> satXYN                     = {800.0, 600.0, 1200.0};
    std::array<double,9> vecTimeconstantsFlatStates = {12.0, 15.0, 15.0, 1.0, 1.0, 1.0, 2.5, 2.5, 2.5};
    std::array<double,3> satUVR                     = {1.8, 0.8, 0.4};
    std::array<double,36> matK                      = {1.80025300316239, 0.0, 0.0, 14.9932006585738, 0.0, 0.0, 34.8623145223247, 0.0, 0.0, 21.6450501549992, 0.0, 0.0, 0.0, 2.25031625395272, 0.0, 0.0, 18.9915008232153, 0.0, 0.0, 44.4528931529019, 0.0, 0.0, 27.6813126937466, 0.0, 0.0, 0.0, 2.25031625395276, 0.0, 0.0, 18.9915008232156, 0.0, 0.0, 44.4528931529024, 0.0, 0.0, 27.681312693747};
    double maxRadiusX                               = 10.0;
    double maxRadiusY                               = 6.0;
    double maxRadiusPsi                             = 1.0;
    double minRadiusPosition                        = 2.0;
    if(!vehicleSimulator.SetModel(matF, matB, vecTimeconstantsXYN, vecTimeconstantsInput, satXYN)){
        fprintf(stderr,"ERROR: Failed to set model parameters!\n");
    }
    if(!vehicleSimulator.SetController(vecTimeconstantsFlatStates, satUVR, matK, maxRadiusX, maxRadiusY, maxRadiusPsi, minRadiusPosition)){
        fprintf(stderr,"ERROR: Failed to set controller parameters!\n");
    }
}

double Benchmark::PolygonOverlap(void){
    // Count the number of overlaps between all convex polygons
    mpsv::core::PerformanceCounter timer;
    size_t numOverlaps = 0;
    timer.Start();
    for(size_t i = 0; i < convexPolygons.size(); ++i){
        for(size_t j = i + 1; j < convexPolygons.size(); ++j){
            if(convexPolygons[i].Overlap(convexPolygons[j])){
                numOverlaps++;
            }
        }
    }
    double t = timer.TimeToStart();
    preventOptimization += static_cast<double>(numOverlaps);
    return t;
}

double Benchmark::CostMapSingleThread(void){
    // Create a look-up table (LUT)
    constexpr int32_t N = 100;
    std::vector<double> lut;
    lut.resize(N*N);

    // Calculate cost function for all cells of the LUT
    mpsv::core::PerformanceCounter timer;
    double res = 100.0 / static_cast<double>(N);
    timer.Start();
    for(int32_t ix = 0, i0 = 0; ix < N; ++ix, i0 += N){
        double x = static_cast<double>(ix) * res;
        for(int32_t iy = 0; iy < N; ++iy){
            double y = static_cast<double>(iy) * res;
            lut[i0 + iy] = CostFunction(x, y);
        }
    }
    double t = timer.TimeToStart();
    preventOptimization += lut[0];
    return t;
}

double Benchmark::CostMapMultiThread(void){
    // Create a look-up table (LUT)
    constexpr int32_t N = 100;
    std::vector<double> lut;
    lut.resize(N*N);

    // Calculate cost function for all cells of the LUT
    mpsv::core::PerformanceCounter timer;
    double res = 100.0 / static_cast<double>(N);
    timer.Start();
    int32_t numTotal = N * N;
    #pragma omp parallel for shared(lut)
    for(int32_t i = 0; i < numTotal; ++i){
        int32_t ix = i / N;
        int32_t iy = i % N;
        double x = static_cast<double>(ix) * res;
        double y = static_cast<double>(iy) * res;
        lut[ix*N + iy] = CostFunction(x, y);
    }
    double t = timer.TimeToStart();
    preventOptimization += lut[0];
    return t;
}

double Benchmark::CostFunction(double x, double y){
    double squaredDistance = std::numeric_limits<double>::infinity();
    for(auto&& polygon : convexPolygons){
        squaredDistance = std::min(squaredDistance, polygon.MinimumSquaredDistanceToEdges(x,y));
    }
    if(std::isfinite(squaredDistance)){
        return 0.1 * std::exp(-0.1 * squaredDistance);
    }
    return 0.0;
}

double Benchmark::ForwardSimulation(void){
    // Define some variables
    std::array<double,12> initialStateAndInput = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double,9> result;
    double sampletime = 0.01;
    uint32_t numSimulationSteps = 10000;
    constexpr int32_t numTrials = 1000;

    mpsv::core::PerformanceCounter timer;
    timer.Start();
    for(int32_t i = 0; i < numTrials; ++i){
        double angle = 2.0 * 3.14159265358979323 * static_cast<double>(i) / static_cast<double>(numTrials);
        initialStateAndInput[0] = 100.0 * std::cos(angle);
        initialStateAndInput[1] = 100.0 * std::sin(angle);
        initialStateAndInput[2] = angle;
        initialStateAndInput[3] = 1.5 * std::cos(3.0 * angle + 1.0);
        initialStateAndInput[4] = 0.5 * std::cos(5.0 * angle + 2.0);
        initialStateAndInput[5] = 0.02 * std::cos(7.0 * angle + 3.0);
        result = vehicleSimulator.PredictMotion(initialStateAndInput, sampletime, numSimulationSteps);
    }
    double t = timer.TimeToStart();
    preventOptimization += result[0];
    return t;
}

double Benchmark::PathPlanning(void){
    mpsv::planner::PathPlanner planner;
    mpsv::planner::PathPlannerParameterSet parameterSet;
    mpsv::planner::PathPlannerInput dataIn;
    mpsv::planner::PathPlannerOutput dataOut;
    mpsv::core::PerformanceCounter executionTimer;

    // Set parameters for this benchmark
    int16_t maxNumNodes = 1000;
    uint32_t maxNumSamples = 1000000;
    uint32_t maxIterations = 50000;
    parameterSet.costMap.modBreakpoints                                = 10;
    parameterSet.costMap.resolution                                    = 0.05;
    parameterSet.costMap.distanceScale                                 = 5.0;
    parameterSet.costMap.distanceDecay                                 = 0.02;
    parameterSet.geometry.collisionCheckMaxPositionDeviation           = 0.100000000000000;
    parameterSet.geometry.collisionCheckMaxAngleDeviation              = mpsv::math::deg2rad(5.0);
    std::vector<std::array<double,2>> vertices                         = {{6,1.5},{-6,1.5},{-6,-1.5},{6,-1.5},{8,0}};
    parameterSet.geometry.vehicleShape.Create(vertices);
    parameterSet.geometry.skeletalPoints.clear();
    parameterSet.geometry.skeletalPoints.push_back({-6.0, 0.0});
    parameterSet.geometry.skeletalPoints.push_back({6.0, 0.0});
    parameterSet.metric.weightPsi                                      = 3.0;
    parameterSet.metric.weightSway                                     = 2.0;
    parameterSet.pathPlanner.periodGoalSampling                        = 50;

    // Generate input data
    dataIn.initialPose           = {-56.8924, -29.7461, 0.4302};
    dataIn.finalPose             = {54.4740, -8.8271, -0.1608};
    dataIn.originOldToNew        = {0.0, 0.0};
    dataIn.samplingBoxCenterPose = {-1.2092, -19.2866, 0.1857};
    dataIn.samplingBoxDimension  = {173.3141, 160.0};
    GenerateObstaclesForPlanning(dataIn.staticObstacles);

    // Initialize planner and apply parameters
    if(!planner.Initialize(maxNumNodes, maxNumSamples)){
        fprintf(stderr,"ERROR: Failed to initialize the path planner!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(!planner.ApplyParameterSet(parameterSet)){
        fprintf(stderr,"ERROR: Failed to set parameters for the path planner!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(!dataIn.IsValid()){
        fprintf(stderr,"ERROR: Input data for the path planner is invalid!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Run actual path planning
    executionTimer.Start();
    planner.Prepare(dataOut, dataIn, true);
    planner.Solve(dataOut, dataIn, maxIterations);
    double t = executionTimer.TimeToStart();

    // Terminate the planner
    planner.Terminate();
    preventOptimization += dataOut.cost;
    return t;
}

double Benchmark::MotionPlanning(void){
    mpsv::planner::MotionPlanner planner;
    mpsv::planner::MotionPlannerParameterSet parameterSet;
    mpsv::planner::MotionPlannerInput dataIn;
    mpsv::planner::MotionPlannerOutput dataOut;
    mpsv::core::PerformanceCounter executionTimer;

    // Set parameters for this benchmark
    int16_t maxNumNodes = 200;
    uint32_t maxNumSamples = 1000000;
    uint32_t maxIterations = 400;

    parameterSet.model.matF                                            = {-0.031604977390302, 0.0, 0.0, 0.233741474945168, 0.0, 0.0, 0.0, 0.0, 0.133462292221887, -0.020792892088185, 0.0, 0.0, 0.0, -0.041708610292712, 0.017242263124379, 0.0, -0.251243228165994, -0.000476044356140, 0.0, 0.0, 0.005297185500344, 0.0, -0.167981638498434, 0.497442136687157, 0.0, 0.000017966020615, -0.023528504195578, 0.0, 0.000108223241795, 0.000649602175186, 0.0, 0.0, -0.000002281767319, 0.0, 0.000072358238719, -0.678801229030825};
    parameterSet.model.matB                                            = {0.000237948834266, -0.000004551592718, 0.000010003488944, -0.000009313932115, 0.000215194147058, -0.000024957572224, -0.000002202124158, -0.000002930260852, 0.000043018345190};
    parameterSet.model.vecTimeconstantsXYN                             = {0.2, 0.2, 0.2};
    parameterSet.model.vecTimeconstantsInput                           = {0.5, 0.5, 0.5};
    parameterSet.model.satXYN                                          = {800.0, 600.0, 1200.0};
    parameterSet.costMap.modBreakpoints                                = 10;
    parameterSet.costMap.resolution                                    = 0.05;
    parameterSet.costMap.distanceScale                                 = 5.0;
    parameterSet.costMap.distanceDecay                                 = 0.02;
    parameterSet.geometry.collisionCheckMaxPositionDeviation           = 0.100000000000000;
    parameterSet.geometry.collisionCheckMaxAngleDeviation              = mpsv::math::deg2rad(5.0);
    std::vector<std::array<double,2>> vertices                         = {{6,1.5},{-6,1.5},{-6,-1.5},{6,-1.5},{8,0}};
    parameterSet.geometry.vehicleShape.Create(vertices);
    parameterSet.geometry.skeletalPoints.clear();
    parameterSet.geometry.skeletalPoints.push_back({-3.0, 0.0});
    parameterSet.geometry.skeletalPoints.push_back({3.0, 0.0});
    parameterSet.metric.weightPsi                                      = 3.0;
    parameterSet.metric.weightSway                                     = 2.0;
    parameterSet.motionPlanner.sampletime                              = 0.050000000000000;
    parameterSet.motionPlanner.samplingRangePosition                   = 10.0;
    parameterSet.motionPlanner.samplingRangeAngle                      = 1.0;
    parameterSet.motionPlanner.maxInputPathLength                      = 50.0;
    parameterSet.motionPlanner.maxPositionOvershoot                    = 10.0;
    parameterSet.motionPlanner.periodGoalSampling                      = 50;
    parameterSet.motionPlanner.regionOfAttraction.rangePose            = {0.25, 0.25, 0.15};
    parameterSet.motionPlanner.regionOfAttraction.rangeUVR             = {0.1, 0.1, 0.01};
    parameterSet.motionPlanner.regionOfAttraction.rangeXYN             = {10.0, 10.0, 10.0};
    parameterSet.motionPlanner.controller.vecTimeconstantsFlatStates   = {12.0, 15.0, 15.0, 1.0, 1.0, 1.0, 2.5, 2.5, 2.5};
    parameterSet.motionPlanner.controller.matK                         = {1.80025300316239, 0.0, 0.0, 14.9932006585738, 0.0, 0.0, 34.8623145223247, 0.0, 0.0, 21.6450501549992, 0.0, 0.0, 0.0, 2.25031625395272, 0.0, 0.0, 18.9915008232153, 0.0, 0.0, 44.4528931529019, 0.0, 0.0, 27.6813126937466, 0.0, 0.0, 0.0, 2.25031625395276, 0.0, 0.0, 18.9915008232156, 0.0, 0.0, 44.4528931529024, 0.0, 0.0, 27.681312693747};
    parameterSet.motionPlanner.controller.satUVR                       = {1.8, 0.8, 0.4};
    parameterSet.motionPlanner.controller.maxRadiusX                   = 10.0;
    parameterSet.motionPlanner.controller.maxRadiusY                   = 6.0;
    parameterSet.motionPlanner.controller.maxRadiusPsi                 = 1.0;
    parameterSet.motionPlanner.controller.minRadiusPosition            = 2.0;

    // Generate input data
    dataIn.initialStateAndInput = {-56.8924, -29.7461, 0.4302, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    dataIn.originOldToNew = {0.0, 0.0};
    dataIn.initialPath.clear();
    dataIn.initialPath.push_back({-56.8924,-29.7461,0.4302});
    dataIn.initialPath.push_back({-37.5868,-15.6169,0.7787});
    dataIn.initialPath.push_back({-28.4370,-6.3110,0.6899});
    dataIn.initialPath.push_back({-12.0707,3.0823,0.3009});
    dataIn.initialPath.push_back({3.0479,5.0647,-0.0196});
    dataIn.initialPath.push_back({17.3663,0.1339,-0.4240});
    dataIn.initialPath.push_back({32.9519,-9.1796,-0.3449});
    dataIn.initialPath.push_back({42.4645,-13.4085,-0.2170});
    dataIn.initialPath.push_back({53.6586,-12.3645,0.0266});
    dataIn.initialPath.push_back({54.4740,-8.8271,-0.1608});
    GenerateObstaclesForPlanning(dataIn.staticObstacles);

    // Initialize planner and apply parameters
    if(!planner.Initialize(maxNumNodes, maxNumSamples)){
        fprintf(stderr,"ERROR: Failed to initialize the motion planner!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(!planner.ApplyParameterSet(parameterSet)){
        fprintf(stderr,"ERROR: Failed to set parameters for the motion planner!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(!dataIn.IsValid()){
        fprintf(stderr,"ERROR: Input data for the motion planner is invalid!\n");
        planner.Terminate();
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Run actual motion planning
    executionTimer.Start();
    planner.Prepare(dataOut, dataIn);
    planner.Solve(dataOut, dataIn, maxIterations);
    double t = executionTimer.TimeToStart();

    // Terminate the planner
    planner.Terminate();
    preventOptimization += dataOut.cost;
    return t;
}

void Benchmark::PrintResultHeader(void){
    printf("\n\n");
    printf("    Resulting computation times (lower is better)\n\n\n");
    printf("    BENCHMARK                            Average        Minimum        Maximum       Std. Dev.\n");
    printf("    Name                                s  ms  us      s  ms  us      s  ms  us      s  ms  us\n");
    printf("    ------------------------------------------------------------------------------------------\n");
}

void Benchmark::PrintResultData(std::string name, std::vector<double>& times){
    // Ensure a maximum string length
    auto L = name.length();
    name.resize(30, ' ');
    if(L > 30){
        name[27] = name[28] = name[29] = '.';
    }

    // Calculate average, min, max, std. dev values
    double tAvg = 0.0;
    double tMin = std::numeric_limits<double>::infinity();
    double tMax = 0.0;
    double tStd = 0.0;
    for(auto&& t : times){
        tAvg += t;
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }
    if(times.size()){
        tAvg /= static_cast<double>(times.size());
        for(auto&& t : times){
            double dt = t - tAvg;
            tStd += (dt * dt);
        }
        tStd = std::sqrt(tStd / static_cast<double>(times.size()));
    }
    else{
        tMin = 0.0;
    }

    // Print result
    int32_t avgS = static_cast<int32_t>(std::floor(tAvg));
    int32_t avgM = static_cast<int32_t>(std::floor(tAvg * 1e3)) % 1000;
    int32_t avgU = static_cast<int32_t>(std::floor(tAvg * 1e6)) % 1000;
    int32_t minS = static_cast<int32_t>(std::floor(tMin));
    int32_t minM = static_cast<int32_t>(std::floor(tMin * 1e3)) % 1000;
    int32_t minU = static_cast<int32_t>(std::floor(tMin * 1e6)) % 1000;
    int32_t maxS = static_cast<int32_t>(std::floor(tMax));
    int32_t maxM = static_cast<int32_t>(std::floor(tMax * 1e3)) % 1000;
    int32_t maxU = static_cast<int32_t>(std::floor(tMax * 1e6)) % 1000;
    int32_t stdS = static_cast<int32_t>(std::floor(tStd));
    int32_t stdM = static_cast<int32_t>(std::floor(tStd * 1e3)) % 1000;
    int32_t stdU = static_cast<int32_t>(std::floor(tStd * 1e6)) % 1000;
    printf("    %s    %03d %03d %03d    %03d %03d %03d    %03d %03d %03d    %03d %03d %03d\n", name.c_str(), avgS, avgM, avgU, minS, minM, minU, maxS, maxM, maxU, stdS, stdM, stdU);
}

void Benchmark::PrintResultFooter(void){
    printf("    ------------------------------------------------------------------------------------------\n\n\n");
}

void Benchmark::PrintSystemInfo(void){
    // Title
    printf("MPSV Benchmark\n\n");

    // Operating system
    printf("Operating System:         %s\n", strOS.c_str());
    #if __linux__
    struct utsname info;
    (void) uname(&info);
    printf("sysname:                  %s\n", info.sysname);
    printf("nodename:                 %s\n", info.nodename);
    printf("release:                  %s\n", info.release);
    printf("version:                  %s\n", info.version);
    printf("machine:                  %s\n", info.machine);
    #ifdef __USE_GNU
    printf("domainname:               %s\n", info.domainname);
    #endif
    #elif _WIN32
    OSVERSIONINFO osvi;
    ZeroMemory(&osvi, sizeof(OSVERSIONINFO));
    osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
    (void) GetVersionEx(&osvi);
    printf("dwBuildNumber:            %ld\n", osvi.dwBuildNumber);
    printf("dwMajorVersion:           %ld\n", osvi.dwMajorVersion);
    printf("dwMinorVersion:           %ld\n", osvi.dwMinorVersion);
    printf("dwPlatformId:             %ld\n", osvi.dwPlatformId);
    printf("szCSDVersion:             %s\n", osvi.szCSDVersion);
    #endif

    // Version and date
    auto timePoint = std::chrono::system_clock::now();
    std::time_t systemTime = std::chrono::system_clock::to_time_t(timePoint);
    std::tm* gmTime = std::gmtime(&systemTime);
    printf("Benchmark Version:        %s\n", strVersion.c_str());
    printf("MPSV API Version:         %u\n", MPSV_VERSION);
    printf("Compiler Version:         %s\n", strCompilerVersion.c_str());
    printf("Built (local):            %s\n", strBuilt.c_str());
    printf("Current time (UTC):       %04u-%02u-%02u %02u:%02u:%02u\n", 1900 + gmTime->tm_year, 1 + gmTime->tm_mon, gmTime->tm_mday, gmTime->tm_hour, gmTime->tm_min, gmTime->tm_sec);
    #ifdef DEBUG
    printf("DEBUG:                    1\n");
    #else
    printf("DEBUG:                    0\n");
    #endif
    #ifdef MPSV_DONT_USE_OMP
    printf("MPSV_DONT_USE_OMP:        1\n");
    #else
    printf("MPSV_DONT_USE_OMP:        0\n");
    printf("OMP max threads:          %d\n",omp_get_max_threads());
    #endif
    printf("\n");
}

void Benchmark::GenerateObstaclesForPlanning(std::vector<mpsv::geometry::StaticObstacle>& staticObstacles){
    staticObstacles.clear();
    staticObstacles.push_back(StaticObstacle({{-79.9851,-100.0000},{-21.6209,-62.3559},{-18.5046,-60.6264},{-16.1786,-60.0077},{-13.2405,-59.3359},{100.0000,-100.0000}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,-100.0000},{-8.6330,-55.8570},{-6.2624,-51.6664},{33.3245,-25.4451},{40.6811,-25.9506},{54.9267,-27.0015}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,-100.0000},{76.4063,-10.2128},{76.6623,-9.7605},{81.1169,-1.9088},{81.4029,-1.4088},{82.6338,0.7252},{92.1956,17.3020},{94.3474,21.0324}}));
    staticObstacles.push_back(StaticObstacle({{-6.3067,70.9910},{-3.4134,48.4085},{-4.5375,45.4551},{-6.2291,42.8809},{-7.7428,40.6061},{-9.2564,38.3778},{-40.4185,54.3090},{-37.5692,67.2133},{-8.0763,72.8402}}));
    staticObstacles.push_back(StaticObstacle({{76.4063,-10.2128},{100.0000,-100.0000},{68.3598,-24.2743},{70.7556,-20.0477},{71.0410,-19.5477}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,-100.0000},{94.6355,21.5324},{99.9990,30.8508}}));
    staticObstacles.push_back(StaticObstacle({{-8.6330,-55.8570},{100.0000,-100.0000},{-13.2405,-59.3359}}));
    staticObstacles.push_back(StaticObstacle({{-21.8216,17.8573},{-49.5448,23.8440},{-44.3695,41.7970},{-40.4185,54.3090},{-10.2803,35.6306},{-13.0293,27.8681},{-14.6987,23.9968},{-16.4461,21.7219},{-19.1728,19.6133}}));
    staticObstacles.push_back(StaticObstacle({{-32.3158,85.3259},{-8.9443,75.2016},{-8.0763,72.8402},{-37.5692,67.2133}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,-100.0000},{54.9267,-27.0015},{61.5932,-27.3341},{64.1195,-27.5203}}));
    staticObstacles.push_back(StaticObstacle({{68.3598,-24.2743},{100.0000,-100.0000},{64.1195,-27.5203}}));
    staticObstacles.push_back(StaticObstacle({{-46.3280,-68.0501},{-43.6236,-67.1454},{-35.9666,-65.3826},{-29.2333,-64.0056},{-79.9851,-100.0000},{-79.0367,-96.2941},{-76.7665,-89.3230},{-74.9859,-85.3385},{-73.2163,-82.0858},{-71.7695,-80.2831},{-71.5804,-80.0636},{-69.5326,-78.0148},{-64.4576,-75.2675},{-59.1490,-72.8728},{-52.6161,-70.1655}}));
    staticObstacles.push_back(StaticObstacle({{-6.2624,-51.6664},{10.1643,-27.8399},{21.5274,-24.9131},{24.3543,-24.6403},{26.4911,-24.6204},{33.3245,-25.4451}}));
    staticObstacles.push_back(StaticObstacle({{15.3565,-19.7760},{10.1643,-27.8399},{-6.1737,-13.9712},{-7.4355,-6.5240},{-5.2166,-3.8473},{-4.0035,-3.6810},{-2.1671,-5.1843}}));
    staticObstacles.push_back(StaticObstacle({{54.9609,95.3683},{20.8485,100.0000},{63.5557,100.0000},{63.5307,99.8580},{62.3286,98.1552},{59.2235,96.6320}}));
    staticObstacles.push_back(StaticObstacle({{-29.2333,-64.0056},{-21.6209,-62.3559},{-79.9851,-100.0000}}));
    staticObstacles.push_back(StaticObstacle({{-38.5602,2.9441},{-40.3631,3.0506},{-42.4666,3.5428},{-45.2044,4.9264},{-46.5733,6.1835},{-48.8103,9.2500},{-50.3907,12.1568},{-51.2254,13.8597},{-51.4591,15.8885},{-51.2810,17.5315},{-51.0696,19.4006},{-49.5448,23.8440},{-24.0141,15.6024},{-25.4386,13.6068},{-32.5726,4.1680},{-35.8112,3.0306}}));
    staticObstacles.push_back(StaticObstacle({{-29.8449,93.8135},{-10.5912,88.2057},{-9.8567,86.1104},{-9.1779,82.6382},{-8.9888,77.6627},{-8.9443,75.2016},{-32.3158,85.3259}}));
    staticObstacles.push_back(StaticObstacle({{-27.5299,98.4630},{-10.0679,99.4937},{-10.8472,90.4740},{-28.1198,97.1061}}));
    staticObstacles.push_back(StaticObstacle({{-65.6375,-64.6846},{-42.0878,-59.6954},{-46.2835,-65.6887},{-47.7526,-66.2076},{-64.5356,-69.7399},{-65.0536,-67.3635}}));
    staticObstacles.push_back(StaticObstacle({{41.7057,90.1537},{20.8485,100.0000},{51.9226,93.9183},{48.9733,92.4750}}));
    staticObstacles.push_back(StaticObstacle({{-10.2803,35.6306},{-40.4185,54.3090},{-9.2564,38.3778}}));
    staticObstacles.push_back(StaticObstacle({{29.7972,83.7683},{27.3153,83.6087},{19.0574,87.2274},{19.3580,93.4600},{19.9146,97.2714},{20.8485,100.0000},{38.3001,88.8966},{35.7181,86.6617}}));
    staticObstacles.push_back(StaticObstacle({{-21.8216,17.8573},{-24.0141,15.6024},{-49.5448,23.8440}}));
    staticObstacles.push_back(StaticObstacle({{21.5274,-24.9131},{10.1643,-27.8399},{15.8565,-20.1923}}));
    staticObstacles.push_back(StaticObstacle({{54.9609,95.3683},{51.9226,93.9183},{20.8485,100.0000}}));
    staticObstacles.push_back(StaticObstacle({{41.7057,90.1537},{38.3001,88.8966},{20.8485,100.0000}}));
    staticObstacles.push_back(StaticObstacle({{74.5699,-6.0555},{69.9179,-7.6320},{50.4303,-5.2574},{50.5194,-4.4658},{50.5973,-3.8273},{50.6974,-3.1355}}));
    staticObstacles.push_back(StaticObstacle({{-2.8791,49.3996},{-6.3067,70.9910},{-3.5801,68.2838},{-2.1112,50.5171}}));
    staticObstacles.push_back(StaticObstacle({{98.4092,30.8207},{86.1890,9.5624},{85.4767,9.9748},{92.6664,24.7080},{93.0003,25.2667}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,100.0000},{93.5354,96.4451},{86.7561,100.0000}}));
    staticObstacles.push_back(StaticObstacle({{-10.5912,88.2057},{-29.8449,93.8135},{-29.0102,95.6028},{-28.1198,97.1061},{-10.8472,90.4740}}));
    staticObstacles.push_back(StaticObstacle({{98.8440,93.3454},{93.5354,96.4451},{100.0000,100.0000}}));
    staticObstacles.push_back(StaticObstacle({{100.0000,-100.0000},{94.3474,21.0324},{94.6355,21.5324}}));
    staticObstacles.push_back(StaticObstacle({{82.4050,7.4339},{66.1339,12.3162},{69.2056,13.3605},{75.9389,11.3517},{76.5287,11.1721}}));
    staticObstacles.push_back(StaticObstacle({{-0.7200,52.3596},{-2.1112,50.5171},{-3.5801,68.2838},{-0.8200,67.0798},{1.5060,64.7383},{1.6617,57.2153},{0.6601,54.4548}}));
    staticObstacles.push_back(StaticObstacle({{-40.9525,-64.6510},{-46.2835,-65.6887},{-42.0878,-59.6954}}));
    staticObstacles.push_back(StaticObstacle({{-27.5299,98.4630},{-27.2346,100.0000},{-10.0301,100.0000},{-10.0679,99.4937}}));
    staticObstacles.push_back(StaticObstacle({{27.3153,83.6087},{24.3215,82.0323},{22.0400,80.9415},{20.4373,80.7021},{18.8458,81.1079},{18.0780,83.3029},{19.0574,87.2274}}));
    staticObstacles.push_back(StaticObstacle({{83.4178,5.0859},{77.2521,-5.2241},{76.7513,-4.9314},{75.9277,-4.4259},{79.4001,2.1525},{81.7261,6.0969}}));
    staticObstacles.push_back(StaticObstacle({{-6.1737,-13.9712},{-10.1135,-10.6187},{-7.8510,-7.0240},{-7.4355,-6.5240}}));
    staticObstacles.push_back(StaticObstacle({{17.2187,26.0548},{19.6020,26.2316},{14.0929,17.4847},{12.4681,18.5090}}));
    staticObstacles.push_back(StaticObstacle({{85.4767,9.9748},{84.3193,8.8307},{84.5641,10.4936},{92.6664,24.7080}}));
    staticObstacles.push_back(StaticObstacle({{-27.2305,8.4052},{-28.7997,6.5760},{-30.5581,5.1658},{-32.5726,4.1680},{-25.4386,13.6068},{-26.1620,10.5138},{-26.2399,10.3542}}));
    staticObstacles.push_back(StaticObstacle({{-2.8791,49.3996},{-3.4134,48.4085},{-6.3067,70.9910}}));
    staticObstacles.push_back(StaticObstacle({{46.7243,20.1053},{47.5812,17.8770},{40.2804,20.0654},{39.6905,20.2450},{39.2787,22.3403},{39.8797,22.1607}}));
    staticObstacles.push_back(StaticObstacle({{32.6901,22.3469},{25.6675,24.4489},{25.3225,26.5242},{31.6885,24.6218},{32.2784,24.4422}}));
    staticObstacles.push_back(StaticObstacle({{32.2784,24.4422},{39.2787,22.3403},{39.6905,20.2450},{33.2800,22.1674},{32.6901,22.3469}}));
    staticObstacles.push_back(StaticObstacle({{61.8046,15.5888},{68.6157,13.5401},{69.2056,13.3605},{66.1339,12.3162},{65.5329,12.4891},{61.0589,13.8328},{61.2036,15.7617}}));
    staticObstacles.push_back(StaticObstacle({{55.1269,15.6154},{54.4703,17.7838},{61.2036,15.7617},{61.0589,13.8328},{60.4579,14.0123}}));
    staticObstacles.push_back(StaticObstacle({{82.4050,7.4339},{76.5287,11.1721},{83.2175,9.1633},{84.3193,8.8307},{83.8185,7.0148},{83.2954,7.1612}}));
    staticObstacles.push_back(StaticObstacle({{47.3141,19.9323},{53.8693,17.9634},{54.5260,15.7950},{48.1711,17.6974},{47.5812,17.8770},{46.7243,20.1053}}));
    staticObstacles.push_back(StaticObstacle({{93.0003,25.2667},{95.9496,30.4083},{96.2723,30.9671},{98.4092,30.8207}}));
    staticObstacles.push_back(StaticObstacle({{24.7327,26.7038},{25.3225,26.5242},{25.0665,24.6285},{20.4590,25.9722},{20.4924,26.1052},{20.6927,26.7571}}));
    staticObstacles.push_back(StaticObstacle({{1.6617,57.2153},{1.5060,64.7383},{2.9083,61.9712},{2.7524,59.8227}}));
    staticObstacles.push_back(StaticObstacle({{99.9998,92.5980},{98.8440,93.3454},{100.0000,100.0000}}));
    staticObstacles.push_back(StaticObstacle({{75.7163,-8.3437},{71.9657,-7.8847},{69.9179,-7.6320},{74.5699,-6.0555},{75.7163,-6.1952},{75.9611,-6.2285}}));
    staticObstacles.push_back(StaticObstacle({{35.7181,86.6617},{33.3030,84.3869},{29.7972,83.7683}}));
    staticObstacles.push_back(StaticObstacle({{10.1643,-27.8399},{15.3565,-19.7760},{15.8565,-19.7342},{15.8565,-20.1923}}));
    staticObstacles.push_back(StaticObstacle({{21.0043,27.8213},{24.7327,26.7038},{20.6927,26.7571}}));
    staticObstacles.push_back(StaticObstacle({{19.6020,26.2316},{17.5332,26.5548},{18.6004,28.2537},{19.9916,26.8502},{20.4924,26.1052}}));
    staticObstacles.push_back(StaticObstacle({{70.3297,19.2671},{70.9307,19.0808},{69.2056,13.3605},{68.6157,13.5401}}));
    staticObstacles.push_back(StaticObstacle({{41.5937,27.8877},{39.8797,22.1607},{39.2787,22.3403},{41.0038,28.0673}}));
    staticObstacles.push_back(StaticObstacle({{23.3526,18.9081},{25.0665,24.6285},{25.6675,24.4489},{23.9424,18.7285}}));
    staticObstacles.push_back(StaticObstacle({{48.1711,17.6974},{46.4572,11.9770},{45.8562,12.1566},{47.5812,17.8770}}));
    staticObstacles.push_back(StaticObstacle({{52.8120,10.0746},{54.5260,15.7950},{55.1269,15.6154},{53.4019,9.8950}}));
    staticObstacles.push_back(StaticObstacle({{56.1843,23.5042},{54.4703,17.7838},{53.8693,17.9634},{55.5944,23.6838}}));
    staticObstacles.push_back(StaticObstacle({{58.7440,8.2920},{60.4579,14.0123},{61.0589,13.8328},{59.3450,8.1124}}));
    staticObstacles.push_back(StaticObstacle({{61.2036,15.7617},{62.9175,21.4821},{63.5185,21.3092},{61.8046,15.5888}}));
    staticObstacles.push_back(StaticObstacle({{64.4200,6.5958},{63.8190,6.7687},{65.5329,12.4891},{66.1339,12.3162}}));
    staticObstacles.push_back(StaticObstacle({{26.4466,32.4309},{27.0365,32.2513},{25.3225,26.5242},{24.7327,26.7038}}));
    staticObstacles.push_back(StaticObstacle({{32.6901,22.3469},{33.2800,22.1674},{31.5549,16.4469},{30.9762,16.6265}}));
    staticObstacles.push_back(StaticObstacle({{39.6905,20.2450},{40.2804,20.0654},{38.5664,14.3450},{37.9766,14.5246}}));
    staticObstacles.push_back(StaticObstacle({{77.6528,17.0720},{78.2427,16.8924},{76.5287,11.1721},{75.9389,11.3517}}));
    staticObstacles.push_back(StaticObstacle({{32.2784,24.4422},{31.6885,24.6218},{33.4025,30.3356},{33.9923,30.1626}}));
    staticObstacles.push_back(StaticObstacle({{46.7243,20.1053},{48.4382,25.8257},{49.0281,25.6527},{47.3141,19.9323}}));
    staticObstacles.push_back(StaticObstacle({{-46.2835,-65.6887},{-44.8367,-67.5512},{-46.3280,-68.0501},{-47.7526,-66.2076}}));
    staticObstacles.push_back(StaticObstacle({{19.4796,29.6705},{20.3144,36.0893},{20.8486,36.0162},{20.0138,29.6040}}));
    staticObstacles.push_back(StaticObstacle({{76.6623,-9.7605},{76.4063,-10.2128},{70.5268,-7.8742},{69.9179,-7.6320},{71.9657,-7.8847}}));
    staticObstacles.push_back(StaticObstacle({{48.9733,92.4750},{46.5359,91.2777},{41.7057,90.1537}}));
    staticObstacles.push_back(StaticObstacle({{75.9277,-4.4259},{75.5827,-4.2264},{79.4001,2.1525}}));
    staticObstacles.push_back(StaticObstacle({{93.0003,25.2667},{92.6664,24.7080},{89.1272,26.7567},{89.4389,27.3088}}));
    staticObstacles.push_back(StaticObstacle({{95.9496,30.4083},{92.3993,32.4570},{92.7221,33.0158},{96.2723,30.9671}}));
    staticObstacles.push_back(StaticObstacle({{46.5350,-3.3617},{50.5973,-3.8273},{50.5194,-4.4658},{46.4571,-3.9936}}));
    staticObstacles.push_back(StaticObstacle({{75.7163,-6.1952},{74.5699,-6.0555},{75.9277,-4.4259},{76.7513,-4.9314}}));
    staticObstacles.push_back(StaticObstacle({{-10.0912,-9.7141},{-7.8510,-7.0240},{-10.1135,-10.6187}}));
    staticObstacles.push_back(StaticObstacle({{83.2175,9.1633},{84.5641,10.4936},{84.3193,8.8307}}));
    staticObstacles.push_back(StaticObstacle({{98.4092,30.8207},{96.2723,30.9671},{96.7398,31.7786}}));
    staticObstacles.push_back(StaticObstacle({{82.6721,5.5315},{81.8486,6.0237},{82.4050,7.4339},{83.2954,7.1612}}));
    staticObstacles.push_back(StaticObstacle({{25.6675,24.4489},{25.0665,24.6285},{25.3225,26.5242}}));
    staticObstacles.push_back(StaticObstacle({{54.4703,17.7838},{55.1269,15.6154},{54.5260,15.7950},{53.8693,17.9634}}));
    staticObstacles.push_back(StaticObstacle({{17.5332,26.5548},{19.6020,26.2316},{17.2187,26.0548},{17.1261,26.0548},{17.1261,26.5548}}));
    staticObstacles.push_back(StaticObstacle({{20.2364,27.2227},{19.9916,26.8502},{18.6004,28.2537}}));
    staticObstacles.push_back(StaticObstacle({{20.4924,26.1052},{19.9916,26.8502},{20.6927,26.7571}}));
    staticObstacles.push_back(StaticObstacle({{-40.0015,-58.9809},{-40.5015,-58.9809},{-40.5015,-58.4809},{-40.0015,-58.4809}}));
    staticObstacles.push_back(StaticObstacle({{-44.6415,-66.9498},{-44.6415,-66.4498},{-44.1415,-66.4498},{-44.1415,-66.9498}}));
    staticObstacles.push_back(StaticObstacle({{-15.7163,-59.4462},{-15.7164,-58.9462},{-15.2164,-58.9462},{-15.2163,-59.4462}}));
    staticObstacles.push_back(StaticObstacle({{-33.0455,-63.8033},{-33.5455,-63.8033},{-33.5456,-63.3033},{-33.0456,-63.3033}}));
    staticObstacles.push_back(StaticObstacle({{-1.9051,-19.3430},{-1.9051,-18.8430},{-1.4051,-18.8430},{-1.4051,-19.3430}}));
    staticObstacles.push_back(StaticObstacle({{-11.2210,-54.6425},{-11.2210,-55.1425},{-11.7210,-55.1425},{-11.7210,-54.6425}}));
    staticObstacles.push_back(StaticObstacle({{-67.3232,-65.1608},{-67.3232,-64.6608},{-66.8232,-64.6607},{-66.8232,-65.1607}}));
    staticObstacles.push_back(StaticObstacle({{-40.9911,-61.8800},{-40.4911,-61.8800},{-40.4911,-62.3800},{-40.9911,-62.3800}}));
    staticObstacles.push_back(StaticObstacle({{-22.6841,-61.5483},{-23.1841,-61.5483},{-23.1841,-61.0483},{-22.6841,-61.0483}}));
    staticObstacles.push_back(StaticObstacle({{4.8727,-25.3162},{4.8727,-24.8162},{5.3727,-24.8162},{5.3727,-25.3162}}));
    staticObstacles.push_back(StaticObstacle({{87.5305,10.1704},{87.5305,10.6704},{88.0305,10.6704},{88.0305,10.1704}}));
    staticObstacles.push_back(StaticObstacle({{50.7805,-5.3000},{51.1589,-5.3462},{51.1589,-5.8000},{50.6589,-5.8000},{50.6589,-5.3000}}));
    staticObstacles.push_back(StaticObstacle({{66.5620,12.0274},{66.5620,11.5274},{66.0620,11.5274},{66.0620,12.0274}}));
    staticObstacles.push_back(StaticObstacle({{20.4873,37.2706},{20.9873,37.2706},{20.9873,36.7706},{20.4873,36.7706}}));
    staticObstacles.push_back(StaticObstacle({{70.1019,-8.3742},{70.1019,-7.8742},{70.5268,-7.8742},{70.6019,-7.9041},{70.6019,-8.3742}}));
    staticObstacles.push_back(StaticObstacle({{19.2185,28.4693},{19.2185,28.9693},{19.7185,28.9693},{19.7185,28.4693}}));
    staticObstacles.push_back(StaticObstacle({{73.4065,-14.9726},{72.9065,-14.9726},{72.9065,-14.4726},{73.4065,-14.4726}}));
    staticObstacles.push_back(StaticObstacle({{-11.4658,-55.6868},{-11.4658,-56.1868},{-11.9658,-56.1868},{-11.9658,-55.6868}}));
    staticObstacles.push_back(StaticObstacle({{82.1439,0.7252},{82.1439,1.2252},{82.6439,1.2252},{82.6439,0.7426},{82.6338,0.7252}}));
    staticObstacles.push_back(StaticObstacle({{92.9951,20.2487},{93.4951,20.2487},{93.4951,19.7487},{92.9951,19.7487}}));
    staticObstacles.push_back(StaticObstacle({{-30.3968,-62.9108},{-30.3968,-63.4108},{-30.8968,-63.4109},{-30.8968,-62.9109}}));
    staticObstacles.push_back(StaticObstacle({{98.9597,29.5730},{98.4597,29.5730},{98.4597,30.0730},{98.9597,30.0730}}));
    staticObstacles.push_back(StaticObstacle({{-65.0536,-67.6618},{-65.5536,-67.6618},{-65.5536,-67.1618},{-65.0976,-67.1618},{-65.0536,-67.3635}}));
    staticObstacles.push_back(StaticObstacle({{-7.8510,-7.0240},{-7.8928,-7.0240},{-7.8928,-6.5240},{-7.4355,-6.5240}}));
    staticObstacles.push_back(StaticObstacle({{81.4029,-1.4088},{81.1169,-1.9088},{81.0087,-1.9088},{81.0087,-1.4088}}));
    staticObstacles.push_back(StaticObstacle({{94.2416,21.5324},{94.6355,21.5324},{94.3474,21.0324},{94.2416,21.0324}}));
    staticObstacles.push_back(StaticObstacle({{70.6473,-19.5477},{71.0410,-19.5477},{70.7556,-20.0477},{70.6473,-20.0477}}));
    staticObstacles.push_back(StaticObstacle({{-47.1503,-68.3267},{-47.3014,-68.3267},{-47.3014,-67.8267},{-46.8014,-67.8267},{-46.8014,-68.2094}}));
    staticObstacles.push_back(StaticObstacle({{13.3078,17.9797},{13.0305,17.9797},{13.0305,18.1544}}));
    staticObstacles.push_back(StaticObstacle({{15.3565,-19.7760},{15.3565,-19.7342},{15.8565,-19.7342}}));
    staticObstacles.push_back(StaticObstacle({{91.9692,16.9096},{91.8710,16.7393},{91.8710,16.9096}}));
    staticObstacles.push_back(StaticObstacle({{92.1956,17.3020},{92.1492,17.2216},{92.1492,17.3020}}));
    for(auto&& p : staticObstacles){
        p.EnsureCorrectVertexOrder();
    }
}

