#include <IterationPerformanceTest.hpp>


SceneDataFile IterationPerformanceTest::m_sceneData;


void IterationPerformanceTest::Run(void){
    if(IterationPerformanceTest::LoadSceneData()){
        for(size_t i = 0; i < m_sceneData.scenes.size(); ++i){
            std::cout << "solve " << (i + 1) << " / " << m_sceneData.scenes.size() << "\n";
            IterationPerformanceTest::Solve(m_sceneData.scenes[i], !i);
        }
    }
}

bool IterationPerformanceTest::LoadSceneData(void){
    printf("Loading sceneData: ");
    bool success = m_sceneData.Read("sceneData");
    if(success){
        printf("OK (number of scenes: %zu)\n", m_sceneData.scenes.size());
    }
    else{
        printf("ERROR: Failed to read sceneData from the current working directory!\n");
    }
    bool valid = m_sceneData.IsValid();
    if(!valid){
        printf("ERROR: The scene data is not valid!\n");
    }
    return success && valid;
}

mpsv::planner::SequentialPlannerParameterSet IterationPerformanceTest::GetParameterSet(void){
    mpsv::planner::SequentialPlannerParameterSet parameterSet;

    // cost map
    parameterSet.costMap.modBreakpoints                                = 10;
    parameterSet.costMap.resolution                                    = 0.05;
    parameterSet.costMap.distanceScale                                 = 5.0;
    parameterSet.costMap.distanceDecay                                 = 0.02;

    // geometry
    parameterSet.geometry.collisionCheckMaxPositionDeviation           = 0.100000000000000;
    parameterSet.geometry.collisionCheckMaxAngleDeviation              = mpsv::math::deg2rad(5.0);
    std::vector<std::array<double,2>> vertices                         = {{3.25,1.65},{-3.25,1.65},{-3.25,-1.65},{3.25,-1.65}};
    parameterSet.geometry.vehicleShape.Create(vertices);
    parameterSet.geometry.skeletalPoints.clear();
    parameterSet.geometry.skeletalPoints.push_back({3.0, 0.0});
    parameterSet.geometry.skeletalPoints.push_back({-3.0, 0.0});

    // metric
    parameterSet.metric.weightPsi                                      = 3.0;
    parameterSet.metric.weightSway                                     = 2.0;
    parameterSet.metric.weightReverseScale                             = 0.0;
    parameterSet.metric.weightReverseDecay                             = 1.0;

    // path planner
    parameterSet.pathPlanner.periodGoalSampling                        = 50;

    // model
    parameterSet.model.matF                                            = {-0.031604977390302, 0.0, 0.0, 0.233741474945168, 0.0, 0.0, 0.0, 0.0, 0.133462292221887, -0.020792892088185, 0.0, 0.0, 0.0, -0.041708610292712, 0.017242263124379, 0.0, -0.251243228165994, -0.000476044356140, 0.0, 0.0, 0.005297185500344, 0.0, -0.167981638498434, 0.497442136687157, 0.0, 0.000017966020615, -0.023528504195578, 0.0, 0.000108223241795, 0.000649602175186, 0.0, 0.0, -0.000002281767319, 0.0, 0.000072358238719, -0.678801229030825};
    parameterSet.model.matB                                            = {0.000237948834266, -0.000004551592718, 0.000010003488944, -0.000009313932115, 0.000215194147058, -0.000024957572224, -0.000002202124158, -0.000002930260852, 0.000043018345190};
    parameterSet.model.vecTimeconstantsXYN                             = {0.2, 0.2, 0.2};
    parameterSet.model.vecTimeconstantsInput                           = {0.5, 0.5, 0.5};
    parameterSet.model.lowerLimitXYN                                   = {-630.0, -495.0, -675.0};
    parameterSet.model.upperLimitXYN                                   = {630.0, 495.0, 675.0};

    // motion planner
    parameterSet.motionPlanner.samplingRangePosition                   = 10.0;
    parameterSet.motionPlanner.samplingRangeAngle                      = 1.0;
    parameterSet.motionPlanner.periodGoalSampling                      = 50;
    parameterSet.motionPlanner.sampletime                              = 0.05;
    parameterSet.motionPlanner.maxInputPathLength                      = 50.0;
    parameterSet.motionPlanner.controller.matK                         = {0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35};
    parameterSet.motionPlanner.controller.maxRadiusX                   = 10.0;
    parameterSet.motionPlanner.controller.maxRadiusY                   = 6.0;
    parameterSet.motionPlanner.controller.maxRadiusPsi                 = 1.0;
    parameterSet.motionPlanner.controller.minRadiusPosition            = 2.0;
    parameterSet.motionPlanner.regionOfAttraction.rangePose            = {0.25, 0.25, 0.15};
    parameterSet.motionPlanner.regionOfAttraction.rangeUVR             = {0.1, 0.1, 0.01};
    parameterSet.motionPlanner.regionOfAttraction.rangeXYN             = {10.0, 10.0, 10.0};
    
    return parameterSet;
}

mpsv::planner::SequentialPlannerInput IterationPerformanceTest::GetInput(Scene& scene){
    mpsv::planner::SequentialPlannerInput dataIn;
    dataIn.initialStateAndInput = scene.initialStateAndInput;
    dataIn.finalPose = scene.finalPose;
    dataIn.originOldToNew.fill(0.0);
    mpsv::geometry::OrientedBox box;
    box.CreateFromFocalPoints(dataIn.initialStateAndInput, dataIn.finalPose, {25.0, 50.0});
    auto n = box.GetNormal();
    dataIn.samplingBoxCenterPose[0] = (dataIn.initialStateAndInput[0] + dataIn.finalPose[0]) / 2.0;
    dataIn.samplingBoxCenterPose[1] = (dataIn.initialStateAndInput[1] + dataIn.finalPose[1]) / 2.0;
    dataIn.samplingBoxCenterPose[2] = std::atan2(n[1], n[0]);
    dataIn.samplingBoxDimension = box.GetDimension();
    dataIn.staticObstacles = scene.staticObstacles;
    return dataIn;
}

void IterationPerformanceTest::Solve(Scene& scene, bool recreateFile){
    // construct and initialize the sequential planner
    mpsv::planner::SequentialPlanner planner;
    constexpr int16_t pathMaxNumNodes = 200;
    constexpr uint32_t pathMaxNumSamples = 1000000;
    constexpr int16_t motionMaxNumNodes = 200;
    constexpr uint32_t motionMaxNumSamples = 1000000;
    constexpr double maxComputationTimePathPlanning = 1.0;
    constexpr double maxComputationTimeMotionPlanning = 1.0;
    planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples);

    // apply a parameter set to the sequential planner
    mpsv::planner::SequentialPlannerParameterSet parameter = IterationPerformanceTest::GetParameterSet();
    mpsv::error_code ec = planner.ApplyParameterSet(parameter);
    if(mpsv::error_code::NONE != ec){
        std::cerr << "ERROR: Could not apply parameter set (code: " << (int)(ec) << ")!\n";
        return;
    }

    // get valid input data for planning problem
    mpsv::planner::SequentialPlannerInput dataIn = IterationPerformanceTest::GetInput(scene);
    ec = dataIn.IsValid();
    if(mpsv::error_code::NONE != ec){
        std::cerr << "ERROR: Input data is invalid (code: " << (int)(ec) << ")!\n";
        return;
    }

    // solve the sequential planning problem
    mpsv::planner::SequentialPlannerOutput dataOut;
    planner.Solve(dataOut, dataIn, maxComputationTimePathPlanning, maxComputationTimeMotionPlanning);

    // write results to file
    mpsv::logging::File f;
    std::string filename = "dataIterationPerformanceTest";
    if(f.Open(filename, !recreateFile)){
        //parameter.WriteToFile(f, "parameter.");
        //dataIn.WriteToFile(f, "input.");
        dataOut.WriteToFile(f, "output.");
        //planner.WriteToFile(f, "planner.", dataIn.staticObstacles);
        f.Close();
    }
}
