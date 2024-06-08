#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>


MPSV_WrapperAsynchronousOnlinePlanner::MPSV_WrapperAsynchronousOnlinePlanner(){}

MPSV_WrapperAsynchronousOnlinePlanner::~MPSV_WrapperAsynchronousOnlinePlanner(){
    Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Initialize(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic){
    (void) planner.Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, threadPriority, ompNumThreads, ompDynamic);
}

void MPSV_WrapperAsynchronousOnlinePlanner::Terminate(void){
    planner.Terminate();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Step(SerializationAsynchronousOnlinePlannerOutputUnion* output, SerializationAsynchronousOnlinePlannerInputUnion* input, SerializationAsynchronousOnlinePlannerParameterUnion* parameter){
    // read and set parameter
    ReadFromParameter(parameter);
    planner.SetParameter(plannerParameter);

    // read and set input
    ReadFromInput(input);
    planner.SetInput(plannerInput);

    // get and write current output
    plannerOutput = planner.GetOutput();
    WriteToOutput(output);
}

void MPSV_WrapperAsynchronousOnlinePlanner::ReadFromInput(SerializationAsynchronousOnlinePlannerInputUnion* input){
    plannerInput.enable                = input->data.enable;
    plannerInput.reset                 = input->data.reset;
    plannerInput.timestamp             = input->data.timestamp;
    plannerInput.originLLA             = input->data.originLLA;
    plannerInput.initialStateAndInput  = input->data.initialStateAndInput;
    plannerInput.finalPose             = input->data.finalPose;
    plannerInput.samplingBoxCenterPose = input->data.samplingBoxCenterPose;
    plannerInput.samplingBoxDimension  = input->data.samplingBoxDimension;
    plannerInput.staticObstacles.clear();
    bool validStaticObstacles = (input->data.numStaticObstacles <= input->data.numVerticesPerStaticObstacle.size());
    for(size_t p = 0; (p < static_cast<size_t>(input->data.numStaticObstacles)) && validStaticObstacles; ++p){
        validStaticObstacles &= (input->data.numVerticesPerStaticObstacle[p] > 2) && (input->data.numVerticesPerStaticObstacle[p] <= input->data.verticesStaticObstacles[p].size());
        std::vector<std::array<double,2>> vertices(input->data.numVerticesPerStaticObstacle[p]);
        std::transform(input->data.verticesStaticObstacles[p].begin(), input->data.verticesStaticObstacles[p].begin() + input->data.numVerticesPerStaticObstacle[p], vertices.begin(), [](std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
        plannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
        validStaticObstacles &= plannerInput.staticObstacles.back().EnsureCorrectVertexOrder();
    }

    // if there're errors make whole input invalid (set timestampt o NaN)
    if(!validStaticObstacles){
        plannerInput.timestamp = std::numeric_limits<double>::quiet_NaN();
    }
}

void MPSV_WrapperAsynchronousOnlinePlanner::ReadFromParameter(SerializationAsynchronousOnlinePlannerParameterUnion* parameter){
    plannerParameter.timestamp                                                             = parameter->data.timestamp;
    plannerParameter.timeoutInput                                                          = parameter->data.timeoutInput;
    plannerParameter.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation         = parameter->data.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation;
    plannerParameter.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation            = parameter->data.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation;

    // vehicle shape
    plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
    bool validVehicleShape = (parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape > 0) && (parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape <= parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape.size());
    if(validVehicleShape){
        for(uint8_t p = 0; p < parameter->data.sequentialPlanner.geometry.numPolygonsVehicleShape; ++p){
            validVehicleShape &= (parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p] > 2) && (parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p] <= parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].size());
            if(!validVehicleShape){
                break;
            }
            for(uint8_t v = 0; v < parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p]; ++v){
                std::vector<std::array<double,2>> vertices(parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].begin(), parameter->data.sequentialPlanner.geometry.verticesVehicleShape[p].begin() + parameter->data.sequentialPlanner.geometry.numVerticesVehicleShape[p]);
                plannerParameter.sequentialPlanner.geometry.vehicleShape.Add(vertices);
            }
        }
        validVehicleShape &= plannerParameter.sequentialPlanner.geometry.vehicleShape.EnsureCorrectVertexOrder();
    }

    // skeletal points
    plannerParameter.sequentialPlanner.geometry.skeletalPoints.clear();
    bool validSkeletalPoints = (parameter->data.sequentialPlanner.geometry.numSkeletalPoints > 0) && (parameter->data.sequentialPlanner.geometry.numSkeletalPoints <= parameter->data.sequentialPlanner.geometry.skeletalPoints.size());
    if(validSkeletalPoints){
        plannerParameter.sequentialPlanner.geometry.skeletalPoints.resize(parameter->data.sequentialPlanner.geometry.numSkeletalPoints);
        for(uint8_t k = 0; (k < parameter->data.sequentialPlanner.geometry.numSkeletalPoints); ++k){
            plannerParameter.sequentialPlanner.geometry.skeletalPoints[k] = parameter->data.sequentialPlanner.geometry.skeletalPoints[k];
        }
    }

    plannerParameter.sequentialPlanner.costMap.modBreakpoints                              = parameter->data.sequentialPlanner.costMap.modBreakpoints;
    plannerParameter.sequentialPlanner.costMap.resolution                                  = parameter->data.sequentialPlanner.costMap.resolution;
    plannerParameter.sequentialPlanner.costMap.distanceScale                               = parameter->data.sequentialPlanner.costMap.distanceScale;
    plannerParameter.sequentialPlanner.costMap.distanceDecay                               = parameter->data.sequentialPlanner.costMap.distanceDecay;
    plannerParameter.sequentialPlanner.metric.weightPsi                                    = parameter->data.sequentialPlanner.metric.weightPsi;
    plannerParameter.sequentialPlanner.metric.weightSway                                   = parameter->data.sequentialPlanner.metric.weightSway;
    plannerParameter.sequentialPlanner.model.matF                                          = parameter->data.sequentialPlanner.model.matF;
    plannerParameter.sequentialPlanner.model.matB                                          = parameter->data.sequentialPlanner.model.matB;
    plannerParameter.sequentialPlanner.model.vecTimeconstantsXYN                           = parameter->data.sequentialPlanner.model.vecTimeconstantsXYN;
    plannerParameter.sequentialPlanner.model.vecTimeconstantsInput                         = parameter->data.sequentialPlanner.model.vecTimeconstantsInput;
    plannerParameter.sequentialPlanner.model.satXYN                                        = parameter->data.sequentialPlanner.model.satXYN;
    plannerParameter.sequentialPlanner.pathPlanner.periodGoalSampling                      = parameter->data.sequentialPlanner.pathPlanner.periodGoalSampling;
    plannerParameter.sequentialPlanner.motionPlanner.samplingRangePosition                 = parameter->data.sequentialPlanner.motionPlanner.samplingRangePosition;
    plannerParameter.sequentialPlanner.motionPlanner.samplingRangeAngle                    = parameter->data.sequentialPlanner.motionPlanner.samplingRangeAngle;
    plannerParameter.sequentialPlanner.motionPlanner.periodGoalSampling                    = parameter->data.sequentialPlanner.motionPlanner.periodGoalSampling;
    plannerParameter.sequentialPlanner.motionPlanner.sampletime                            = parameter->data.sequentialPlanner.motionPlanner.sampletime;
    plannerParameter.sequentialPlanner.motionPlanner.maxPositionOvershoot                  = parameter->data.sequentialPlanner.motionPlanner.maxPositionOvershoot;
    plannerParameter.sequentialPlanner.motionPlanner.maxInputPathLength                    = parameter->data.sequentialPlanner.motionPlanner.maxInputPathLength;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusX                 = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusX;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusY                 = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusY;
    plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusPsi               = parameter->data.sequentialPlanner.motionPlanner.controller.maxRadiusPsi;
    plannerParameter.sequentialPlanner.motionPlanner.controller.minRadiusPosition          = parameter->data.sequentialPlanner.motionPlanner.controller.minRadiusPosition;
    plannerParameter.sequentialPlanner.motionPlanner.controller.vecTimeconstantsFlatStates = parameter->data.sequentialPlanner.motionPlanner.controller.vecTimeconstantsFlatStates;
    plannerParameter.sequentialPlanner.motionPlanner.controller.matK                       = parameter->data.sequentialPlanner.motionPlanner.controller.matK;
    plannerParameter.sequentialPlanner.motionPlanner.controller.satUVR                     = parameter->data.sequentialPlanner.motionPlanner.controller.satUVR;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangePose          = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangePose;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR           = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR;
    plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN           = parameter->data.sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN;
    plannerParameter.onlinePlanner.predictInitialStateOnReset                              = parameter->data.onlinePlanner.predictInitialStateOnReset;
    plannerParameter.onlinePlanner.maxComputationTimePathOnReset                           = parameter->data.onlinePlanner.maxComputationTimePathOnReset;
    plannerParameter.onlinePlanner.maxComputationTimeMotionOnReset                         = parameter->data.onlinePlanner.maxComputationTimeMotionOnReset;
    plannerParameter.onlinePlanner.maxComputationTimePath                                  = parameter->data.onlinePlanner.maxComputationTimePath;
    plannerParameter.onlinePlanner.maxComputationTimeMotion                                = parameter->data.onlinePlanner.maxComputationTimeMotion;
    plannerParameter.onlinePlanner.additionalAheadPlanningTime                             = parameter->data.onlinePlanner.additionalAheadPlanningTime;
    plannerParameter.onlinePlanner.additionalTrajectoryDuration                            = parameter->data.onlinePlanner.additionalTrajectoryDuration;
    plannerParameter.onlinePlanner.timeKeepPastTrajectory                                  = parameter->data.onlinePlanner.timeKeepPastTrajectory;

    // if there're errors make whole parameter invalid (remove vehicle shape and skeletal points)
    if(!validVehicleShape || !validSkeletalPoints){
        plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
        plannerParameter.sequentialPlanner.geometry.skeletalPoints.clear();
    }
}

void MPSV_WrapperAsynchronousOnlinePlanner::WriteToOutput(SerializationAsynchronousOnlinePlannerOutputUnion* output){
    output->data.timestamp                                 = plannerOutput.timestamp;
    output->data.timestampInput                            = plannerOutput.timestampInput;
    output->data.timestampParameter                        = plannerOutput.timestampParameter;
    output->data.threadState                               = static_cast<uint8_t>(plannerOutput.threadState);
    output->data.timeoutInput                              = static_cast<uint8_t>(plannerOutput.timeoutInput);
    output->data.validInput                                = static_cast<uint8_t>(plannerOutput.validInput);
    output->data.validParameter                            = static_cast<uint8_t>(plannerOutput.validParameter);
    output->data.performedReset                            = static_cast<uint8_t>(plannerOutput.performedReset);
    output->data.error                                     = static_cast<uint8_t>(plannerOutput.error);
    output->data.originLLA                                 = plannerOutput.originLLA;

    // trajectory
    output->data.numTrajectoryPoints                       = static_cast<uint16_t>(std::min(plannerOutput.trajectory.size(), output->data.trajectory.size()));
    output->data.trajectoryShrinked                        = static_cast<uint8_t>(plannerOutput.trajectory.size() > output->data.trajectory.size());
    output->data.sampletime                                = plannerOutput.sampletime;
    for(uint16_t k = 0; k < output->data.numTrajectoryPoints; ++k){
        output->data.trajectory[k] = plannerOutput.trajectory[k];
    }

    // pathPlanner.path
    output->data.pathPlanner.numPoses                      = static_cast<uint16_t>(std::min(plannerOutput.pathPlanner.path.size(), output->data.pathPlanner.path.size()));
    output->data.pathPlanner.pathShrinked                  = static_cast<uint8_t>(plannerOutput.pathPlanner.path.size() > output->data.pathPlanner.path.size());
    for(uint16_t k = 0; k < output->data.pathPlanner.numPoses; ++k){
        output->data.pathPlanner.path[k] = plannerOutput.pathPlanner.path[k];
    }
    output->data.pathPlanner.goalReached                   = static_cast<uint8_t>(plannerOutput.pathPlanner.goalReached);
    output->data.pathPlanner.isFeasible                    = static_cast<uint8_t>(plannerOutput.pathPlanner.isFeasible);
    output->data.pathPlanner.outOfNodes                    = static_cast<uint8_t>(plannerOutput.pathPlanner.outOfNodes);
    output->data.pathPlanner.numberOfPerformedIterations   = plannerOutput.pathPlanner.numberOfPerformedIterations;
    output->data.pathPlanner.timestampOfComputationUTC     = plannerOutput.pathPlanner.timestampOfComputationUTC;
    output->data.pathPlanner.cost                          = plannerOutput.pathPlanner.cost;

    // motionPlanner.referencePath
    output->data.motionPlanner.numPoses                    = static_cast<uint16_t>(std::min(plannerOutput.motionPlanner.referencePath.size(), output->data.motionPlanner.referencePath.size()));
    output->data.motionPlanner.referencePathShrinked       = static_cast<uint8_t>(plannerOutput.motionPlanner.referencePath.size() > output->data.motionPlanner.referencePath.size());
    for(uint16_t k = 0; k < output->data.motionPlanner.numPoses; ++k){
        output->data.motionPlanner.referencePath[k] = plannerOutput.motionPlanner.referencePath[k];
    }
    output->data.motionPlanner.goalReached                 = static_cast<uint8_t>(plannerOutput.motionPlanner.goalReached);
    output->data.motionPlanner.isFeasible                  = static_cast<uint8_t>(plannerOutput.motionPlanner.isFeasible);
    output->data.motionPlanner.outOfNodes                  = static_cast<uint8_t>(plannerOutput.motionPlanner.outOfNodes);
    output->data.motionPlanner.numberOfPerformedIterations = plannerOutput.motionPlanner.numberOfPerformedIterations;
    output->data.motionPlanner.timestampOfComputationUTC   = plannerOutput.motionPlanner.timestampOfComputationUTC;
    output->data.motionPlanner.cost                        = plannerOutput.motionPlanner.cost;
}

