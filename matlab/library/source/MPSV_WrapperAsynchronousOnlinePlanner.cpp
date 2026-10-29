#include <MPSV_WrapperAsynchronousOnlinePlanner.hpp>
#include <algorithm>
#include <cmath>
#include <vector>


MPSV_WrapperAsynchronousOnlinePlanner::MPSV_WrapperAsynchronousOnlinePlanner(){}

MPSV_WrapperAsynchronousOnlinePlanner::~MPSV_WrapperAsynchronousOnlinePlanner(){
    Stop();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Start(int16_t pathMaxNumNodes, uint32_t pathMaxNumSamples, int16_t motionMaxNumNodes, uint32_t motionMaxNumSamples, uint32_t maxNumCells, int32_t threadPriority, int32_t ompNumThreads, int32_t ompDynamic, std::vector<int32_t> cpuCoreIDs){
    (void) m_planner.Start(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, maxNumCells, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
}

void MPSV_WrapperAsynchronousOnlinePlanner::Stop(void){
    m_planner.Stop();
}

void MPSV_WrapperAsynchronousOnlinePlanner::Step(serialization_asynchronous_online_planner_output* output, serialization_asynchronous_online_planner_input* input, serialization_asynchronous_online_planner_parameter* parameter){
    DeserializeParameter(parameter);
    m_planner.SetParameter(m_plannerParameter);

    DeserializeInput(input);
    m_planner.SetInput(m_plannerInput);

    m_plannerOutput = m_planner.GetOutput();
    SerializeOutput(output);
}

void MPSV_WrapperAsynchronousOnlinePlanner::DeserializeParameter(serialization_asynchronous_online_planner_parameter* parameter){
    m_plannerParameter.timestamp                                                     = parameter->timestamp;
    m_plannerParameter.timeoutInput                                                  = parameter->timeoutInput;
    m_plannerParameter.sequentialPlanner.geometry.collisionCheckMaxPositionDeviation = parameter->sequentialPlanner.geometry.collisionCheckMaxPositionDeviation;
    m_plannerParameter.sequentialPlanner.geometry.collisionCheckMaxAngleDeviation    = parameter->sequentialPlanner.geometry.collisionCheckMaxAngleDeviation;

    // vehicle shape
    m_plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
    int32_t numVertices = static_cast<int32_t>(parameter->sequentialPlanner.geometry.verticesVehicleShape.size());
    int32_t firstVertex = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    int32_t numPolygonsAdded = 0;
    for(int32_t i = 0; i < numVertices; ++i){
        bool finiteVertex = std::isfinite(parameter->sequentialPlanner.geometry.verticesVehicleShape[i][0]) && std::isfinite(parameter->sequentialPlanner.geometry.verticesVehicleShape[i][1]);
        if((firstVertex < 0) && finiteVertex){
            firstVertex = i;
        }
        else if((firstVertex >= 0) && (!finiteVertex || (i == (numVertices - 1)))){
            i += static_cast<int32_t>((i == (numVertices - 1)) && finiteVertex);
            int32_t polygonSize = i - firstVertex;
            if(polygonSize < 3){
                m_plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
                break;
            }
            std::vector<std::array<double,2>> vertices(polygonSize);
            std::transform(parameter->sequentialPlanner.geometry.verticesVehicleShape.begin() + firstVertex, parameter->sequentialPlanner.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float,2>& vertex){
                return std::array<double,2>{static_cast<double>(vertex[0]), static_cast<double>(vertex[1])};
            });
            m_plannerParameter.sequentialPlanner.geometry.vehicleShape.Add(vertices);
            ++numPolygonsAdded;
            firstVertex = -1;
        }
    }
    if(!numPolygonsAdded){
        m_plannerParameter.sequentialPlanner.geometry.vehicleShape.Clear();
    }

    // skeletal points
    m_plannerParameter.sequentialPlanner.geometry.skeletalPoints.clear();
    for(size_t k = 0; k < parameter->sequentialPlanner.geometry.skeletalPoints.size(); ++k){
        const float x = parameter->sequentialPlanner.geometry.skeletalPoints[k][0];
        const float y = parameter->sequentialPlanner.geometry.skeletalPoints[k][1];
        if(std::isfinite(x) && std::isfinite(y)){
            m_plannerParameter.sequentialPlanner.geometry.skeletalPoints.push_back({static_cast<double>(x), static_cast<double>(y)});
        }
    }

    m_plannerParameter.sequentialPlanner.costMap.modBreakpoints                     = parameter->sequentialPlanner.costMap.modBreakpoints;
    m_plannerParameter.sequentialPlanner.costMap.resolution                         = parameter->sequentialPlanner.costMap.resolution;
    m_plannerParameter.sequentialPlanner.costMap.distanceScale                      = parameter->sequentialPlanner.costMap.distanceScale;
    m_plannerParameter.sequentialPlanner.costMap.distanceDecay                      = parameter->sequentialPlanner.costMap.distanceDecay;
    m_plannerParameter.sequentialPlanner.metric.weightPsi                           = parameter->sequentialPlanner.metric.weightPsi;
    m_plannerParameter.sequentialPlanner.metric.weightSway                          = parameter->sequentialPlanner.metric.weightSway;
    m_plannerParameter.sequentialPlanner.metric.weightReverseScale                  = parameter->sequentialPlanner.metric.weightReverseScale;
    m_plannerParameter.sequentialPlanner.metric.weightReverseDecay                  = parameter->sequentialPlanner.metric.weightReverseDecay;
    m_plannerParameter.sequentialPlanner.model.matF                                 = parameter->sequentialPlanner.model.matF;
    m_plannerParameter.sequentialPlanner.model.matB                                 = parameter->sequentialPlanner.model.matB;
    m_plannerParameter.sequentialPlanner.model.vecTimeconstantsXYN                  = parameter->sequentialPlanner.model.vecTimeconstantsXYN;
    m_plannerParameter.sequentialPlanner.model.vecTimeconstantsInput                = parameter->sequentialPlanner.model.vecTimeconstantsInput;
    m_plannerParameter.sequentialPlanner.model.lowerLimitXYN                        = parameter->sequentialPlanner.model.lowerLimitXYN;
    m_plannerParameter.sequentialPlanner.model.upperLimitXYN                        = parameter->sequentialPlanner.model.upperLimitXYN;
    m_plannerParameter.sequentialPlanner.pathPlanner.periodGoalSampling             = parameter->sequentialPlanner.pathPlanner.periodGoalSampling;
    m_plannerParameter.sequentialPlanner.motionPlanner.samplingRangePosition        = parameter->sequentialPlanner.motionPlanner.samplingRangePosition;
    m_plannerParameter.sequentialPlanner.motionPlanner.samplingRangeAngle           = parameter->sequentialPlanner.motionPlanner.samplingRangeAngle;
    m_plannerParameter.sequentialPlanner.motionPlanner.periodGoalSampling           = parameter->sequentialPlanner.motionPlanner.periodGoalSampling;
    m_plannerParameter.sequentialPlanner.motionPlanner.sampletime                   = parameter->sequentialPlanner.motionPlanner.sampletime;
    m_plannerParameter.sequentialPlanner.motionPlanner.maxInputPathLength           = parameter->sequentialPlanner.motionPlanner.maxInputPathLength;
    m_plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusX        = parameter->sequentialPlanner.motionPlanner.controller.maxRadiusX;
    m_plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusY        = parameter->sequentialPlanner.motionPlanner.controller.maxRadiusY;
    m_plannerParameter.sequentialPlanner.motionPlanner.controller.maxRadiusPsi      = parameter->sequentialPlanner.motionPlanner.controller.maxRadiusPsi;
    m_plannerParameter.sequentialPlanner.motionPlanner.controller.minRadiusPosition = parameter->sequentialPlanner.motionPlanner.controller.minRadiusPosition;
    m_plannerParameter.sequentialPlanner.motionPlanner.controller.matK              = parameter->sequentialPlanner.motionPlanner.controller.matK;
    m_plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangePose = parameter->sequentialPlanner.motionPlanner.regionOfAttraction.rangePose;
    m_plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR  = parameter->sequentialPlanner.motionPlanner.regionOfAttraction.rangeUVR;
    m_plannerParameter.sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN  = parameter->sequentialPlanner.motionPlanner.regionOfAttraction.rangeXYN;
    m_plannerParameter.onlinePlanner.maxComputationTimePathOnReset                  = parameter->onlinePlanner.maxComputationTimePathOnReset;
    m_plannerParameter.onlinePlanner.maxComputationTimeMotionOnReset                = parameter->onlinePlanner.maxComputationTimeMotionOnReset;
    m_plannerParameter.onlinePlanner.maxComputationTimePath                         = parameter->onlinePlanner.maxComputationTimePath;
    m_plannerParameter.onlinePlanner.maxComputationTimeMotion                       = parameter->onlinePlanner.maxComputationTimeMotion;
    m_plannerParameter.onlinePlanner.additionalAheadPlanningTime                    = parameter->onlinePlanner.additionalAheadPlanningTime;
    m_plannerParameter.onlinePlanner.additionalTrajectoryDuration                   = parameter->onlinePlanner.additionalTrajectoryDuration;
    m_plannerParameter.onlinePlanner.timeKeepPastTrajectory                         = parameter->onlinePlanner.timeKeepPastTrajectory;
}

void MPSV_WrapperAsynchronousOnlinePlanner::DeserializeInput(serialization_asynchronous_online_planner_input* input){
    m_plannerInput.enable                = input->enable;
    m_plannerInput.reset                 = input->reset;
    m_plannerInput.timestamp             = input->timestamp;
    m_plannerInput.originLLA             = input->originLLA;
    m_plannerInput.initialStateAndInput  = input->initialStateAndInput;
    m_plannerInput.finalPose             = input->finalPose;
    m_plannerInput.samplingBoxCenterPose = input->samplingBoxCenterPose;
    m_plannerInput.samplingBoxDimension  = input->samplingBoxDimension;
    m_plannerInput.staticObstacles.clear();
    int32_t numVertices = static_cast<int32_t>(input->verticesStaticObstacles.size());
    int32_t firstVertex = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    for(int32_t i = 0; i < numVertices; ++i){
        bool finiteVertex = std::isfinite(input->verticesStaticObstacles[i][0]) && std::isfinite(input->verticesStaticObstacles[i][1]);
        if((firstVertex < 0) && finiteVertex){
            firstVertex = i;
        }
        else if((firstVertex >= 0) && (!finiteVertex || (i == (numVertices - 1)))){
            i += static_cast<int32_t>((i == (numVertices - 1)) && finiteVertex);
            int32_t polygonSize = i - firstVertex;
            if(polygonSize < 3){
                std::vector<std::array<double,2>> invalidVertices = {{0,0},{0,0},{0,0}};
                m_plannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(invalidVertices));
                break;
            }
            std::vector<std::array<double,2>> vertices(polygonSize);
            std::transform(input->verticesStaticObstacles.begin() + firstVertex, input->verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float,2>& vertex){
                return std::array<double,2>{static_cast<double>(vertex[0]), static_cast<double>(vertex[1])};
            });
            m_plannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            firstVertex = -1;
        }
    }
}

void MPSV_WrapperAsynchronousOnlinePlanner::SerializeOutput(serialization_asynchronous_online_planner_output* output){
    output->timestamp           = m_plannerOutput.timestamp;
    output->timestampInput      = m_plannerOutput.timestampInput;
    output->timestampParameter  = m_plannerOutput.timestampParameter;
    output->inputError          = static_cast<uint8_t>(m_plannerOutput.inputError);
    output->parameterError      = static_cast<uint8_t>(m_plannerOutput.parameterError);
    output->threadState         = static_cast<uint8_t>(m_plannerOutput.threadState);
    output->performedReset      = static_cast<uint8_t>(m_plannerOutput.performedReset);
    output->reserved            = 0;
    output->error               = static_cast<uint8_t>(m_plannerOutput.errorCode != mpsv::error_code::NONE);
    output->originLLA           = m_plannerOutput.originLLA;

    // trajectory
    output->numTrajectoryPoints = static_cast<uint16_t>(std::min(m_plannerOutput.trajectory.size(), output->trajectory.size()));
    output->trajectoryShrinked  = static_cast<uint8_t>(m_plannerOutput.trajectory.size() > output->trajectory.size());
    output->sampletime          = m_plannerOutput.sampletime;
    for(uint16_t k = 0; k < output->numTrajectoryPoints; ++k){
        output->trajectory[k] = m_plannerOutput.trajectory[k];
    }

    // pathPlanner.path
    output->pathPlanner.numPoses                      = static_cast<uint16_t>(std::min(m_plannerOutput.pathPlanner.path.size(), output->pathPlanner.path.size()));
    output->pathPlanner.pathShrinked                  = static_cast<uint8_t>(m_plannerOutput.pathPlanner.path.size() > output->pathPlanner.path.size());
    for(uint16_t k = 0; k < output->pathPlanner.numPoses; ++k){
        output->pathPlanner.path[k] = m_plannerOutput.pathPlanner.path[k];
    }
    output->pathPlanner.goalReached                   = static_cast<uint8_t>(m_plannerOutput.pathPlanner.goalReached);
    output->pathPlanner.isFeasible                    = static_cast<uint8_t>(m_plannerOutput.errorCode != mpsv::error_code::PATHPLANNER_INITIAL_POSE_COLLISION);
    output->pathPlanner.outOfNodes                    = static_cast<uint8_t>(m_plannerOutput.errorCode == mpsv::error_code::PATHPLANNER_OUT_OF_NODES);
    output->pathPlanner.numberOfPerformedIterations   = m_plannerOutput.pathPlanner.numberOfPerformedIterations;
    output->pathPlanner.timestampOfComputationUTC     = m_plannerOutput.pathPlanner.timestampOfComputationUTC;
    output->pathPlanner.cost                          = m_plannerOutput.pathPlanner.cost;

    // motionPlanner.referencePath
    output->motionPlanner.numPoses                    = static_cast<uint16_t>(std::min(m_plannerOutput.motionPlanner.referencePath.size(), output->motionPlanner.referencePath.size()));
    output->motionPlanner.referencePathShrinked       = static_cast<uint8_t>(m_plannerOutput.motionPlanner.referencePath.size() > output->motionPlanner.referencePath.size());
    for(uint16_t k = 0; k < output->motionPlanner.numPoses; ++k){
        output->motionPlanner.referencePath[k] = m_plannerOutput.motionPlanner.referencePath[k];
    }
    output->motionPlanner.goalReached                 = static_cast<uint8_t>(m_plannerOutput.motionPlanner.goalReached);
    output->motionPlanner.isFeasible                  = static_cast<uint8_t>((m_plannerOutput.errorCode != mpsv::error_code::MOTIONPLANNER_INITIAL_POSE_COLLISION) && (m_plannerOutput.errorCode != mpsv::error_code::MOTIONPLANNER_INFEASIBLE_TRAJECTORY));
    output->motionPlanner.outOfNodes                  = static_cast<uint8_t>(m_plannerOutput.errorCode == mpsv::error_code::MOTIONPLANNER_OUT_OF_NODES);
    output->motionPlanner.numberOfPerformedIterations = m_plannerOutput.motionPlanner.numberOfPerformedIterations;
    output->motionPlanner.timestampOfComputationUTC   = m_plannerOutput.motionPlanner.timestampOfComputationUTC;
    output->motionPlanner.cost                        = m_plannerOutput.motionPlanner.cost;
}
