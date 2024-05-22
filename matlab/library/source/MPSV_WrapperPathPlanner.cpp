#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_WrapperPathPlanner::Initialize(int16_t maxNumNodes, uint32_t maxNumSamples){
    initializationOK = pathPlanner.Initialize(maxNumNodes, maxNumSamples);
}

void MPSV_WrapperPathPlanner::Terminate(void){
    pathPlanner.Terminate();
    initializationOK = false;
    maxComputationTime = 0.0;
}

void MPSV_WrapperPathPlanner::Step(SerializationPathPlannerOutputUnion* output, SerializationPathPlannerInputUnion* input){
    // Assign input data to path planner data
    bool validInput = AssignInput(input);

    // Set default output values
    ClearOutput(output, validInput);

    // Solve the planning problem
    if(validInput){
        pathPlanner.ApplyParameterSet(pathPlannerParameter);
        pathPlanner.Prepare(pathPlannerOutput, pathPlannerInput);
        pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, input->data.parameter.pathPlanner.maxComputationTime);

        // Assign output data
        AssignOutput(output);
    }
}

bool MPSV_WrapperPathPlanner::AssignInput(SerializationPathPlannerInputUnion* input){
    // Assign parameters
    pathPlannerParameter.costMap.modBreakpoints                      = input->data.parameter.costMap.modBreakpoints;
    pathPlannerParameter.costMap.resolution                          = input->data.parameter.costMap.resolution;
    pathPlannerParameter.costMap.distanceScale                       = input->data.parameter.costMap.distanceScale;
    pathPlannerParameter.costMap.distanceDecay                       = input->data.parameter.costMap.distanceDecay;
    pathPlannerParameter.geometry.collisionCheckMaxPositionDeviation = input->data.parameter.geometry.collisionCheckMaxPositionDeviation;
    pathPlannerParameter.geometry.collisionCheckMaxAngleDeviation    = input->data.parameter.geometry.collisionCheckMaxAngleDeviation;
    bool validSkeletalPoints = (input->data.parameter.geometry.numSkeletalPoints > 0) && (input->data.parameter.geometry.numSkeletalPoints <= input->data.parameter.geometry.skeletalPoints.size());
    if(validSkeletalPoints){
        pathPlannerParameter.geometry.skeletalPoints.resize(input->data.parameter.geometry.numSkeletalPoints);
        for(uint8_t k = 0; (k < input->data.parameter.geometry.numSkeletalPoints); ++k){
            pathPlannerParameter.geometry.skeletalPoints[k] = input->data.parameter.geometry.skeletalPoints[k];
        }
    }
    pathPlannerParameter.geometry.vehicleShape.Clear();
    bool validVehicleShape = (input->data.parameter.geometry.numPolygonsVehicleShape > 0) && (input->data.parameter.geometry.numPolygonsVehicleShape <= input->data.parameter.geometry.numVerticesVehicleShape.size());
    if(validVehicleShape){
        for(uint8_t p = 0; p < input->data.parameter.geometry.numPolygonsVehicleShape; ++p){
            validVehicleShape &= (input->data.parameter.geometry.numVerticesVehicleShape[p] > 2) && (input->data.parameter.geometry.numVerticesVehicleShape[p] <= input->data.parameter.geometry.verticesVehicleShape[p].size());
            if(!validVehicleShape){
                break;
            }
            for(uint8_t v = 0; v < input->data.parameter.geometry.numVerticesVehicleShape[p]; ++v){
                std::vector<std::array<double,2>> vertices(input->data.parameter.geometry.verticesVehicleShape[p].begin(), input->data.parameter.geometry.verticesVehicleShape[p].begin() + input->data.parameter.geometry.numVerticesVehicleShape[p]);
                pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            }
        }
        validVehicleShape &= pathPlannerParameter.geometry.vehicleShape.EnsureCorrectVertexOrder();
    }
    pathPlannerParameter.metric.weightPsi                            = input->data.parameter.metric.weightPsi;
    pathPlannerParameter.metric.weightSway                           = input->data.parameter.metric.weightSway;
    pathPlannerParameter.pathPlanner.periodGoalSampling              = input->data.parameter.pathPlanner.periodGoalSampling;

    // Assign maximum computation time
    maxComputationTime                                               = input->data.parameter.pathPlanner.maxComputationTime;
    bool validMaxComputationTime = std::isfinite(maxComputationTime) && (maxComputationTime >= 0.0);

    // Assign inputs
    pathPlannerInput.initialPose                                     = input->data.initialPose;
    pathPlannerInput.finalPose                                       = input->data.finalPose;
    pathPlannerInput.originOldToNew                                  = input->data.originOldToNew;
    pathPlannerInput.samplingBoxCenterPose                           = input->data.samplingBoxCenterPose;
    pathPlannerInput.samplingBoxDimension                            = input->data.samplingBoxDimension;
    pathPlannerInput.staticObstacles.clear();
    bool validStaticObstacles = (input->data.numStaticObstacles <= input->data.numVerticesPerStaticObstacle.size());
    if(validStaticObstacles){
        for(size_t p = 0; p < static_cast<size_t>(input->data.numStaticObstacles); ++p){
            validStaticObstacles &= (input->data.numVerticesPerStaticObstacle[p] > 2) && (input->data.numVerticesPerStaticObstacle[p] <= input->data.verticesStaticObstacles[p].size());
            if(!validStaticObstacles){
                break;
            }
            std::vector<std::array<double,2>> vertices(input->data.verticesStaticObstacles[p].begin(), input->data.verticesStaticObstacles[p].begin() + input->data.numVerticesPerStaticObstacle[p]);
            pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            validStaticObstacles &= pathPlannerInput.staticObstacles.back().EnsureCorrectVertexOrder();
        }
    }

    // Check for data validity
    return initializationOK && validSkeletalPoints && validVehicleShape && validMaxComputationTime && validStaticObstacles && pathPlannerParameter.IsValid() && pathPlannerInput.IsValid();
}

void MPSV_WrapperPathPlanner::AssignOutput(SerializationPathPlannerOutputUnion* output){
    // Limit the number of poses to at most 1000
    if(pathPlannerOutput.path.size() > 1000){
        pathPlannerOutput.path.resize(1000);
    }

    // Copy planning results to the output
    output->data.cost                        = pathPlannerOutput.cost;
    output->data.goalReached                 = pathPlannerOutput.goalReached;
    output->data.isFeasible                  = pathPlannerOutput.isFeasible;
    output->data.outOfNodes                  = pathPlannerOutput.outOfNodes;
    output->data.numberOfPerformedIterations = pathPlannerOutput.numberOfPerformedIterations;
    output->data.timestampOfComputationUTC   = pathPlannerOutput.timestampOfComputationUTC;
    output->data.numPoses                    = static_cast<uint16_t>(pathPlannerOutput.path.size());
    for(size_t k = 0; k < pathPlannerOutput.path.size(); ++k){
        output->data.path[k] = pathPlannerOutput.path[k];
    }
}

void MPSV_WrapperPathPlanner::ClearOutput(SerializationPathPlannerOutputUnion* output, bool validInput){
    output->data.cost = 0.0;
    output->data.goalReached = 0;
    output->data.isFeasible = 0;
    output->data.outOfNodes = 0;
    output->data.invalidInput = static_cast<uint8_t>(!validInput);
    output->data.numberOfPerformedIterations = 0;
    output->data.numPoses = 0;
    output->data.timestampOfComputationUTC = mpsv::core::GetTimestampUTC();
}

