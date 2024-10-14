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
    bool validVehicleShape = true;
    int32_t N = static_cast<int32_t>(input->data.parameter.geometry.verticesVehicleShape.size());
    int32_t i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    int32_t numPolygonsAdded = 0;
    for(int32_t i = 0; (i < N) && validVehicleShape; ++i){
        bool finiteVertex = std::isfinite(input->data.parameter.geometry.verticesVehicleShape[i][0]) && std::isfinite(input->data.parameter.geometry.verticesVehicleShape[i][1]);
        if((i0 < 0) && finiteVertex){
            i0 = i;
        }
        else if((i0 >= 0) && (!finiteVertex || (i == (N - 1)))){
            i += static_cast<int32_t>((i == (N - 1)) && finiteVertex);
            int32_t numVertices = i - i0;
            if(numVertices < 3){
                validVehicleShape = false;
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->data.parameter.geometry.verticesVehicleShape.begin() + i0, input->data.parameter.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            numPolygonsAdded++;
            i0 = -1;
        }
    }
    validVehicleShape &= (numPolygonsAdded > 0);
    if(validVehicleShape){
        validVehicleShape &= pathPlannerParameter.geometry.vehicleShape.EnsureCorrectVertexOrder();
    }
    pathPlannerParameter.metric.weightPsi                            = input->data.parameter.metric.weightPsi;
    pathPlannerParameter.metric.weightSway                           = input->data.parameter.metric.weightSway;
    pathPlannerParameter.metric.weightReverseScale                   = input->data.parameter.metric.weightReverseScale;
    pathPlannerParameter.metric.weightReverseDecay                   = input->data.parameter.metric.weightReverseDecay;
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
    bool validStaticObstacles = true;
    N = static_cast<int32_t>(input->data.verticesStaticObstacles.size());
    i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    for(int32_t i = 0; (i < N) && validStaticObstacles; ++i){
        bool finiteVertex = std::isfinite(input->data.verticesStaticObstacles[i][0]) && std::isfinite(input->data.verticesStaticObstacles[i][1]);
        if((i0 < 0) && finiteVertex){
            i0 = i;
        }
        else if((i0 >= 0) && (!finiteVertex || (i == (N - 1)))){
            i += static_cast<int32_t>((i == (N - 1)) && finiteVertex);
            int32_t numVertices = i - i0;
            if(numVertices < 3){
                validStaticObstacles = false;
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->data.verticesStaticObstacles.begin() + i0, input->data.verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            validStaticObstacles &= pathPlannerInput.staticObstacles.back().EnsureCorrectVertexOrder();
            i0 = -1;
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

