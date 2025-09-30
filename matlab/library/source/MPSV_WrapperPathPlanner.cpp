#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_WrapperPathPlanner::Initialize(int16_t maxNumNodes, uint32_t maxNumSamples){
    initializationOK = pathPlanner.Initialize(maxNumNodes, maxNumSamples);
}

void MPSV_WrapperPathPlanner::Terminate(void){
    pathPlanner.Terminate();
    initializationOK = false;
    maxComputationTime = 0.0;
}

void MPSV_WrapperPathPlanner::Step(serialization_path_planner_output* output, serialization_path_planner_input* input){
    // Assign input data to path planner data
    bool validInput = AssignInput(input);

    // Set default output values
    ClearOutput(output, validInput);

    // Solve the planning problem
    if(validInput){
        pathPlanner.ApplyParameterSet(pathPlannerParameter);
        pathPlanner.Prepare(pathPlannerOutput, pathPlannerInput);
        pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, input->parameter.pathPlanner.maxComputationTime);

        // Assign output data
        AssignOutput(output);
    }
}

bool MPSV_WrapperPathPlanner::AssignInput(serialization_path_planner_input* input){
    // Assign parameters
    pathPlannerParameter.costMap.modBreakpoints                      = input->parameter.costMap.modBreakpoints;
    pathPlannerParameter.costMap.resolution                          = input->parameter.costMap.resolution;
    pathPlannerParameter.costMap.distanceScale                       = input->parameter.costMap.distanceScale;
    pathPlannerParameter.costMap.distanceDecay                       = input->parameter.costMap.distanceDecay;
    pathPlannerParameter.geometry.collisionCheckMaxPositionDeviation = input->parameter.geometry.collisionCheckMaxPositionDeviation;
    pathPlannerParameter.geometry.collisionCheckMaxAngleDeviation    = input->parameter.geometry.collisionCheckMaxAngleDeviation;
    bool validSkeletalPoints = (input->parameter.geometry.numSkeletalPoints > 0) && (input->parameter.geometry.numSkeletalPoints <= input->parameter.geometry.skeletalPoints.size());
    if(validSkeletalPoints){
        pathPlannerParameter.geometry.skeletalPoints.resize(input->parameter.geometry.numSkeletalPoints);
        for(uint8_t k = 0; (k < input->parameter.geometry.numSkeletalPoints); ++k){
            pathPlannerParameter.geometry.skeletalPoints[k] = input->parameter.geometry.skeletalPoints[k];
        }
    }
    pathPlannerParameter.geometry.vehicleShape.Clear();
    bool validVehicleShape = true;
    int32_t N = static_cast<int32_t>(input->parameter.geometry.verticesVehicleShape.size());
    int32_t i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    int32_t numPolygonsAdded = 0;
    for(int32_t i = 0; (i < N) && validVehicleShape; ++i){
        bool finiteVertex = std::isfinite(input->parameter.geometry.verticesVehicleShape[i][0]) && std::isfinite(input->parameter.geometry.verticesVehicleShape[i][1]);
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
            std::transform(input->parameter.geometry.verticesVehicleShape.begin() + i0, input->parameter.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            numPolygonsAdded++;
            i0 = -1;
        }
    }
    validVehicleShape &= (numPolygonsAdded > 0);
    if(validVehicleShape){
        validVehicleShape &= pathPlannerParameter.geometry.vehicleShape.EnsureCorrectVertexOrder();
    }
    pathPlannerParameter.metric.weightPsi                            = input->parameter.metric.weightPsi;
    pathPlannerParameter.metric.weightSway                           = input->parameter.metric.weightSway;
    pathPlannerParameter.metric.weightReverseScale                   = input->parameter.metric.weightReverseScale;
    pathPlannerParameter.metric.weightReverseDecay                   = input->parameter.metric.weightReverseDecay;
    pathPlannerParameter.pathPlanner.periodGoalSampling              = input->parameter.pathPlanner.periodGoalSampling;

    // Assign maximum computation time
    maxComputationTime                                               = input->parameter.pathPlanner.maxComputationTime;
    bool validMaxComputationTime = std::isfinite(maxComputationTime) && (maxComputationTime >= 0.0);

    // Assign inputs
    pathPlannerInput.initialPose                                     = input->initialPose;
    pathPlannerInput.finalPose                                       = input->finalPose;
    pathPlannerInput.originOldToNew                                  = input->originOldToNew;
    pathPlannerInput.samplingBoxCenterPose                           = input->samplingBoxCenterPose;
    pathPlannerInput.samplingBoxDimension                            = input->samplingBoxDimension;
    pathPlannerInput.staticObstacles.clear();
    bool validStaticObstacles = true;
    N = static_cast<int32_t>(input->verticesStaticObstacles.size());
    i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    for(int32_t i = 0; (i < N) && validStaticObstacles; ++i){
        bool finiteVertex = std::isfinite(input->verticesStaticObstacles[i][0]) && std::isfinite(input->verticesStaticObstacles[i][1]);
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
            std::transform(input->verticesStaticObstacles.begin() + i0, input->verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            validStaticObstacles &= pathPlannerInput.staticObstacles.back().EnsureCorrectVertexOrder();
            i0 = -1;
        }
    }

    // Check for data validity
    return initializationOK && validSkeletalPoints && validVehicleShape && validMaxComputationTime && validStaticObstacles && pathPlannerParameter.IsValid() && pathPlannerInput.IsValid();
}

void MPSV_WrapperPathPlanner::AssignOutput(serialization_path_planner_output* output){
    // Limit the number of poses to at most 1000
    if(pathPlannerOutput.path.size() > 1000){
        pathPlannerOutput.path.resize(1000);
    }

    // Copy planning results to the output
    output->cost                        = pathPlannerOutput.cost;
    output->goalReached                 = pathPlannerOutput.goalReached;
    output->isFeasible                  = pathPlannerOutput.isFeasible;
    output->outOfNodes                  = pathPlannerOutput.outOfNodes;
    output->numberOfPerformedIterations = pathPlannerOutput.numberOfPerformedIterations;
    output->timestampOfComputationUTC   = pathPlannerOutput.timestampOfComputationUTC;
    output->numPoses                    = static_cast<uint16_t>(pathPlannerOutput.path.size());
    for(size_t k = 0; k < pathPlannerOutput.path.size(); ++k){
        output->path[k] = pathPlannerOutput.path[k];
    }
}

void MPSV_WrapperPathPlanner::ClearOutput(serialization_path_planner_output* output, bool validInput){
    output->cost = 0.0;
    output->goalReached = 0;
    output->isFeasible = 0;
    output->outOfNodes = 0;
    output->invalidInput = static_cast<uint8_t>(!validInput);
    output->numberOfPerformedIterations = 0;
    output->numPoses = 0;
    output->timestampOfComputationUTC = mpsv::core::GetTimestampUTC();
}

