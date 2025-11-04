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
    mpsv::error_code errorCode = AssignInput(input);

    // Set default output values
    ClearOutput(output, errorCode);

    // Solve the planning problem
    if(mpsv::error_code::NONE == errorCode){
        pathPlanner.ApplyParameterSet(pathPlannerParameter);
        pathPlanner.Prepare(pathPlannerOutput, pathPlannerInput);
        pathPlanner.Solve(pathPlannerOutput, pathPlannerInput, input->parameter.pathPlanner.maxComputationTime);

        // Assign output data
        AssignOutput(output, errorCode);
    }
}

mpsv::error_code MPSV_WrapperPathPlanner::AssignInput(serialization_path_planner_input* input){
    // Assign parameters
    pathPlannerParameter.costMap.modBreakpoints                      = input->parameter.costMap.modBreakpoints;
    pathPlannerParameter.costMap.resolution                          = input->parameter.costMap.resolution;
    pathPlannerParameter.costMap.distanceScale                       = input->parameter.costMap.distanceScale;
    pathPlannerParameter.costMap.distanceDecay                       = input->parameter.costMap.distanceDecay;
    pathPlannerParameter.geometry.collisionCheckMaxPositionDeviation = input->parameter.geometry.collisionCheckMaxPositionDeviation;
    pathPlannerParameter.geometry.collisionCheckMaxAngleDeviation    = input->parameter.geometry.collisionCheckMaxAngleDeviation;
    pathPlannerParameter.geometry.skeletalPoints.clear();
    for(size_t k = 0; k < input->parameter.geometry.skeletalPoints.size(); ++k){
        if(std::isfinite(input->parameter.geometry.skeletalPoints[k][0]) && std::isfinite(input->parameter.geometry.skeletalPoints[k][1])){
            pathPlannerParameter.geometry.skeletalPoints.push_back({static_cast<double>(input->parameter.geometry.skeletalPoints[k][0]), static_cast<double>(input->parameter.geometry.skeletalPoints[k][1])});
        }
    }
    pathPlannerParameter.geometry.vehicleShape.Clear();
    int32_t N = static_cast<int32_t>(input->parameter.geometry.verticesVehicleShape.size());
    int32_t i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    int32_t numPolygonsAdded = 0;
    for(int32_t i = 0; i < N; ++i){
        bool finiteVertex = std::isfinite(input->parameter.geometry.verticesVehicleShape[i][0]) && std::isfinite(input->parameter.geometry.verticesVehicleShape[i][1]);
        if((i0 < 0) && finiteVertex){
            i0 = i;
        }
        else if((i0 >= 0) && (!finiteVertex || (i == (N - 1)))){
            i += static_cast<int32_t>((i == (N - 1)) && finiteVertex);
            int32_t numVertices = i - i0;
            if(numVertices < 3){
                pathPlannerParameter.geometry.vehicleShape.Clear();
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->parameter.geometry.verticesVehicleShape.begin() + i0, input->parameter.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            numPolygonsAdded++;
            i0 = -1;
        }
    }
    if(!numPolygonsAdded){
        pathPlannerParameter.geometry.vehicleShape.Clear();
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
    N = static_cast<int32_t>(input->verticesStaticObstacles.size());
    i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    for(int32_t i = 0; i < N; ++i){
        bool finiteVertex = std::isfinite(input->verticesStaticObstacles[i][0]) && std::isfinite(input->verticesStaticObstacles[i][1]);
        if((i0 < 0) && finiteVertex){
            i0 = i;
        }
        else if((i0 >= 0) && (!finiteVertex || (i == (N - 1)))){
            i += static_cast<int32_t>((i == (N - 1)) && finiteVertex);
            int32_t numVertices = i - i0;
            if(numVertices < 3){
                std::vector<std::array<double,2>> invalid_vertices = {{0,0},{0,0},{0,0}};
                pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(invalid_vertices));
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->verticesStaticObstacles.begin() + i0, input->verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            i0 = -1;
        }
    }

    // Check for data validity
    mpsv::error_code e;
    if(mpsv::error_code::NONE != (e = pathPlannerParameter.IsValid()))
        return e;
    if(mpsv::error_code::NONE != (e = pathPlannerInput.IsValid()))
        return e;
    if(!(initializationOK && validMaxComputationTime))
        return mpsv::error_code::NOT_AVAILABLE;
    return mpsv::error_code::NONE;
}

void MPSV_WrapperPathPlanner::AssignOutput(serialization_path_planner_output* output, mpsv::error_code errorCode){
    // Limit the number of poses to at most 1000
    if(pathPlannerOutput.path.size() > 1000){
        pathPlannerOutput.path.resize(1000);
    }

    // Copy planning results to the output
    output->errorCode                   = static_cast<uint8_t>(errorCode);
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

void MPSV_WrapperPathPlanner::ClearOutput(serialization_path_planner_output* output, mpsv::error_code errorCode){
    output->cost = 0.0;
    output->goalReached = 0;
    output->isFeasible = 0;
    output->outOfNodes = 0;
    output->reserved = 0;
    output->errorCode = static_cast<uint8_t>(errorCode);
    output->numberOfPerformedIterations = 0;
    output->numPoses = 0;
    output->timestampOfComputationUTC = mpsv::core::GetTimestampUTC();
}

