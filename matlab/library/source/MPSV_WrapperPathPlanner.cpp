#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_WrapperPathPlanner::Initialize(int16_t maxNumNodes, uint32_t maxNumSamples){
    m_pathPlanner.Initialize(maxNumNodes, maxNumSamples);
}

void MPSV_WrapperPathPlanner::Terminate(void){
    m_pathPlanner.Terminate();
    m_maxComputationTime = 0.0;
}

void MPSV_WrapperPathPlanner::Step(serialization_path_planner_output* output, serialization_path_planner_input* input){
    // Assign input data to path planner data
    mpsv::error_code errorCode = AssignInput(input);

    // Set default output values
    ClearOutput(output, errorCode);

    // Solve the planning problem
    if(mpsv::error_code::NONE == errorCode){
        m_pathPlanner.Solve(m_pathPlannerOutput, m_pathPlannerInput, m_pathPlannerParameter, input->parameter.pathPlanner.maxComputationTime);
        errorCode = m_pathPlannerOutput.errorCode;

        // Assign output data
        AssignOutput(output, errorCode);
    }
}

mpsv::error_code MPSV_WrapperPathPlanner::AssignInput(serialization_path_planner_input* input){
    // Assign parameters
    m_pathPlannerParameter.costMap.modBreakpoints                      = input->parameter.costMap.modBreakpoints;
    m_pathPlannerParameter.costMap.resolution                          = input->parameter.costMap.resolution;
    m_pathPlannerParameter.costMap.distanceScale                       = input->parameter.costMap.distanceScale;
    m_pathPlannerParameter.costMap.distanceDecay                       = input->parameter.costMap.distanceDecay;
    m_pathPlannerParameter.geometry.collisionCheckMaxPositionDeviation = input->parameter.geometry.collisionCheckMaxPositionDeviation;
    m_pathPlannerParameter.geometry.collisionCheckMaxAngleDeviation    = input->parameter.geometry.collisionCheckMaxAngleDeviation;
    m_pathPlannerParameter.geometry.skeletalPoints.clear();
    for(size_t k = 0; k < input->parameter.geometry.skeletalPoints.size(); ++k){
        if(std::isfinite(input->parameter.geometry.skeletalPoints[k][0]) && std::isfinite(input->parameter.geometry.skeletalPoints[k][1])){
            m_pathPlannerParameter.geometry.skeletalPoints.push_back({static_cast<double>(input->parameter.geometry.skeletalPoints[k][0]), static_cast<double>(input->parameter.geometry.skeletalPoints[k][1])});
        }
    }
    m_pathPlannerParameter.geometry.vehicleShape.Clear();
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
                m_pathPlannerParameter.geometry.vehicleShape.Clear();
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->parameter.geometry.verticesVehicleShape.begin() + i0, input->parameter.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            m_pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            numPolygonsAdded++;
            i0 = -1;
        }
    }
    if(!numPolygonsAdded){
        m_pathPlannerParameter.geometry.vehicleShape.Clear();
    }
    m_pathPlannerParameter.metric.weightPsi                            = input->parameter.metric.weightPsi;
    m_pathPlannerParameter.metric.weightSway                           = input->parameter.metric.weightSway;
    m_pathPlannerParameter.metric.weightReverseScale                   = input->parameter.metric.weightReverseScale;
    m_pathPlannerParameter.metric.weightReverseDecay                   = input->parameter.metric.weightReverseDecay;
    m_pathPlannerParameter.pathPlanner.periodGoalSampling              = input->parameter.pathPlanner.periodGoalSampling;

    // Assign maximum computation time
    m_maxComputationTime                                               = input->parameter.pathPlanner.maxComputationTime;
    bool validMaxComputationTime = std::isfinite(m_maxComputationTime) && (m_maxComputationTime >= 0.0);

    // Assign inputs
    m_pathPlannerInput.initialPose                                     = input->initialPose;
    m_pathPlannerInput.finalPose                                       = input->finalPose;
    m_pathPlannerInput.originOldToNew                                  = input->originOldToNew;
    m_pathPlannerInput.samplingBoxCenterPose                           = input->samplingBoxCenterPose;
    m_pathPlannerInput.samplingBoxDimension                            = input->samplingBoxDimension;
    m_pathPlannerInput.staticObstacles.clear();
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
                m_pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(invalid_vertices));
                break;
            }
            std::vector<std::array<double,2>> vertices(numVertices);
            std::transform(input->verticesStaticObstacles.begin() + i0, input->verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float,2>& f){ return std::array<double,2>({static_cast<double>(f[0]), static_cast<double>(f[1])}); });
            m_pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            i0 = -1;
        }
    }

    // Check for data validity
    mpsv::error_code e;
    if(mpsv::error_code::NONE != (e = m_pathPlannerParameter.IsValid()))
        return e;
    if(mpsv::error_code::NONE != (e = m_pathPlannerInput.IsValid()))
        return e;
    if(!validMaxComputationTime)
        return mpsv::error_code::NOT_AVAILABLE;
    return mpsv::error_code::NONE;
}

void MPSV_WrapperPathPlanner::AssignOutput(serialization_path_planner_output* output, mpsv::error_code errorCode){
    // Limit the number of poses to at most 1000
    if(m_pathPlannerOutput.path.size() > 1000){
        m_pathPlannerOutput.path.resize(1000);
    }

    // Copy planning results to the output
    output->errorCode                   = static_cast<uint8_t>(errorCode);
    output->cost                        = m_pathPlannerOutput.cost;
    output->goalReached                 = m_pathPlannerOutput.goalReached;
    output->isFeasible                  = (m_pathPlannerOutput.errorCode == mpsv::error_code::PATHPLANNER_INITIAL_POSE_COLLISION ? 0 : 1);
    output->outOfNodes                  = (m_pathPlannerOutput.errorCode == mpsv::error_code::PATHPLANNER_OUT_OF_NODES ? 1 : 0);
    output->numberOfPerformedIterations = m_pathPlannerOutput.numberOfPerformedIterations;
    output->timestampOfComputationUTC   = m_pathPlannerOutput.timestampOfComputationUTC;
    output->numPoses                    = static_cast<uint16_t>(m_pathPlannerOutput.path.size());
    for(size_t k = 0; k < m_pathPlannerOutput.path.size(); ++k){
        output->path[k] = m_pathPlannerOutput.path[k];
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
    output->timestampOfComputationUTC = mpsv::runtime::GetTimestampUTC();
}
