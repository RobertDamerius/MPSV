#include <MPSV_WrapperPathPlanner.hpp>


void MPSV_WrapperPathPlanner::Initialize(std::int16_t maxNumNodes, std::uint32_t maxNumSamples, std::uint32_t maxNumCells) {
    m_pathPlanner.Reconfigure(maxNumNodes, maxNumSamples, maxNumCells);
}

void MPSV_WrapperPathPlanner::Terminate() {
}

void MPSV_WrapperPathPlanner::Step(serialization_path_planner_output* output, serialization_path_planner_input* input) {
    DeserializeInput(input);
    double maxComputationTime{std::max(0.0, input->parameter.pathPlanner.maxComputationTime)};
    if (!std::isfinite(maxComputationTime)) {
        maxComputationTime = 0.0;
    }
    m_pathPlanner.Solve(m_pathPlannerOutput, m_pathPlannerInput, m_pathPlannerParameter, maxComputationTime);
    SerializeOutput(output);
}

void MPSV_WrapperPathPlanner::DeserializeInput(serialization_path_planner_input* input) {
    // Assign parameters
    m_pathPlannerParameter.costMap.resolution                          = input->parameter.costMap.resolution;
    m_pathPlannerParameter.costMap.distanceScale                       = input->parameter.costMap.distanceScale;
    m_pathPlannerParameter.costMap.distanceDecay                       = input->parameter.costMap.distanceDecay;
    m_pathPlannerParameter.geometry.collisionCheckMaxPositionDeviation = input->parameter.geometry.collisionCheckMaxPositionDeviation;
    m_pathPlannerParameter.geometry.collisionCheckMaxAngleDeviation    = input->parameter.geometry.collisionCheckMaxAngleDeviation;
    m_pathPlannerParameter.geometry.skeletalPoints.clear();
    for (std::size_t k{0U}; k < input->parameter.geometry.skeletalPoints.size(); ++k) {
        if (std::isfinite(input->parameter.geometry.skeletalPoints[k][0U]) && std::isfinite(input->parameter.geometry.skeletalPoints[k][1U])) {
            m_pathPlannerParameter.geometry.skeletalPoints.push_back({static_cast<double>(input->parameter.geometry.skeletalPoints[k][0U]), static_cast<double>(input->parameter.geometry.skeletalPoints[k][1U])});
        }
    }
    m_pathPlannerParameter.geometry.vehicleShape.Clear();
    std::int32_t N{static_cast<std::int32_t>(input->parameter.geometry.verticesVehicleShape.size())};
    std::int32_t i0{-1}; // index of previous finite vertex (-1 indicates no previous finite vertex)
    std::int32_t numPolygonsAdded{0};
    for (std::int32_t i{0}; i < N; ++i) {
        bool finiteVertex{std::isfinite(input->parameter.geometry.verticesVehicleShape[i][0U]) && std::isfinite(input->parameter.geometry.verticesVehicleShape[i][1U])};
        if ((i0 < 0) && finiteVertex) {
            i0 = i;
        } else if ((i0 >= 0) && (!finiteVertex || (i == (N - 1)))) {
            i += static_cast<std::int32_t>((i == (N - 1)) && finiteVertex);
            std::int32_t numVertices{i - i0};
            if (numVertices < 3) {
                m_pathPlannerParameter.geometry.vehicleShape.Clear();
                break;
            }
            std::vector<std::array<double, 2U>> vertices(numVertices);
            std::transform(input->parameter.geometry.verticesVehicleShape.begin() + i0, input->parameter.geometry.verticesVehicleShape.begin() + i, vertices.begin(), [](const std::array<float, 2U>& f) { return std::array<double, 2U>({static_cast<double>(f[0U]), static_cast<double>(f[1U])}); });
            m_pathPlannerParameter.geometry.vehicleShape.Add(vertices);
            numPolygonsAdded++;
            i0 = -1;
        }
    }
    if (!numPolygonsAdded) {
        m_pathPlannerParameter.geometry.vehicleShape.Clear();
    }
    m_pathPlannerParameter.metric.weightPsi                            = input->parameter.metric.weightPsi;
    m_pathPlannerParameter.metric.weightSway                           = input->parameter.metric.weightSway;
    m_pathPlannerParameter.metric.weightReverseScale                   = input->parameter.metric.weightReverseScale;
    m_pathPlannerParameter.metric.weightReverseDecay                   = input->parameter.metric.weightReverseDecay;
    m_pathPlannerParameter.pathPlanner.periodGoalSampling              = input->parameter.pathPlanner.periodGoalSampling;

    // Assign inputs
    m_pathPlannerInput.initialPose                                     = input->initialPose;
    m_pathPlannerInput.finalPose                                       = input->finalPose;
    m_pathPlannerInput.originOldToNew                                  = input->originOldToNew;
    m_pathPlannerInput.samplingBoxCenterPose                           = input->samplingBoxCenterPose;
    m_pathPlannerInput.samplingBoxDimension                            = input->samplingBoxDimension;
    m_pathPlannerInput.staticObstacles.clear();
    N = static_cast<std::int32_t>(input->verticesStaticObstacles.size());
    i0 = -1; // index of previous finite vertex (-1 indicates no previous finite vertex)
    for (std::int32_t i{0}; i < N; ++i) {
        bool finiteVertex{std::isfinite(input->verticesStaticObstacles[i][0U]) && std::isfinite(input->verticesStaticObstacles[i][1U])};
        if ((i0 < 0) && finiteVertex) {
            i0 = i;
        } else if ((i0 >= 0) && (!finiteVertex || (i == (N - 1)))) {
            i += static_cast<std::int32_t>((i == (N - 1)) && finiteVertex);
            std::int32_t numVertices{i - i0};
            if (numVertices < 3) {
                std::vector<std::array<double, 2U>> invalid_vertices = {{0, 0}, {0, 0}, {0, 0}};
                m_pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(invalid_vertices));
                break;
            }
            std::vector<std::array<double, 2U>> vertices(numVertices);
            std::transform(input->verticesStaticObstacles.begin() + i0, input->verticesStaticObstacles.begin() + i, vertices.begin(), [](const std::array<float, 2U>& f) { return std::array<double, 2U>({static_cast<double>(f[0U]), static_cast<double>(f[1U])}); });
            m_pathPlannerInput.staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
            i0 = -1;
        }
    }
}

void MPSV_WrapperPathPlanner::SerializeOutput(serialization_path_planner_output* output) {
    // Copy planning results to the output
    output->errorCode                   = static_cast<std::uint8_t>(m_pathPlannerOutput.errorCode);
    output->cost                        = m_pathPlannerOutput.cost;
    output->goalReached                 = m_pathPlannerOutput.goalReached;
    output->pathTruncated               = static_cast<std::uint8_t>(m_pathPlannerOutput.path.size() > output->path.size());
    output->reserved                    = 0U;
    output->numberOfPerformedIterations = m_pathPlannerOutput.numberOfPerformedIterations;
    output->timestampOfComputationUTC   = m_pathPlannerOutput.timestampOfComputationUTC;
    output->numPoses                    = static_cast<std::int32_t>(std::min(m_pathPlannerOutput.path.size(), output->path.size()));
    for (std::size_t k{0U}; k < static_cast<std::size_t>(output->numPoses); ++k) {
        output->path[k] = m_pathPlannerOutput.path[k];
    }
}
