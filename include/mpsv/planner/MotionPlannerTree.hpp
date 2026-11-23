#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/structures/FixedSizeTreeSE2.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents a tree node for the motion planner.
 */
class MotionPlannerTreeNode: public mpsv::structures::FixedSizeTreeSE2Node {
    public:
        using mpsv::structures::FixedSizeTreeSE2Node::FixedSizeTreeSE2Node;

        std::vector<std::array<double, 3U>> pathBuffer;          // Buffer for path generation. Each element is given as {x,y,psi}.
        std::vector<std::array<double, 12U>> trajectoryBuffer;   // Buffer for trajectory generation. Each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
};


class MotionPlannerTree: public mpsv::structures::FixedSizeTreeSE2<mpsv::planner::MotionPlannerTreeNode> {
    public:
        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept override {
            std::vector<double> vecPoses;
            std::vector<double> vecCost;
            std::vector<std::int16_t> vecParentIndices;
            std::uint32_t Na{static_cast<std::uint32_t>(m_activeIndices.size())};
            std::uint32_t Nn{static_cast<std::uint32_t>(m_nodes.size())};
            if ((Nn > 0U) && (Na > 0U)) {
                vecPoses.reserve(3U * Nn);
                vecCost.reserve(Nn);
                vecParentIndices.reserve(Nn);
                for (std::uint32_t n{0U}; n != Nn; ++n) {
                    vecPoses.push_back(m_nodes[n].pose[0U]);
                    vecPoses.push_back(m_nodes[n].pose[1U]);
                    vecPoses.push_back(m_nodes[n].pose[2U]);
                    vecCost.push_back(m_nodes[n].cost);
                    vecParentIndices.push_back(m_nodes[n].iParent);

                    // Trajectory
                    std::uint32_t N{static_cast<std::uint32_t>(m_nodes[n].trajectoryBuffer.size())};
                    std::vector<double> vecTrajectory;
                    vecTrajectory.reserve(N * 12U);
                    for (std::uint32_t i{0U}; i != N; ++i) {
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][0U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][1U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][2U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][3U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][4U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][5U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][6U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][7U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][8U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][9U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][10U]);
                        vecTrajectory.push_back(m_nodes[n].trajectoryBuffer[i][11U]);
                    }
                    if (N == 0U) {
                        vecTrajectory.clear();
                        vecTrajectory.resize(12U, std::numeric_limits<double>::quiet_NaN());
                        N = 1U;
                    }
                    file.WriteField("double", preString + "trajectoryFromRoot{" + std::to_string(n + 1U) + "}", {12U, N}, &vecTrajectory[0U], sizeof(double) * vecTrajectory.size());
                }
                file.WriteField("double", preString + "pose", {3U, Nn}, &vecPoses[0U], sizeof(double) * vecPoses.size());
                file.WriteField("int16_t", preString + "iParent", {Nn}, &vecParentIndices[0U], sizeof(std::int16_t) * vecParentIndices.size());
                file.WriteField("double", preString + "cost", {Nn}, &vecCost[0U], sizeof(double) * vecCost.size());
                file.WriteField("int16_t", preString + "activeIndices", {Na}, &m_activeIndices[0U], sizeof(std::int16_t) * m_activeIndices.size());
            }
        }
};


} /* namespace: planner */


} /* namespace: mpsv */
