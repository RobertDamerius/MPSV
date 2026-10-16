#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/FixedSizeTreeSE2.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents a tree node for the motion planner.
 */
class MotionPlannerTreeNode: public mpsv::core::FixedSizeTreeSE2Node {
    public:
        using mpsv::core::FixedSizeTreeSE2Node::FixedSizeTreeSE2Node;

        std::vector<std::array<double,3>> pathBuffer;          // Buffer for path generation. Each element is given as {x,y,psi}.
        std::vector<std::array<double,12>> trajectoryBuffer;   // Buffer for trajectory generation. Each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
};


class MotionPlannerTree: public mpsv::core::FixedSizeTreeSE2<mpsv::planner::MotionPlannerTreeNode> {
    public:
        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept override {
            std::vector<double> vecPoses;
            std::vector<double> vecCost;
            std::vector<int16_t> vecParentIndices;
            uint32_t Na = static_cast<uint32_t>(activeIndices.size());
            uint32_t Nn = static_cast<uint32_t>(nodes.size());
            vecPoses.reserve(3 * Nn);
            vecCost.reserve(Nn);
            vecParentIndices.reserve(Nn);
            for(uint32_t n = 0; n != Nn; ++n){
                vecPoses.push_back(nodes[n].pose[0]);
                vecPoses.push_back(nodes[n].pose[1]);
                vecPoses.push_back(nodes[n].pose[2]);
                vecCost.push_back(nodes[n].cost);
                vecParentIndices.push_back(nodes[n].iParent);

                // Trajectory
                uint32_t N = static_cast<uint32_t>(nodes[n].trajectoryBuffer.size());
                std::vector<double> vecTrajectory;
                vecTrajectory.reserve(N * 12);
                for(uint32_t i = 0; i != N; ++i){
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][0]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][1]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][2]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][3]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][4]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][5]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][6]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][7]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][8]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][9]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][10]);
                    vecTrajectory.push_back(nodes[n].trajectoryBuffer[i][11]);
                }
                if(!N){
                    vecTrajectory.clear();
                    vecTrajectory.resize(12, std::numeric_limits<double>::quiet_NaN());
                    N = 1;
                }
                file.WriteField("double", preString + "trajectoryFromRoot{" + std::to_string(n+1) + "}", {12, N}, &vecTrajectory[0], sizeof(double) * vecTrajectory.size());
            }
            if(Nn){
                file.WriteField("double", preString + "pose", {3, Nn}, &vecPoses[0], sizeof(double) * vecPoses.size());
                file.WriteField("int16_t", preString + "iParent", {Nn}, &vecParentIndices[0], sizeof(int16_t) * vecParentIndices.size());
                file.WriteField("double", preString + "cost", {Nn}, &vecCost[0], sizeof(double) * vecCost.size());
                file.WriteField("int16_t", preString + "activeIndices", {Na}, &activeIndices[0], sizeof(int16_t) * activeIndices.size());
            }
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

