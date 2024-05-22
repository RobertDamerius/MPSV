#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_DataLogFile.hpp>
#include <mpsv_core_FixedSizeTreeSE2.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents a tree node for the path planner.
 */
class PathPlannerTreeNode: public mpsv::core::FixedSizeTreeSE2Node {
    public:
        double tmp;   // Undefined temporary value for this node. The actual meaning depends on the implementation that assignes and uses this value. For example, it's used by the warm start procedure to store a temporary cummulative cost.

        /**
         * @brief Construct a new path planner tree node object.
         * @param[in] numChildsReserved The number of childs to be reserved.
         */
        explicit PathPlannerTreeNode(const size_t numChildsReserved = 64) noexcept : mpsv::core::FixedSizeTreeSE2Node(numChildsReserved), tmp(0.0){}

        /**
         * @brief Construct a new path planner tree node object.
         * @param[in] pose The pose to be used as configuration.
         */
        explicit PathPlannerTreeNode(const std::array<double,3> pose) noexcept : mpsv::core::FixedSizeTreeSE2Node(0), tmp(0.0){ this->pose = pose; }
};


/**
 * @brief This class represents the data structure of a tree to be used by the path planner.
 */
class PathPlannerTree: public mpsv::core::FixedSizeTreeSE2<mpsv::planner::PathPlannerTreeNode> {
    public:
        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
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
                vecParentIndices.push_back(nodes[n].idxParent);
            }
            if(Nn){
                file.WriteField("double", preString + "pose", {3, Nn}, &vecPoses[0], sizeof(double) * vecPoses.size());
                file.WriteField("int16_t", preString + "idxParent", {Nn}, &vecParentIndices[0], sizeof(int16_t) * vecParentIndices.size());
                file.WriteField("double", preString + "cost", {Nn}, &vecCost[0], sizeof(double) * vecCost.size());
                file.WriteField("int16_t", preString + "activeIndices", {Na}, &activeIndices[0], sizeof(int16_t) * activeIndices.size());
            }
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

