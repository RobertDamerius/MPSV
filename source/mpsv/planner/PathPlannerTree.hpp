#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/FixedSizeTreeSE2.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents a tree node for the path planner.
 */
class PathPlannerTreeNode: public mpsv::core::FixedSizeTreeSE2Node {
    public:
        using mpsv::core::FixedSizeTreeSE2Node::FixedSizeTreeSE2Node;
};


/**
 * @brief This class represents the data structure of a tree to be used by the path planner.
 */
class PathPlannerTree: public mpsv::core::FixedSizeTreeSE2<mpsv::planner::PathPlannerTreeNode> { };


} /* namespace: planner */


} /* namespace: mpsv */

