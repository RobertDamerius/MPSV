#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_LookUpTable2DScalar.hpp>
#include <mpsv_geometry_StaticObstacle.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the argument for building the costmap for the motion planner.
 */
class MotionPlannerCostMapArgument {
    public:
        double weightScale;                                                   // Scale factor (>= 0) of distance cost scale*exp(-decay*d^2).
        double weightDecay;                                                   // Decay factor (> 0) of distance cost scale*exp(-decay*d^2).
        const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles;   // Reference to the container of static obstacles.

        /**
         * @brief Construct a new argument object for the motion planner cost map.
         * @param[in] weightScale Scale factor (>= 0) of distance cost scale*exp(-decay*d^2).
         * @param[in] weightDecay Decay factor (> 0) of distance cost scale*exp(-decay*d^2).
         * @param[in] staticObstacles Reference to the container of static obstacles.
         */
        MotionPlannerCostMapArgument(double weightScale, double weightDecay, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept : weightScale(weightScale), weightDecay(weightDecay), staticObstacles(staticObstacles) {}
};


/**
 * @brief This class represents a 2D costmap for motion planning. It is a 2D look-up table that contains precalculated cost values for each cell.
 * The cost function for a 2D position is given by
 * 
 *     c(x,y) = scale * exp(-decay * d(x,y)^2)
 * 
 * where (scale >= 0) and (decay > 0) are tuning parameters and d(x,y)^2 denotes the minimum squared distance to the closest
 * edge of all polygons. The cost value inside polygons is not of importance as those configurations would collide anyway.
 */
class MotionPlannerCostMap: public mpsv::core::LookUpTable2DScalar<MotionPlannerCostMapArgument> {
    public:
        /**
         * @brief Calculate the cost along a given line between two poses. Two additional points are added along the longitudinal axis to consider the angle of the pose.
         * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
         * @tparam U This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
         * @param[in] pose0 First pose from which the numerical integration starts.
         * @param[in] pose1 Last pose where the numerical interation ends.
         * @param[in] skeletalPoints Skeletal points (b-frame) at which the cost map is to be evaluated.
         * @return Resulting cost value. This is a numerical integration through the internal LUT along a line between two points.
         */
        template <class T, class U> double CostAlongLine(const T& pose0, const U& pose1, std::vector<std::array<double,2>> skeletalPoints) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {pose0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<U>::value,"Argument {pose1} must be of type std::array<double,N> with N >= 2!");
            std::array<double,2> p0, p1;
            double c0 = std::cos(pose0[2]);
            double s0 = std::sin(pose0[2]);
            double c1 = std::cos(pose1[2]);
            double s1 = std::sin(pose1[2]);
            double cost = 0.0;
            for(auto&& point : skeletalPoints){
                p0[0] = pose0[0] + c0*point[0] - s0*point[1];
                p0[1] = pose0[1] + s0*point[0] + c0*point[1];
                p1[0] = pose1[0] + c1*point[0] - s1*point[1];
                p1[1] = pose1[1] + s1*point[0] + c1*point[1];
                cost += LineIntegral(p0, p1);
            }
            return cost / static_cast<double>(std::max(static_cast<size_t>(1), skeletalPoints.size()));
        }

    protected:
        /**
         * @brief Callback function for evaluating the look-up table data for a specific 2D position.
         * @param[in] x X position for which to evaluate the table data.
         * @param[in] y Y position for which to evaluate the table data.
         * @param[in] args User-defined arguments to be used for evaluating the table data.
         * @return Final table data value to be stored in the LUT.
         */
        double CallbackTableData(double x, double y, const MotionPlannerCostMapArgument& args) noexcept {
            double result = 0.0;
            double squaredDistance = std::numeric_limits<double>::infinity();
            bool vertexInsidePolygon = false;
            std::array<double,2> position({x, y});
            for(auto&& obstacle : args.staticObstacles){
                squaredDistance = std::min(squaredDistance, obstacle.MinimumSquaredDistanceToEdges(x,y));
                vertexInsidePolygon |= obstacle.IsInside(position);
            }
            if(std::isfinite(squaredDistance)){
                result = args.weightScale;
                if(!vertexInsidePolygon){
                    result *= std::exp(-args.weightDecay * squaredDistance);
                }
            }
            return result;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

