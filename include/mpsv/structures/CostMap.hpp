#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/structures/LookUpTable2DScalar.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>


namespace mpsv::structures {


/**
 * @brief This class represents the argument for building the costmap.
 */
class CostMapArgument {
    public:
        double weightScale;                                                   // Scale factor (>= 0) of distance cost scale*exp(-decay*d^2).
        double weightDecay;                                                   // Decay factor (> 0) of distance cost scale*exp(-decay*d^2).
        const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles;   // Reference to the container of static obstacles.

        /**
         * @brief Construct a new argument object for the cost map.
         * @param[in] weightScale Scale factor (>= 0) of distance cost scale*exp(-decay*d^2).
         * @param[in] weightDecay Decay factor (> 0) of distance cost scale*exp(-decay*d^2).
         * @param[in] staticObstacles Reference to the container of static obstacles.
         */
        CostMapArgument(double weightScale, double weightDecay, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept : weightScale(weightScale), weightDecay(weightDecay), staticObstacles(staticObstacles) {}
};


/**
 * @brief This class represents a 2D costmap. It is a 2D look-up table that contains precalculated cost values for each cell.
 * The cost function for a 2D position is given by
 * 
 *     c(x,y) = scale * exp(-decay * d(x,y)^2)
 * 
 * where (scale >= 0) and (decay > 0) are tuning parameters and d(x,y)^2 denotes the minimum squared distance to the closest
 * edge of all polygons. The cost value inside polygons is not of importance as those configurations would collide anyway.
 */
class CostMap: public mpsv::structures::LookUpTable2DScalar<mpsv::structures::CostMapArgument> {
    public:
        /**
         * @brief Calculate the cost along a given line between two poses. Two additional points are added along the longitudinal axis to consider the angle of the pose.
         * @tparam Tpose0 This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
         * @tparam Tpose1 This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
         * @param[in] pose0 First pose from which the numerical integration starts.
         * @param[in] pose1 Last pose where the numerical interation ends.
         * @param[in] skeletalPoints Skeletal points (b-frame) at which the cost map is to be evaluated.
         * @return Resulting cost value. This is a numerical integration through the internal LUT along a line between two points.
         */
        template <class Tpose0, class Tpose1> [[nodiscard]] double CostAlongLine(const Tpose0& pose0, const Tpose1& pose1, const std::vector<std::array<double, 2U>>& skeletalPoints) const noexcept {
            static_assert(mpsv::is_vec2<Tpose0>::value, "Argument {pose0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<Tpose1>::value, "Argument {pose1} must be of type std::array<double,N> with N >= 2!");
            std::array<double, 2U> p0, p1;
            double c0{std::cos(pose0[2U])};
            double s0{std::sin(pose0[2U])};
            double c1{std::cos(pose1[2U])};
            double s1{std::sin(pose1[2U])};
            double cost{0.0};
            for (auto&& point : skeletalPoints) {
                p0[0U] = pose0[0U] + c0*point[0U] - s0*point[1U];
                p0[1U] = pose0[1U] + s0*point[0U] + c0*point[1U];
                p1[0U] = pose1[0U] + c1*point[0U] - s1*point[1U];
                p1[1U] = pose1[1U] + s1*point[0U] + c1*point[1U];
                cost += LineIntegral(p0, p1);
            }
            return cost / static_cast<double>(std::max(std::size_t{1U}, skeletalPoints.size()));
        }

    protected:
        /**
         * @brief Callback function for evaluating the look-up table data for a specific 2D position.
         * @param[in] x X position for which to evaluate the table data.
         * @param[in] y Y position for which to evaluate the table data.
         * @param[in] args User-defined arguments to be used for evaluating the table data.
         * @return Final table data value to be stored in the LUT.
         * @note This callback may be invoked concurrently from multiple OpenMP threads. Overrides must be thread-safe.
         */
        double CallbackTableData(double x, double y, const CostMapArgument& args) noexcept {
            double result{0.0};
            double squaredDistance{std::numeric_limits<double>::infinity()};
            bool vertexInsidePolygon{false};
            std::array<double, 2U> position({x, y});
            for (auto&& obstacle : args.staticObstacles) {
                squaredDistance = std::min(squaredDistance, obstacle.MinimumSquaredDistanceToEdges(x, y));
                vertexInsidePolygon |= obstacle.IsInside(position);
            }
            if (std::isfinite(squaredDistance)) {
                result = args.weightScale;
                if (!vertexInsidePolygon) {
                    result *= std::exp(-args.weightDecay * squaredDistance);
                }
            }
            return result;
        }
};


} /* namespace: mpsv::structures */
