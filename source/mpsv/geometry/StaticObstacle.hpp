#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief This class represents a static (non-moving) obstacle that is based on a convex polygon.
 */
class StaticObstacle : public mpsv::geometry::ConvexPolygon {
    public:
        /**
         * @brief Default construction for a static obstacle.
         */
        StaticObstacle() noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(){}

        /**
         * @brief Construct a static obstacle based on given vertices.
         * @param[in] vertices Container of vertices.
         */
        explicit StaticObstacle(std::vector<std::array<double, 2>> vertices) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices){}
};


} /* namespace: geometry */


} /* namespace: mpsv */

