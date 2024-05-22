#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_geometry_ConvexPolygon.hpp>


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
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        explicit StaticObstacle(std::vector<std::array<double, 2>> vertices) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices){}
};


} /* namespace: geometry */


} /* namespace: mpsv */

