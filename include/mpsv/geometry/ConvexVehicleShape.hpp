#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/DouglasPeucker.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief The convex vehicle shape is represented as a single convex polygon.
 */
class ConvexVehicleShape : public mpsv::geometry::ConvexPolygon {
    public:
        /**
         * @brief Default construction for a convex vehicle shape.
         */
        ConvexVehicleShape() noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon() {}

        /**
         * @brief Construct a convex vehicle shape based on given vertices.
         * @param[in] vertices Container of vertices.
         */
        explicit ConvexVehicleShape(std::vector<std::array<double, 2U>> vertices) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices) {}

        /**
         * @brief Check whether the convex vehicle shape collides with static obstacles or not.
         * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose The pose of the vehicle to be used for collision checking.
         * @return True if a collision occurs, false otherwise.
         */
        template <class Tpose> [[nodiscard]] bool CheckCollisionPose(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, Tpose pose) const noexcept {
            static_assert(mpsv::is_vec3<Tpose>::value, "Argument {pose} must be of type std::array<double,N> with N >= 3!");
            ConvexPolygon transformedVehicleShape = CreateTransformedPolygon(pose);
            for (auto&& polygon : staticObstacles) {
                if (transformedVehicleShape.Overlap(polygon)) {
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Check collision on a line between two poses with given static obstacles. The line may be subdivided into several segments depending on the maximum allowed angle deviation.
         * @tparam Tpose0 Template parameter must be of type std::array<double,N> with N >= 3.
         * @tparam Tpose1 Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose0 First pose vector, containing at least {x, y, psi}.
         * @param[in] pose1 Second pose vector, containing at least {x, y, psi}.
         * @param[in] maxAngleDeviation Maximum angle deviation for path subdivision.
         * @return True if a collision occurs, false otherwise.
         * @details The path between two poses @ref pose0 and @ref pose1 is divided into several sections. For each section a convex hull is computed
         * for the first and last pose of the section. The number of sections between two poses depends on @ref maxAngleDeviation. The path is subdivided
         * in such a way, that the angle difference of two poses of a section is never greater than @ref maxAngleDeviation.
         */
        template <class Tpose0, class Tpose1> [[nodiscard]] bool CheckCollisionLine(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, Tpose0 pose0, Tpose1 pose1, double maxAngleDeviation) const noexcept {
            static_assert(mpsv::is_vec3<Tpose0>::value, "Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<Tpose1>::value, "Argument {pose1} must be of type std::array<double,N> with N >= 3!");

            // Check if AABB of path could actually overlap with the AABB of any static obstacle
            if (!FastOverlapCheckAABB(staticObstacles, pose0, pose1)) {
                return false;
            }

            // Get number of segments and adapt deltaPose to segment size
            std::array<double, 3U> deltaPose = mpsv::math::PoseDifference(pose1, pose0);
            std::int32_t numSegments{std::max(std::int32_t{1}, static_cast<std::int32_t>(std::ceil(std::fabs(deltaPose[2U]) / std::fabs(maxAngleDeviation))))};
            double d{1.0 / static_cast<double>(numSegments)};
            deltaPose[0U] *= d;
            deltaPose[1U] *= d;
            deltaPose[2U] *= d;

            // Check collision for all segments
            double c0{std::cos(pose0[2U])};
            double s0{std::sin(pose0[2U])};
            double c1, s1;
            for (std::int32_t i{0}; i < numSegments; ++i) {
                // Start and end pose for this segment
                pose1[0U] = pose0[0U] + deltaPose[0U];
                pose1[1U] = pose0[1U] + deltaPose[1U];
                pose1[2U] = pose0[2U] + deltaPose[2U];
                c1 = std::cos(pose1[2U]);
                s1 = std::sin(pose1[2U]);

                // If there's an AABB overlap check for exact collision of convex hull from two configurations
                if (FastOverlapCheckAABB(staticObstacles, pose0, pose1)) {
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(pose0[0U], pose0[1U], c0, s0, pose1[0U], pose1[1U], c1, s1);
                    for (auto&& polygon : staticObstacles) {
                        if (hullPolygon.Overlap(polygon)) {
                            return true;
                        }
                    }
                }

                // Next segment
                pose0[0U] = pose1[0U];
                pose0[1U] = pose1[1U];
                pose0[2U] = pose1[2U];
                c0 = c1;
                s0 = s1;
            }
            return false;
        }

        /**
         * @brief Check collision along a given curve of poses with given static obstacles. The curve may be subdivided into several segments depending on the maximum allowed position and angle deviation.
         * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] curve The curve of poses {x,y,psi} to be simplified and checked for collision.
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification.
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification.
         * @return True if a collision occurs, false otherwise.
         * @details The curve is simplified using Douglas-Peucher. For each line segment a convex hull based on the vehicle shape is generated for the collision check.
         * As long as the 2D position error is larger than @ref thresholdPosition, the algorithm behaves equal to the default Douglas-Peucker algorithm for 2D line simplification. If the position error of a simplified line
         * segment is smaller than the specified @ref thresholdPosition, then it is checked whether the angle error stays under the specified threshold. If not, the line segment is further divided.
         */
        template <class Tpose> [[nodiscard]] bool CheckCollisionCurve(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const std::vector<Tpose>& curve, double thresholdPosition, double thresholdAngle) const noexcept {
            static_assert(mpsv::is_vec3<Tpose>::value, "Argument {curve} must be of type std::vector<std::array<double,N>> with N >= 3!");
            if (curve.empty()) {
                return false;
            }
            if (1U == curve.size()) {
                return CheckCollisionPose(staticObstacles, curve[0U]);
            }

            // Simplify poses using the douglas-peucker algorithm
            std::vector<std::size_t> indices = mpsv::geometry::DouglasPeuckerSE2(curve, thresholdPosition, thresholdAngle);

            // Check collision for all line segments (first line segment always starts with index 0)
            double c0{std::cos(curve[0U][2U])};
            double s0{std::sin(curve[0U][2U])};
            double c1, s1;
            for (std::size_t n{1U}; n < indices.size(); ++n) {
                // Current segment
                c1 = std::cos(curve[indices[n]][2U]);
                s1 = std::sin(curve[indices[n]][2U]);

                // If there's an AABB overlap check for exact collision of convex hull from two configurations
                if (FastOverlapCheckAABB(staticObstacles, curve[indices[n - 1U]], curve[indices[n]])) {
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(curve[indices[n - 1U]][0U], curve[indices[n - 1U]][1U], c0, s0, curve[indices[n]][0U], curve[indices[n]][1U], c1, s1);
                    for (auto&& polygon : staticObstacles) {
                        if (hullPolygon.Overlap(polygon)) {
                            return true;
                        }
                    }
                }

                // Next segment
                c0 = c1;
                s0 = s1;
            }
            return false;
        }

    protected:
        /* Hide some of the derived base member functions and make them protected. Prevent transformation of the convex vehicle shape vertices. */
        using mpsv::geometry::ConvexPolygon::Transform;

        /**
         * @brief Perform a fast overlap check for a line connection between two given poses by testing the AABB of the line segment with AABBs of the static obstacles.
         * @tparam Tpose0 Template parameter must be of type std::array<double,N> with N >= 3.
         * @tparam Tpose1 Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose0 First pose vector, containing at least {x, y, psi}.
         * @param[in] pose1 Second pose vector, containing at least {x, y, psi}.
         * @return True if any of the obstacles AABBs overlaps with the AABB of the line segment.
         * @details The AABB of the line segment is calculated from an AABB containing the given two poses, extended by the farthest vertex distance of the vehicle shape.
         */
        template <class Tpose0, class Tpose1> [[nodiscard]] bool FastOverlapCheckAABB(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, Tpose0 pose0, Tpose1 pose1) const noexcept {
            static_assert(mpsv::is_vec3<Tpose0>::value, "Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<Tpose1>::value, "Argument {pose1} must be of type std::array<double,N> with N >= 3!");
            mpsv::geometry::AABB aabb(pose0[0U], pose0[1U], pose1[0U], pose1[1U]);
            aabb.Extend(m_farthestVertexDistance);
            for (auto&& obs : staticObstacles) {
                if (obs.OverlapAABB(aabb)) {
                    return true;
                }
            }
            return false;
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */
