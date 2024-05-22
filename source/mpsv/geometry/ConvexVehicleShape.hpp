#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/MovingObstacle.hpp>
#include <mpsv/geometry/DouglasPeucker.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief The convex vehicle shape represented as a single convex polygon.
 * @note IMPORTANT NOTE: All member functions assume, that the vertex data represents a valid convex polygon. The construction of a convex polygon
 * will not fail due to invalid input data. You MUST make sure that you created a valid convex polygon with AT LEAST 3 vertices. You can check
 * the vertex data for convexity and finiteness using the @ref IsConvex and @ref IsFinite member functions, respectively.
 */
class ConvexVehicleShape : public mpsv::geometry::ConvexPolygon {
    public:
        /**
         * @brief Default construction for a convex vehicle shape.
         */
        ConvexVehicleShape() noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(){}

        /**
         * @brief Construct a convex vehicle shape based on given vertices.
         * @param[in] vertices Container of vertices.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        explicit ConvexVehicleShape(std::vector<std::array<double, 2>> vertices) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices){}

        /**
         * @brief Check whether the convex vehicle shape collides with static obstacles or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose The pose of the vehicle to be used for collision checking.
         * @return True if a collision occurs, false otherwise.
         * @note IMPORTANT: MAKE SURE THAT THE VEHICLE SHAPE IS VALID AND CONTAINS AT LEAST 3 VERTICES.
         */
        template <class T> bool CheckCollisionPose(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            ConvexPolygon transformedVehicleShape = CreateTransformedPolygon(pose[0], pose[1], std::cos(pose[2]), std::sin(pose[2]));
            for(auto&& polygon : staticObstacles){
                if(transformedVehicleShape.Overlap(polygon)){
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Check collision on a line between two poses with given static obstacles. The line may be subdivided into several segments depending on the maximum allowed angle deviation.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose0 First pose vector, containing at least {x, y, psi}.
         * @param[in] pose1 Second pose vector, containing at least {x, y, psi}.
         * @param[in] maxAngleDeviation Maximum angle deviation for path subdivision. MAKE SURE THAT THIS VALUE IS POSITIVE!
         * @return True if a collision occurs, false otherwise.
         * @details The path between two poses @ref pose0 and @ref pose1 is divided into several sections. For each section a convex hull is computed
         * for the first and last pose of the section. The number of sections between two poses depends on @ref maxAngleDeviation. The path is subdivided
         * in such a way, that the angle difference of two poses of a section is never greater than @ref maxAngleDeviation.
         * @note IMPORTANT: MAKE SURE THAT THE VEHICLE SHAPE IS VALID AND CONTAINS AT LEAST 3 VERTICES.
         */
        template <class T, class U> bool CheckCollisionLine(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose0, U pose1, double maxAngleDeviation) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<U>::value,"Argument {pose1} must be of type std::array<double,N> with N >= 3!");

            // Check if AABB of path could actually overlap with the AABB of any static obstacle
            if(!FastOverlapCheckAABB(staticObstacles, pose0, pose1)) return false;

            // Get number of segments and adapt deltaPose to segment size
            std::array<double, 3> deltaPose = mpsv::math::PoseDifference(pose1, pose0);
            int32_t numSegments = std::max(static_cast<int32_t>(1), static_cast<int32_t>(std::ceil(std::fabs(deltaPose[2]) / maxAngleDeviation)));
            double d = 1.0 / static_cast<double>(numSegments);
            deltaPose[0] *= d;
            deltaPose[1] *= d;
            deltaPose[2] *= d;

            // Check collision for all segments
            double c0 = std::cos(pose0[2]);
            double s0 = std::sin(pose0[2]);
            double c1, s1;
            for(int32_t i = 0; i < numSegments; ++i){
                // Start and end pose for this segment
                pose1[0] = pose0[0] + deltaPose[0];
                pose1[1] = pose0[1] + deltaPose[1];
                pose1[2] = pose0[2] + deltaPose[2];
                c1 = std::cos(pose1[2]);
                s1 = std::sin(pose1[2]);

                // If there's an AABB overlap check for exact collision of convex hull from two configurations
                if(FastOverlapCheckAABB(staticObstacles, pose0, pose1)){
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(pose0[0], pose0[1], c0, s0, pose1[0], pose1[1], c1, s1);
                    for(auto&& polygon : staticObstacles){
                        if(hullPolygon.Overlap(polygon)){
                            return true;
                        }
                    }
                }

                // Next segment
                pose0[0] = pose1[0];
                pose0[1] = pose1[1];
                pose0[2] = pose1[2];
                c0 = c1;
                s0 = s1;
            }
            return false;
        }

        /**
         * @brief Check collision of line segments with obstacles without any further subdivision of line segments.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] lineSegments List of poses, where each pair of consecutive poses represents a line segment.
         * @return True if a collision occurs, false otherwise.
         * @details For each line segment a convex hull based on the vehicle shape is generated for the collision check.
         * @note IMPORTANT: MAKE SURE THAT THE VEHICLE SHAPE IS VALID AND CONTAINS AT LEAST 3 VERTICES.
         */
        template <class T> bool CheckCollisionLineSegments(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const std::vector<T>& lineSegments) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {lineSegments} must be of type std::vector<std::array<double,N>> with N >= 3!");
            if(lineSegments.empty()){
                return false;
            }
            if(1 == lineSegments.size()){
                return CheckCollisionPose(staticObstacles, lineSegments[0]);
            }

            // Check collision for all line segments
            double c0 = std::cos(lineSegments[0][2]);
            double s0 = std::sin(lineSegments[0][2]);
            double c1, s1;
            for(size_t n = 1; n < lineSegments.size(); ++n){
                // Current segment
                c1 = std::cos(lineSegments[n][2]);
                s1 = std::sin(lineSegments[n][2]);

                // If there's an AABB overlap check for exact collision of convex hull from two configurations
                if(FastOverlapCheckAABB(staticObstacles, lineSegments[n-1], lineSegments[n])){
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(lineSegments[n-1][0], lineSegments[n-1][1], c0, s0, lineSegments[n][0], lineSegments[n][1], c1, s1);
                    for(auto&& polygon : staticObstacles){
                        if(hullPolygon.Overlap(polygon)){
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

        /**
         * @brief Check collision along a given curve of poses with given static obstacles. The curve may be subdivided into several segments depending on the maximum allowed position and angle deviation.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] curve The curve of poses {x,y,psi} to be simplified and checked for collision.
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
         * @return True if a collision occurs, false otherwise.
         * @details The curve is simplified using Douglas-Peucher. For each line segment a convex hull based on the vehicle shape is generated for the collision check.
         * As long as the 2D position error is larger than @ref thresholdPosition, the algorithm behaves equal to the default Douglas-Peucker algorithm for 2D line simplification. If the position error of a simplified line
         * segment is smaller than the specified @ref thresholdPosition, then it is checked whether the angle error stays under the specified threshold. If not, the line segment is further divided.
         */
        template <class T> bool CheckCollisionCurve(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const std::vector<T>& curve, double thresholdPosition, double thresholdAngle) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {curve} must be of type std::vector<std::array<double,N>> with N >= 3!");
            if(curve.empty()){
                return false;
            }
            if(1 == curve.size()){
                return CheckCollisionPose(staticObstacles, curve[0]);
            }

            // Simplify poses using the douglas-peucker algorithm
            std::vector<size_t> indices = mpsv::geometry::DouglasPeuckerSE2(curve, thresholdPosition, thresholdAngle);

            // Check collision for all line segments (first line segment always starts with index 0)
            double c0 = std::cos(curve[0][2]);
            double s0 = std::sin(curve[0][2]);
            double c1, s1;
            for(size_t n = 1; n < indices.size(); ++n){
                // Current segment
                c1 = std::cos(curve[indices[n]][2]);
                s1 = std::sin(curve[indices[n]][2]);

                // If there's an AABB overlap check for exact collision of convex hull from two configurations
                if(FastOverlapCheckAABB(staticObstacles, curve[indices[n-1]], curve[indices[n]])){
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(curve[indices[n-1]][0], curve[indices[n-1]][1], c0, s0, curve[indices[n]][0], curve[indices[n]][1], c1, s1);
                    for(auto&& polygon : staticObstacles){
                        if(hullPolygon.Overlap(polygon)){
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

        /**
         * @brief Check collision along a given timed curve with moving obstacles. The curve is subdivided into time intervals. Each time interval is checked for collision. The time intervals may be subdivided
         * into several segments depending on the maximum allowed position and angle deviation.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] movingObstacles The moving obstacles to be considered for collision checking. Note that the movingObstacles must be converted to time interval obstacles using the same timeinterval as indicated by numStepsCollisionWindow and sampletime.
         * @param[in] curve The curve of poses {x,y,psi} to be simplified and checked for collision.
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
         * @param[in] numStepsCollisionWindow Number of steps in the curve that indicate a time interval. Note that the movingObstacles must be converted to time interval obstacles using the same timeinterval.
         * @param[in] sampletime The sampletime in seconds, indicating the time step between two points in the curve.
         * @return True if a collision occurs, false otherwise.
         */
        template <class T> bool CheckCollisionTimedCurve(const std::vector<mpsv::geometry::MovingObstacle>& movingObstacles, const std::vector<T>& curve, double thresholdPosition, double thresholdAngle, uint32_t numStepsCollisionWindow, double sampletime) const noexcept {
            // Go through all time intervals
            uint32_t maxNumSteps = static_cast<uint32_t>(curve.size());
            for(uint32_t k = 0; k < maxNumSteps; k += numStepsCollisionWindow){
                // First index of current time interval is k, last index is n
                uint32_t n = std::min(k + numStepsCollisionWindow, maxNumSteps - 1);

                // Corresponding time point (center of time interval)
                double centerOfTimeInterval = sampletime * 0.5 * static_cast<double>(k + n);

                // Go through all moving obstacles
                for(auto&& movingObstacle : movingObstacles){
                    for(auto&& tio : movingObstacle.timeIntervalObstacles){
                        // Find corresponding time interval obstacles
                        if((centerOfTimeInterval >= tio.tStart) && (centerOfTimeInterval <= tio.tEnd)){
                            if(!tio.convexPolygons.empty()){
                                // Check collision for this time interval
                                std::vector<T> subCurve(curve.begin() + k, curve.begin() + n);
                                if(CheckCollisionCurve(tio.convexPolygons, subCurve, thresholdPosition, thresholdAngle)){
                                    return true;
                                }
                            }
                            break;
                        }
                    }
                }
            }
            return false;
        }

    protected:
        /* Hide some of the derived base member functions and make them protected. Prevent transformation of the convex vehicle shape vertices. */
        using mpsv::geometry::ConvexPolygon::Translate;
        using mpsv::geometry::ConvexPolygon::Transform;

        /**
         * @brief Perform a fast overlap check for a line connection between two given poses by testing the AABB of the line segment with AABBs of the static obstacles.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose0 First pose vector, containing at least {x, y, psi}.
         * @param[in] pose1 Second pose vector, containing at least {x, y, psi}.
         * @return True if any of the obstacles AABBs overlaps with the AABB of the line segment.
         * @details The AABB of the line segment is calculated from an AABB containing the given two poses, extended by the farthest vertex distance of the vehicle shape.
         */
        template <class T, class U> bool FastOverlapCheckAABB(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose0, U pose1) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<U>::value,"Argument {pose1} must be of type std::array<double,N> with N >= 3!");
            mpsv::geometry::AABB aabb(pose0[0], pose0[1], pose1[0], pose1[1]);
            aabb.Extend(farthestVertexDistance);
            bool overlap = false;
            for(auto&& obs : staticObstacles){
                overlap |= obs.OverlapAABB(aabb);
            }
            return overlap;
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */

