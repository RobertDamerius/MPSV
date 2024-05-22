#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/ConvexVehicleShape.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief The vehicle shape represented as a list of convex vehicle shapes.
 */
class VehicleShape {
    public:
        std::vector<mpsv::geometry::ConvexVehicleShape> convexVehicleShapes;   // List of convex vehicle shapes.

        /**
         * @brief Clear the list of convex vehicle shapes.
         */
        void Clear(void) noexcept {
            convexVehicleShapes.clear();
        }

        /**
         * @brief Add a convex polygon to this vehicle shape.
         * @param[in] vertices Reference to a container that contains vertices. This container is swapped internally and will be empty after this operation.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        void Add(std::vector<std::array<double, 2>>& vertices) noexcept {
            convexVehicleShapes.push_back(mpsv::geometry::ConvexVehicleShape());
            convexVehicleShapes.back().Create(vertices);
        }

        /**
         * @brief Create a vehicle shape based on a given set of vertices, that represent a convex polygon.
         * @param[in] vertices Reference to a container that contains vertices. This container is swapped internally and will be empty after this operation.
         * @details All @ref convexVehicleShapes are removed and the given vertices are used to set one convex vehicle shape.
         */
        void Create(std::vector<std::array<double, 2>>& vertices) noexcept {
            Clear();
            Add(vertices);
        }

        /**
         * @brief Ensure the correct vertex order by possibly reversing the order of the internal vertices.
         * @return True if all internal polygons are convex and the vertex order is correct or has been corrected, false otherwise. If this vehicle shape does not contain any polygons, true is returned.
         */
        bool EnsureCorrectVertexOrder(void) noexcept {
            bool convex = true;
            for(auto&& v : convexVehicleShapes){
                convex &= v.EnsureCorrectVertexOrder();
            }
            return convex;
        }

        /**
         * @brief Check whether the internal polygons of this vehicle shape are all convex.
         * @return True if all internal polygons are convex, false otherwise. If this vehicle shape does not contain any polygons, false is returned.
         */
        bool InternalPolygonsAreConvex(void) const noexcept{
            bool convex = true;
            for(auto&& v : convexVehicleShapes){
                convex &= v.IsConvex();
            }
            return !convexVehicleShapes.empty() && convex;
        }

        /**
         * @brief Check whether all internal shapes of this vehicle shape contain finite values.
         * @return True if all values are finite, false otherwise. If this vehicle shape does not contain any polygons, true is returned.
         */
        bool IsFinite(void) const noexcept{
            bool finite = true;
            for(auto&& v : convexVehicleShapes){
                finite &= v.IsFinite();
            }
            return finite;
        }

        /**
         * @brief Check whether a point is inside the vehicle shape or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The query point. The array must contain at least 2 elements {x,y}.
         * @return True if point is inside the convex polygon (closed set, including borders), false otherwise. If this vehicle shape does not contain convex polygons, false is returned.
         */
        template <class T> bool IsInside(const T point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            for(auto&& v : convexVehicleShapes){
                if(v.IsInside(point)){
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Check whether the vehicle shape collides with static obstacles or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] staticObstacles The static obstacles to be considered for collision checking.
         * @param[in] pose The pose of the vehicle to be used for collision checking.
         * @return True if a collision occurs, false otherwise.
         */
        template <class T> bool CheckCollisionPose(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            for(auto&& v : convexVehicleShapes){
                if(v.CheckCollisionPose(staticObstacles, pose)){
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
         */
        template <class T, class U> bool CheckCollisionLine(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose0, U pose1, double maxAngleDeviation) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<U>::value,"Argument {pose1} must be of type std::array<double,N> with N >= 3!");
            for(auto&& v : convexVehicleShapes){
                if(v.CheckCollisionLine(staticObstacles, pose0, pose1, maxAngleDeviation)){
                    return true;
                }
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
         */
        template <class T> bool CheckCollisionCurve(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const std::vector<T>& curve, double thresholdPosition, double thresholdAngle) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {curve} must be of type std::vector<std::array<double,N>> with N >= 3!");
            for(auto&& v : convexVehicleShapes){
                if(v.CheckCollisionCurve(staticObstacles, curve, thresholdPosition, thresholdAngle)){
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Get the farthest distance to all vertices of the convex polygons from the origin {0,0}.
         * @return Farthest distance to all vertices from the origin {0,0}.
         * @details The distance is calculated, if the vertex data is changed, e.g. during @ref Create or @ref Add.
         */
        double GetFarthestVertexDistance(void) const noexcept {
            double d = 0.0;
            for(auto&& v : convexVehicleShapes){
                d = std::max(d, v.GetFarthestVertexDistance());
            }
            return d;
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */

