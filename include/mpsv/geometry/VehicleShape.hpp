#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/ConvexVehicleShape.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief The vehicle shape represented as a list of convex vehicle shapes.
 */
class VehicleShape {
    public:
        /**
         * @brief Create a vehicle shape based on a given set of vertices, that represent a convex polygon.
         * @param[inout] vertices Reference to a container that contains vertices. This container is swapped internally and will be empty after a successful operation.
         * @details All @ref m_convexVehicleShapes are removed and the given vertices are used to set one convex vehicle shape.
         */
        void Create(std::vector<std::array<double, 2>>& vertices) noexcept {
            Clear();
            Add(vertices);
        }

        /**
         * @brief Add a convex polygon to this vehicle shape.
         * @param[inout] vertices Reference to a container that contains vertices. This container is swapped internally and will be empty after a successful operation.
         */
        void Add(std::vector<std::array<double, 2>>& vertices) noexcept {
            m_convexVehicleShapes.push_back(mpsv::geometry::ConvexVehicleShape(vertices));
        }

        /**
         * @brief Clear the list of convex vehicle shapes.
         */
        void Clear(void) noexcept {
            m_convexVehicleShapes.clear();
        }

        /**
         * @brief Swap this vehicle shape with a given other vehicle shape.
         * @param[in] other The vehicle shape with which to swap.
         */
        void Swap(mpsv::geometry::VehicleShape& other) noexcept {
            m_convexVehicleShapes.swap(other.m_convexVehicleShapes);
        }

        /**
         * @brief Check whether the internal polygons of this vehicle shape are all valid.
         * @return True if all internal polygons are valid, false otherwise. If this vehicle shape does not contain any polygons, false is returned.
         */
        [[nodiscard]] bool IsValid(void) const noexcept {
            bool isValid = !m_convexVehicleShapes.empty();
            for(auto&& v : m_convexVehicleShapes){
                isValid &= v.IsValid();
            }
            return isValid;
        }

        /**
         * @brief Check whether a point is inside the vehicle shape or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The query point. The array must contain at least 2 elements {x,y}.
         * @return True if point is inside the convex polygon (closed set, including borders), false otherwise. If this vehicle shape does not contain convex polygons, false is returned.
         */
        template <class T> [[nodiscard]] bool IsInside(const T point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            for(auto&& v : m_convexVehicleShapes){
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
        template <class T> [[nodiscard]] bool CheckCollisionPose(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            for(auto&& v : m_convexVehicleShapes){
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
         * @param[in] maxAngleDeviation Maximum angle deviation for path subdivision.
         * @return True if a collision occurs, false otherwise.
         */
        template <class T, class U> [[nodiscard]] bool CheckCollisionLine(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, T pose0, U pose1, double maxAngleDeviation) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose0} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<U>::value,"Argument {pose1} must be of type std::array<double,N> with N >= 3!");
            for(auto&& v : m_convexVehicleShapes){
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
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification.
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification.
         * @return True if a collision occurs, false otherwise.
         */
        template <class T> [[nodiscard]] bool CheckCollisionCurve(const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, const std::vector<T>& curve, double thresholdPosition, double thresholdAngle) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {curve} must be of type std::vector<std::array<double,N>> with N >= 3!");
            for(auto&& v : m_convexVehicleShapes){
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
        [[nodiscard]] double GetFarthestVertexDistance(void) const noexcept {
            double d = 0.0;
            for(auto&& v : m_convexVehicleShapes){
                d = std::max(d, v.GetFarthestVertexDistance());
            }
            return d;
        }

        /**
         * @brief Get a copy of the convex vehicle shapes.
         * @return The convex vehicle shapes that represent this vehicle shape.
         */
        [[nodiscard]] std::vector<mpsv::geometry::ConvexVehicleShape> GetConvexShapes(void) const noexcept {
            return m_convexVehicleShapes;
        }

    private:
        std::vector<mpsv::geometry::ConvexVehicleShape> m_convexVehicleShapes;   // List of convex vehicle shapes.
};


} /* namespace: geometry */


} /* namespace: mpsv */
