#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/QuickHull2D.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief This class represents an oriented box for a 2-dimensional euclidean space. The box is indicated by an origin (lower-left corner), a normal vector and a dimension.
 * The normal vector points to the longitudinal axis of the oriented box.
 */
class OrientedBox {
    public:
        /**
         * @brief Create an oriented box. By default the box orientation is zero.
         */
        OrientedBox() noexcept {
            origin.fill(0.0);
            normal = {1.0, 0.0};
            dimension.fill(0.0);
        }

        /**
         * @brief Create the oriented box for a given center, angle and dimension.
         * @param[in] centerPose The center pose of the box, given as {x,y,psi}. The angle represents the major axis (x) of the box.
         * @param[in] dimension Dimension of the box along the major and minor axes (x,y) of the box.
         */
        void Create(std::array<double,3> centerPose, std::array<double,2> dimension) noexcept {
            normal[0] = std::cos(centerPose[2]);
            normal[1] = std::sin(centerPose[2]);
            this->dimension = dimension;
            origin[0] = centerPose[0] - (normal[0]*dimension[0] - normal[1]*dimension[1]) / 2.0;
            origin[1] = centerPose[1] - (normal[1]*dimension[0] + normal[0]*dimension[1]) / 2.0;
        }

        /**
         * @brief Create the oriented box by specifying new box parameters.
         * @param[in] origin The origin of the box. This is the lower-left corner in the box frame.
         * @param[in] normal The normal vector that specifies the orientation of the box. The normal vector lies in the x-axis (longitudinal direction) of the box frame.
         * @param[in] dimension The dimension of the box given in box frame coordinates {xdim, ydim}. BOTH VALUES MUST BE POSITIVE TO ENSURE A VALID BOX WITH NON-ZERO AREA!
         * @details The normal vector is normalized. If the vector has a length close to zero, a default unit vector {1,0} is set!
         */
        void Create(std::array<double,2> origin, std::array<double,2> normal, std::array<double,2> dimension) noexcept {
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            this->origin = origin;
            this->normal = {1.0, 0.0};
            this->dimension = dimension;
            double L = normal[0]*normal[0] + normal[1]*normal[1];
            if(L >= eps){
                L = 1.0 / std::sqrt(L);
                this->normal[0] = L * normal[0];
                this->normal[1] = L * normal[1];
            }
        }

        /**
         * @brief Create the oriented box from two given focal points. The main axis of the box is defined by the vector between two focal points. The size of the box is extended in longitudinal and lateral direction.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @tparam U Template parameter must be of type std::array<double,N> with N >= 2.
         * @param point0 The first focal point, which must contain at least {x,y}.
         * @param point1 The second focal point, which must contain at least {x,y}.
         * @param boxExtension Absolute values for box extension in longitudinal and lateral box direction. BOTH VALUES MUST BE POSITIVE TO ENSURE A VALID BOX WITH NON-ZERO AREA!
         */
        template <class T, class U> void CreateFromFocalPoints(const T& point0, const U& point1, std::array<double,2> boxExtension) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<U>::value,"Argument {point1} must be of type std::array<double,N> with N >= 2!");
            normal[0] = point1[0] - point0[0];
            normal[1] = point1[1] - point0[1];
            double d = std::sqrt(normal[0]*normal[0] + normal[1]*normal[1]);
            if(d < std::numeric_limits<double>::epsilon()){
                d = 0.0;
                normal[0] = 1.0;
                normal[1] = 0.0;
            }
            else{
                normal[0] /= d;
                normal[1] /= d;
            }
            double b1 = normal[0]*boxExtension[0];
            double b2 = normal[1]*boxExtension[0];
            double b3 = normal[0]*boxExtension[1];
            double b4 = normal[1]*boxExtension[1];
            origin[0] = point0[0] - b1 + b4;
            origin[1] = point0[1] - b2 - b3;
            dimension[0] = boxExtension[0] + boxExtension[0] + d;
            dimension[1] = boxExtension[1] + boxExtension[1];
        }

        /**
         * @brief Create the minimum-area oriented box for a set of 2D points.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         * @note The resulting box may have zero dimension if the input point cloud contains two unique points (1 dimension is zero) or only one unique point (2 dimensions are zero).
         */
        template <class T> void CreateFromPointCloud(const std::vector<T>& points) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            std::vector<int32_t> indices = mpsv::geometry::QuickHull2DIndices(points);
            if(indices.empty()){
                SetBoxVariant(points);
            }
            else{
                SetMinimumAreaBox(points, indices);
            }
        }

        /**
         * @brief Extend the whole oriented by an absolute value. The box is extended by this value in positive
         * and negative direction. Thus, the total dimension of the box grows by 2*e in each dimension.
         * @param[in] e The box extension value. The absolute value is used.
         */
        void Extend(double e) noexcept {
            e = std::fabs(e);
            double e2 = e + e;
            dimension[0] += e2;
            dimension[1] += e2;
            origin[0] -= e * (normal[0] - normal[1]);
            origin[1] -= e * (normal[1] + normal[0]);
        }

        /**
         * @brief Check whether a given point is inside the oriented box including borders.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The point to check.
         * @return True if the given point is inside this oriented box or on its borders, false otherwise.
         */
        template <class T> bool IsInside(const T point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            std::array<double,2> p, s;
            p[0] = point[0] - origin[0];
            p[1] = point[1] - origin[1];
            s[0] = p[0]*normal[0] + p[1]*normal[1];
            s[1] = p[1]*normal[0] - p[0]*normal[1];
            return (s[0] >= 0.0) && (s[0] <= dimension[0]) && (s[1] >= 0.0) && (s[1] <= dimension[1]);
        }

        /**
         * @brief Get the position of the box.
         * @return The absolute 2D position of the oriented box that corresponds to the lower-left corner in box coordinates
         */
        std::array<double,2> GetPosition(void) const noexcept { return origin; }

        /**
         * @brief Get the normal vector of the box.
         * @return The normal vector that represents the orientation of the box. This vector points to the longitudinal direction of the box.
         */
        std::array<double,2> GetNormal(void) const noexcept { return normal; }

        /**
         * @brief Get the dimension of the box.
         * @return The dimension of the oriented box given in box coordinates. The first element corresponds to the longitudinal dimension.
         */
        std::array<double,2> GetDimension(void) const noexcept { return dimension; }

        /**
         * @brief Export the oriented box as a convex polygon.
         * @return The convex polygon that represents the oriented box.
         */
        mpsv::geometry::ConvexPolygon ExportAsConvexPolygon(void) noexcept {
            std::vector<std::array<double,2>> vertices;
            double xLon = normal[0]*dimension[0];
            double yLon = normal[1]*dimension[0];
            double xLat = -normal[1]*dimension[1];
            double yLat = normal[0]*dimension[1];
            vertices.push_back(origin);
            vertices.push_back({origin[0] + xLon, origin[1] + yLon});
            vertices.push_back({origin[0] + xLon + xLat, origin[1] + yLon + yLat});
            vertices.push_back({origin[0] + xLat, origin[1] + yLat});
            return mpsv::geometry::ConvexPolygon(vertices);
        }

    protected:
        std::array<double,2> origin;      // The origin of the box. This is the lower-left corner in the box frame.
        std::array<double,2> normal;      // The normal vector that specifies the orientation of the box. The normal vector lies in the x-axis (longitudinal direction) of the box frame.
        std::array<double,2> dimension;   // The dimension of the box given in box frame coordinates {xdim, ydim}.

        /**
         * @brief Set properties for one possible variant of an oriented box. This function is a fallback function.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         */
        template <class T> void SetBoxVariant(const std::vector<T>& points) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            auto [idxA, idxB, squaredDistance] = FindFarthestPoints(points);
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            origin.fill(0.0);
            normal = {1.0, 0.0};
            dimension.fill(0.0);
            if((idxA < 0) || (idxB < 0)){
                return;
            }
            if(squaredDistance < eps){
                origin = {points[idxA][0], points[idxA][1]};
                return;
            }

            // Find largest distance perpenticular to the line (both sides of the line)
            double aEdge = points[idxB][1] - points[idxA][1];
            double bEdge = points[idxA][0] - points[idxB][0];
            double cEdge = -aEdge*points[idxA][0] - bEdge*points[idxA][1];
            double f, fMax = 0.0, fMin = 0.0;
            int32_t N = static_cast<int32_t>(points.size());
            for(int32_t i = 0; i < N; ++i){
                f = aEdge * points[i][0] + bEdge * points[i][1] + cEdge;
                fMax = std::max(fMax, f);
                fMin = std::min(fMin, f);
            }

            // Calculate box parameters
            dimension[0] = std::sqrt(squaredDistance);
            dimension[1] = std::fabs(fMax - fMax); // prevent -0.0
            normal[0] = -bEdge / dimension[0];
            normal[1] = aEdge / dimension[0];
            origin[0] = points[idxA][0] + normal[1] * fMax;
            origin[1] = points[idxA][1] - normal[0] * fMax;
        }

        /**
         * @brief Find that two points that have the greatest distance to each other.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud for which two find the two most distant points.
         * @return Tuple containing {idxA, idxB, squaredDistance} idxA and idxB are the two indices to those points that are most distant to each other, with squaredDistance being the squared distance between the two points.
         * @return {0,0,0} if there's only one point or a set on unique points with distance 0.
         * @return {-1,-1,0} if there're no input points.
         */
        template <class T> std::tuple<int32_t,int32_t,double> FindFarthestPoints(const std::vector<T>& points) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            int32_t N = static_cast<int32_t>(points.size());
            int32_t idxA = -1, idxB = -1;
            double greatestSquaredDistance = 0.0;
            if(N){
                idxA = idxB = 0;
                double dx, dy, squaredDistance;
                for(int32_t a = 0; a < N; ++a){
                    for(int32_t b = (a + 1); b < N; ++b){
                        dx = points[b][0] - points[a][0];
                        dy = points[b][1] - points[a][1];
                        squaredDistance = dx*dx + dy*dy;
                        if(squaredDistance > greatestSquaredDistance){
                            greatestSquaredDistance = squaredDistance;
                            idxA = a;
                            idxB = b;
                        }
                    }
                }
            }
            return std::make_tuple(idxA, idxB, greatestSquaredDistance);
        }

        /**
         * @brief Find a minimum-area oriented box and set properties @ref origin, @ref normal and @ref dimension.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         * @param[in] indices Indices to those points indicating a convex hull.
         */
        template <class T> void SetMinimumAreaBox(const std::vector<T>& points, const std::vector<int32_t>& indices) noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");

            // Go through all edges of the convex hull and find the bounding box with the orientation of that edge angle that minimizes the area of the box
            double minArea = std::numeric_limits<double>::infinity();
            normal = {1.0, 0.0};
            origin.fill(0.0);
            dimension.fill(0.0);
            int32_t numHullPoints = static_cast<int32_t>(indices.size());
            double edgeAngle, s, c, xmin, xmax, ymin, ymax, xk, yk, x, y, area;
            for(int32_t n = 0; n < numHullPoints; ++n){
                // Compute the angle of the current edge
                const double& x0 = points[indices[n]][0];
                const double& y0 = points[indices[n]][1];
                const double& x1 = points[indices[(n + 1) % numHullPoints]][0];
                const double& y1 = points[indices[(n + 1) % numHullPoints]][1];
                edgeAngle = std::atan2(y1 - y0, x1 - x0);
                #ifdef MPSV_SAFE_ATAN2
                if(!std::isfinite(edgeAngle)){
                    edgeAngle = 0.0;
                }
                #endif
                s = std::sin(edgeAngle);
                c = std::cos(edgeAngle);

                // Go through all vertices, transform them with the current edgeAngle and get farest points in all directions
                xmin = xmax = ymin = ymax = 0.0;
                for(int32_t k = 0; k < numHullPoints; ++k){
                    // Translate all points to {x0,y0} and rotate them with the edgeAngle
                    xk = points[indices[k]][0] - x0;
                    yk = points[indices[k]][1] - y0;
                    x = c*xk + s*yk;
                    y = -s*xk + c*yk;

                    // Now do a simple min/max check to get the farest points in all directions
                    xmin = std::min(xmin, x);
                    xmax = std::max(xmax, x);
                    ymin = std::min(ymin, y);
                    ymax = std::max(ymax, y);
                }

                // Calculate the area
                xk = (xmax - xmin);
                yk = (ymax - ymin);
                area = xk * yk;

                // Check if new minimum area has been found, rotate and translate back the origin
                if(area < minArea){
                    minArea = area;
                    normal = {c, s};
                    origin = {c*xmin - s*ymin + x0, s*xmin + c*ymin + y0};
                    dimension = {xk, yk};
                }
            }
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */

