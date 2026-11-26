#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/QuickHull2D.hpp>


namespace mpsv::geometry {


/**
 * @brief This class represents an oriented box for a 2-dimensional euclidean space. The box is characterized by an origin (lower-left corner), a normal vector and a dimension.
 * The normal vector points to the longitudinal axis of the oriented box.
 */
class OrientedBox {
    public:
        /**
         * @brief Create an oriented box. By default the box orientation is zero.
         */
        OrientedBox() noexcept {
            m_origin.fill(0.0);
            m_normal = {1.0, 0.0};
            m_dimension.fill(0.0);
        }

        /**
         * @brief Create an oriented box for a given center pose and dimension.
         * @param[in] centerPose The center pose of the box, given as {x,y,psi}.
         * @param[in] dimension Dimension of the box along its major and minor axes.
         */
        OrientedBox(const std::array<double, 3U>& centerPose, const std::array<double, 2U>& dimension) noexcept {
            Create(centerPose, dimension);
        }

        /**
         * @brief Create the oriented box for a given center, angle and dimension.
         * @param[in] centerPose The center pose of the box, given as {x,y,psi}. The angle represents the major axis (x) of the box.
         * @param[in] dimension Dimension of the box along the major and minor axes (x,y) of the box.
         */
        void Create(const std::array<double, 3U>& centerPose, const std::array<double, 2U>& dimension) noexcept {
            m_normal[0U] = std::cos(centerPose[2U]);
            m_normal[1U] = std::sin(centerPose[2U]);
            m_dimension = dimension;
            m_origin[0U] = centerPose[0U] - (m_normal[0U]*dimension[0U] - m_normal[1U]*dimension[1U]) / 2.0;
            m_origin[1U] = centerPose[1U] - (m_normal[1U]*dimension[0U] + m_normal[0U]*dimension[1U]) / 2.0;
        }

        /**
         * @brief Create the oriented box by specifying new box parameters.
         * @param[in] origin The origin of the box. This is the lower-left corner in the box frame.
         * @param[in] normal The normal vector that specifies the orientation of the box. The normal vector lies in the x-axis (longitudinal direction) of the box frame.
         * @param[in] dimension The dimension of the box given in box frame coordinates {xdim, ydim}. BOTH VALUES MUST BE POSITIVE TO ENSURE A VALID BOX WITH NON-ZERO AREA!
         * @details The normal vector is normalized. If the vector has a length close to zero, a default unit vector {1,0} is set!
         */
        void Create(const std::array<double, 2U>& origin, const std::array<double, 2U>& normal, const std::array<double, 2U>& dimension) noexcept {
            constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
            m_origin = origin;
            m_normal = {1.0, 0.0};
            m_dimension = dimension;
            double L{normal[0U]*normal[0U] + normal[1U]*normal[1U]};
            if (L >= eps) {
                L = 1.0 / std::sqrt(L);
                m_normal[0U] = L * normal[0U];
                m_normal[1U] = L * normal[1U];
            }
        }

        /**
         * @brief Create the oriented box from two given focal points. The main axis of the box is defined by the vector between two focal points. The size of the box is extended in longitudinal and lateral direction.
         * @tparam Tpoint0 Template parameter must be of type std::array<double,N> with N >= 2.
         * @tparam Tpoint1 Template parameter must be of type std::array<double,N> with N >= 2.
         * @param point0 The first focal point, which must contain at least {x,y}.
         * @param point1 The second focal point, which must contain at least {x,y}.
         * @param boxExtension Absolute values for box extension in longitudinal and lateral box direction. BOTH VALUES MUST BE POSITIVE TO ENSURE A VALID BOX WITH NON-ZERO AREA!
         */
        template <class Tpoint0, class Tpoint1> void CreateFromFocalPoints(const Tpoint0& point0, const Tpoint1& point1, const std::array<double, 2U>& boxExtension) noexcept {
            static_assert(mpsv::is_vec2<Tpoint0>::value, "Argument {point0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<Tpoint1>::value, "Argument {point1} must be of type std::array<double,N> with N >= 2!");
            constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
            m_normal[0U] = point1[0U] - point0[0U];
            m_normal[1U] = point1[1U] - point0[1U];
            double d{std::sqrt(m_normal[0U]*m_normal[0U] + m_normal[1U]*m_normal[1U])};
            if (d < eps) {
                d = 0.0;
                m_normal[0U] = 1.0;
                m_normal[1U] = 0.0;
            } else {
                m_normal[0U] /= d;
                m_normal[1U] /= d;
            }
            double b1{m_normal[0U]*boxExtension[0U]};
            double b2{m_normal[1U]*boxExtension[0U]};
            double b3{m_normal[0U]*boxExtension[1U]};
            double b4{m_normal[1U]*boxExtension[1U]};
            m_origin[0U] = point0[0U] - b1 + b4;
            m_origin[1U] = point0[1U] - b2 - b3;
            m_dimension[0U] = boxExtension[0U] + boxExtension[0U] + d;
            m_dimension[1U] = boxExtension[1U] + boxExtension[1U];
        }

        /**
         * @brief Create the minimum-area oriented box for a set of 2D points.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         * @note The resulting box may have zero dimension if the input point cloud contains two unique points (1 dimension is zero) or only one unique point (2 dimensions are zero).
         */
        template <class Tpoint> void CreateFromPointCloud(const std::vector<Tpoint>& points) noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            std::vector<std::int32_t> indices = mpsv::geometry::QuickHull2DIndices(points);
            if (indices.empty()) {
                SetBoxVariant(points);
            } else {
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
            double e2{e + e};
            m_dimension[0U] += e2;
            m_dimension[1U] += e2;
            m_origin[0U] -= e * (m_normal[0U] - m_normal[1U]);
            m_origin[1U] -= e * (m_normal[1U] + m_normal[0U]);
        }

        /**
         * @brief Check whether a given point is inside the oriented box including borders.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The point to check.
         * @return True if the given point is inside this oriented box or on its borders, false otherwise.
         */
        template <class Tpoint> [[nodiscard]] bool IsInside(const Tpoint& point) const noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {point} must be of type std::array<double,N> with N >= 2!");
            double x{point[0U] - m_origin[0U]};
            double y{point[1U] - m_origin[1U]};
            double sx{x*m_normal[0U] + y*m_normal[1U]};
            double sy{y*m_normal[0U] - x*m_normal[1U]};
            return (sx >= 0.0) && (sx <= m_dimension[0U]) && (sy >= 0.0) && (sy <= m_dimension[1U]);
        }

        /**
         * @brief Get the position of the box.
         * @return The absolute 2D position of the oriented box that corresponds to the lower-left corner in box coordinates
         */
        [[nodiscard]] std::array<double, 2U> GetPosition() const noexcept {
            return m_origin;
        }

        /**
         * @brief Get the normal vector of the box.
         * @return The normal vector that represents the orientation of the box. This vector points to the longitudinal direction of the box.
         */
        [[nodiscard]] std::array<double, 2U> GetNormal() const noexcept {
            return m_normal;
        }

        /**
         * @brief Get the dimension of the box.
         * @return The dimension of the oriented box given in box coordinates. The first element corresponds to the longitudinal dimension.
         */
        [[nodiscard]] std::array<double, 2U> GetDimension() const noexcept {
            return m_dimension;
        }

        /**
         * @brief Get the volume of the oriented box in SE(2).
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @return Volume of the oriented box.
         * @details The volume corresponds to (x) * (y) * (wpsi * psi), where wpsi denotes the weighting for psi in the metric distance function.
         */
        [[nodiscard]] double GetVolume(double metricWeightPsi) const noexcept {
            constexpr double pi2{2.0 * std::numbers::pi};
            return m_dimension[0U] * m_dimension[1U] * std::fabs(metricWeightPsi * pi2);
        }

        /**
         * @brief Export the oriented box as a convex polygon.
         * @return The convex polygon that represents the oriented box.
         */
        [[nodiscard]] mpsv::geometry::ConvexPolygon ExportAsConvexPolygon() const noexcept {
            std::vector<std::array<double, 2U>> vertices;
            double xLon{m_normal[0U]*m_dimension[0U]};
            double yLon{m_normal[1U]*m_dimension[0U]};
            double xLat{-m_normal[1U]*m_dimension[1U]};
            double yLat{m_normal[0U]*m_dimension[1U]};
            vertices.push_back(m_origin);
            vertices.push_back({m_origin[0U] + xLon, m_origin[1U] + yLon});
            vertices.push_back({m_origin[0U] + xLon + xLat, m_origin[1U] + yLon + yLat});
            vertices.push_back({m_origin[0U] + xLat, m_origin[1U] + yLat});
            return mpsv::geometry::ConvexPolygon(vertices);
        }

    protected:
        std::array<double, 2U> m_origin;      // The origin of the box. This is the lower-left corner in the box frame.
        std::array<double, 2U> m_normal;      // The normal vector that specifies the orientation of the box. The normal vector lies in the x-axis (longitudinal direction) of the box frame.
        std::array<double, 2U> m_dimension;   // The dimension of the box given in box frame coordinates {xdim, ydim}.

        /**
         * @brief Set properties for one possible variant of an oriented box. This function is a fallback function.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         */
        template <class Tpoint> void SetBoxVariant(const std::vector<Tpoint>& points) noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            auto [idxA, idxB, squaredDistance] = FindFarthestPoints(points);
            constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
            m_origin.fill(0.0);
            m_normal = {1.0, 0.0};
            m_dimension.fill(0.0);
            if ((idxA < 0) || (idxB < 0)) {
                return;
            }
            if (squaredDistance < eps) {
                m_origin = {points[idxA][0U], points[idxA][1U]};
                return;
            }

            // Find largest distance perpendicular to the line (both sides of the line)
            double aEdge{points[idxB][1U] - points[idxA][1U]};
            double bEdge{points[idxA][0U] - points[idxB][0U]};
            double cEdge{-aEdge*points[idxA][0U] - bEdge*points[idxA][1U]};
            double f, fMax = 0.0, fMin = 0.0;
            std::int32_t N{static_cast<std::int32_t>(points.size())};
            for (std::int32_t i{0}; i < N; ++i) {
                f = aEdge * points[i][0U] + bEdge * points[i][1U] + cEdge;
                fMax = std::max(fMax, f);
                fMin = std::min(fMin, f);
            }

            // Calculate box parameters
            m_dimension[0U] = std::sqrt(squaredDistance);
            m_dimension[1U] = std::fabs(fMax - fMax); // prevent -0.0
            m_normal[0U] = -bEdge / m_dimension[0U];
            m_normal[1U] = aEdge / m_dimension[0U];
            m_origin[0U] = points[idxA][0U] + m_normal[1U] * fMax;
            m_origin[1U] = points[idxA][1U] - m_normal[0U] * fMax;
        }

        /**
         * @brief Find that two points that have the greatest distance to each other.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud for which two find the two most distant points.
         * @return Tuple containing {idxA, idxB, squaredDistance} idxA and idxB are the two indices to those points that are most distant to each other, with squaredDistance being the squared distance between the two points.
         * @return {0,0,0} if there's only one point or a set on unique points with distance 0.
         * @return {-1,-1,0} if there're no input points.
         */
        template <class Tpoint> std::tuple<std::int32_t, std::int32_t, double> FindFarthestPoints(const std::vector<Tpoint>& points) noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
            std::int32_t N{static_cast<std::int32_t>(points.size())};
            std::int32_t idxA = -1, idxB = -1;
            double greatestSquaredDistance{0.0};
            if (N > 0) {
                idxA = idxB = 0;
                double dx, dy, squaredDistance;
                for (std::int32_t a{0}; a < N; ++a) {
                    for (std::int32_t b{a + 1}; b < N; ++b) {
                        dx = points[b][0U] - points[a][0U];
                        dy = points[b][1U] - points[a][1U];
                        squaredDistance = dx*dx + dy*dy;
                        if (squaredDistance > greatestSquaredDistance) {
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
         * @brief Find a minimum-area oriented box and set properties @ref m_origin, @ref m_normal and @ref m_dimension.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] points Input point cloud around which the minimum-area oriented box is to be generated.
         * @param[in] indices Indices to those points indicating a convex hull.
         */
        template <class Tpoint> void SetMinimumAreaBox(const std::vector<Tpoint>& points, const std::vector<std::int32_t>& indices) noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");

            // Go through all edges of the convex hull and find the bounding box with the orientation of that edge angle that minimizes the area of the box
            double minArea{std::numeric_limits<double>::infinity()};
            m_normal = {1.0, 0.0};
            m_origin.fill(0.0);
            m_dimension.fill(0.0);
            std::int32_t numHullPoints{static_cast<std::int32_t>(indices.size())};
            double edgeAngle, s, c, xmin, xmax, ymin, ymax, xk, yk, x, y, area;
            for (std::int32_t n{0}; n < numHullPoints; ++n) {
                // Compute the angle of the current edge
                const double& x0 = points[indices[n]][0U];
                const double& y0 = points[indices[n]][1U];
                const double& x1 = points[indices[(n + 1) % numHullPoints]][0U];
                const double& y1 = points[indices[(n + 1) % numHullPoints]][1U];
                edgeAngle = std::atan2(y1 - y0, x1 - x0);
                s = std::sin(edgeAngle);
                c = std::cos(edgeAngle);

                // Go through all vertices, transform them with the current edgeAngle and get farest points in all directions
                xmin = xmax = ymin = ymax = 0.0;
                for (std::int32_t k{0}; k < numHullPoints; ++k) {
                    // Translate all points to {x0,y0} and rotate them with the edgeAngle
                    xk = points[indices[k]][0U] - x0;
                    yk = points[indices[k]][1U] - y0;
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
                if (area < minArea) {
                    minArea = area;
                    m_normal = {c, s};
                    m_origin = {c*xmin - s*ymin + x0, s*xmin + c*ymin + y0};
                    m_dimension = {xk, yk};
                }
            }
        }
};


} /* namespace: mpsv::geometry */
