#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace geometry {


/* Forward declaration */
class ConvexPolygon;
template <class Tpoint> ConvexPolygon QuickHull2DPolygon(const std::vector<Tpoint>& refPoints) noexcept;


/**
 * @brief This class represents a convex polygon.
 */
class ConvexPolygon {
    public:
        /**
         * @brief Default construction for a convex polygon.
         */
        ConvexPolygon() noexcept {
            m_farthestVertexDistance = 0.0;
        }

        /**
         * @brief Construct a convex polygon based on given vertices.
         * @param[in] vertices Container of vertices.
         */
        explicit ConvexPolygon(std::vector<std::array<double, 2U>> vertices) noexcept {
            Create(vertices);
        }

        /**
         * @brief Create the convex polygon based on vertices.
         * @param[inout] vertices Reference to a container that contains vertices. This container is empty after a successful polygon creation.
         * @note The vertex order may be reversed to ensure a mathematically positive (counterclockwise) orientation. If the given vertices do not indicate a valid
         * polygon, the internal vertex storage of this convex polygon remains empty.
         */
        void Create(std::vector<std::array<double, 2U>>& vertices) noexcept {
            // Clear existing data
            m_vertices.clear();
            m_edges.clear();
            m_farthestVertexDistance = 0.0;
            m_aabb.Reset({0.0, 0.0});

            // Validate input
            const auto& vRef = vertices;
            if (vRef.size() < 3U) {
                return;
            }
            constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
            std::uint32_t N{static_cast<std::uint32_t>(vRef.size())};
            std::int32_t order{0};
            std::int32_t numSignChangesX{0};
            std::int32_t numSignChangesY{0};
            for (std::uint32_t k{0U}; k < N; ++k) {
                const auto& v0 = vRef[(k + N - 1U) % N];
                const auto& v1 = vRef[k];
                const auto& v2 = vRef[(k + 1U) % N];
                double dx1{v1[0U] - v0[0U]};
                double dy1{v1[1U] - v0[1U]};
                if (!std::isfinite(v1[0U]) || !std::isfinite(v1[1U])) {
                    return; // non-finite vertex
                }
                if ((dx1*dx1 + dy1*dy1) <= eps) {
                    return; // zero-length edge: duplicate point -> invalid polygon
                }
                double dx2{v2[0U] - v1[0U]};
                double dy2{v2[1U] - v1[1U]};
                double z{dx1*dy2 - dy1*dx2};
                if (!order) {
                    order = (z >= 0.0) ? 1 : -1;
                }
                if (((order > 0) && (z < eps)) || ((order < 0) && (z > eps))) {
                    return; // invalid vertex order
                }
                bool sx1{(dx1 >= 0.0)};
                bool sy1{(dy1 >= 0.0)};
                bool sx2{(dx2 >= 0.0)};
                bool sy2{(dy2 >= 0.0)};
                numSignChangesX += static_cast<std::int32_t>(sx1 != sx2);
                numSignChangesY += static_cast<std::int32_t>(sy1 != sy2);
            }
            if ((2 != numSignChangesX) || (2 != numSignChangesY)) {
                return; // line or spiral -> invalid polygon
            }
            if (order < 0) {
                std::reverse(vertices.begin(), vertices.end()); // ensure math. positive order
            }

            // Store vertices and set container sizes
            m_vertices.swap(vertices);
            std::size_t numVertices{m_vertices.size()};
            m_edges.resize(numVertices);

            // Update all properties of the convex polygon (AABB, edges, ...)
            m_farthestVertexDistance = std::sqrt(m_vertices[0U][0U]*m_vertices[0U][0U] + m_vertices[0U][1U]*m_vertices[0U][1U]);
            m_aabb.Reset(m_vertices[0U]);
            for (std::size_t k{1U}; k < numVertices; ++k) {
                RebuildEdge(k - 1U, k);
                m_aabb += m_vertices[k];
                m_farthestVertexDistance = std::max(m_farthestVertexDistance, std::sqrt(m_vertices[k][0U]*m_vertices[k][0U] + m_vertices[k][1U]*m_vertices[k][1U]));
            }
            RebuildEdge(numVertices - 1U, 0U);
        }

        /**
         * @brief Check if this polygon is valid, e.g. contains valid data.
         * @return True if this polygon represents a valid convex polygon, false otherwise.
         */
        [[nodiscard]] bool IsValid() const noexcept {
            return (m_vertices.size() > 2U);
        }

        /**
         * @brief Check whether a point is inside the convex polygon or not.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The query point. The array must contain at least 2 elements {x,y}.
         * @return True if point is inside the convex polygon (closed set, including borders), false otherwise. If this polygon is invalid, false is returned.
         */
        template <class Tpoint> [[nodiscard]] bool IsInside(const Tpoint point) const noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {point} must be of type std::array<double,N> with N >= 2!");
            if (m_edges.empty()) {
                return false;
            }
            bool outside{false};
            for (auto&& edgeParameter : m_edges) {
                outside |= ((edgeParameter[0U] * point[0U] + edgeParameter[1U] * point[1U] + edgeParameter[2U]) > 0.0);
            }
            return !outside;
        }

        /**
         * @brief Transform all vertices of the convex polygon. The transformation is specified by a rotation around the origin {0,0} followed by a translation.
         * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be used for transformation.
         * @details All edges are rebuild after the transformation of all vertices. @ref m_farthestVertexDistance ist updated.
         * @note This member function has no effect, if this polygon is not valid.
         */
        template <class Tpose> void Transform(Tpose pose) noexcept {
            static_assert(mpsv::is_vec3<Tpose>::value, "Argument {pose} must be of type std::array<double,N> with N >= 3!");
            std::size_t numVertices{m_vertices.size()};
            bool validInput{std::isfinite(pose[0U]) && std::isfinite(pose[1U]) && std::isfinite(pose[2U])};
            if (validInput && (numVertices > 2U)) {
                double cosPsi{std::cos(pose[2U])};
                double sinPsi{std::sin(pose[2U])};
                double vx{m_vertices[0U][0U]};
                double vy{m_vertices[0U][1U]};
                m_vertices[0U][0U] = pose[0U] + cosPsi * vx - sinPsi * vy;
                m_vertices[0U][1U] = pose[1U] + sinPsi * vx + cosPsi * vy;
                m_aabb.Reset(m_vertices[0U]);
                m_farthestVertexDistance = std::sqrt(m_vertices[0U][0U]*m_vertices[0U][0U] + m_vertices[0U][1U]*m_vertices[0U][1U]);
                for (std::size_t k{1U}; k < numVertices; ++k) {
                    vx = m_vertices[k][0U];
                    vy = m_vertices[k][1U];
                    m_vertices[k][0U] = pose[0U] + cosPsi * vx - sinPsi * vy;
                    m_vertices[k][1U] = pose[1U] + sinPsi * vx + cosPsi * vy;
                    RebuildEdge(k - 1U, k);
                    m_aabb += m_vertices[k];
                    m_farthestVertexDistance = std::max(m_farthestVertexDistance, std::sqrt(m_vertices[k][0U]*m_vertices[k][0U] + m_vertices[k][1U]*m_vertices[k][1U]));
                }
                RebuildEdge(numVertices - 1U, 0U);
            }
        }

        /**
         * @brief Create a convex polygon from all transformed vertices.
         * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be used for transformation.
         * @return A convex polygon that indicates the transformed convex polygon. If this convex polygon contains less than 3 vertices, a copy of this convex polygon without transformation is returned.
         */
        template <class Tpose> [[nodiscard]] ConvexPolygon CreateTransformedPolygon(Tpose pose) const noexcept {
            static_assert(mpsv::is_vec3<Tpose>::value, "Argument {pose} must be of type std::array<double,N> with N >= 3!");
            ConvexPolygon result = *this;
            result.Transform(pose);
            return result;
        }

        /**
         * @brief Create a convex hull polygon from all vertices of two transformed polygons.
         * @param[in] x0 Translation in the x direction for configuration 0.
         * @param[in] y0 Translation in the y direction for configuration 0.
         * @param[in] cosPsi0 The cosine of rotation angle psi for configuration 0.
         * @param[in] sinPsi0 The sine of rotation angle psi for configuration 0.
         * @param[in] x1 Translation in the x direction for configuration 1.
         * @param[in] y1 Translation in the y direction for configuration 1.
         * @param[in] cosPsi1 The cosine of rotation angle psi for configuration 1.
         * @param[in] sinPsi1 The sine of rotation angle psi for configuration 1.
         * @return A convex polygon that indicates the convex hull of two transformed polygons.
         * @details You need to set vertices using the @ref Create member function or by construction before calling this function! If this convex polygon contains less than 3 vertices, an empty hull may be returned.
         */
        [[nodiscard]] ConvexPolygon CreateTransformedConvexHull(double x0, double y0, double cosPsi0, double sinPsi0, double x1, double y1, double cosPsi1, double sinPsi1) const noexcept {
            // Transform vertices by two configurations and create point cloud
            std::size_t numVertices{m_vertices.size()};
            std::vector<std::array<double, 2U>> points;
            points.resize(2U * numVertices);
            for (std::size_t k{0U}; k < numVertices; ++k) {
                points[k][0U] = x0 + cosPsi0 * m_vertices[k][0U] - sinPsi0 * m_vertices[k][1U];
                points[k][1U] = y0 + sinPsi0 * m_vertices[k][0U] + cosPsi0 * m_vertices[k][1U];
                points[k + numVertices][0U] = x1 + cosPsi1 * m_vertices[k][0U] - sinPsi1 * m_vertices[k][1U];
                points[k + numVertices][1U] = y1 + sinPsi1 * m_vertices[k][0U] + cosPsi1 * m_vertices[k][1U];
            }

            // Compute the convex hull
            return mpsv::geometry::QuickHull2DPolygon(points);
        }

        /**
         * @brief Check whether a convex polygon overlaps with this convex polygon.
         * @param[in] polygon The convex polygon for which to check collision or overlapping.
         * @return True if the convex polygon overlaps with this convex polygon, false otherwise. If one polygon is invalid, false is returned.
         */
        [[nodiscard]] bool Overlap(const ConvexPolygon& polygon) const noexcept {
            // Check for validity
            if (!IsValid() || !polygon.IsValid()) {
                return false;
            }

            // Check if AABBs overlap
            if (!m_aabb.Overlap(polygon.m_aabb)) {
                return false;
            }

            // Check for separation by edges
            const auto isSeparated = [](const std::vector<std::array<double, 3U>>& edges, const std::vector<std::array<double, 2U>>& vertices) noexcept {
                for (const auto& edge : edges) {
                    bool allVerticesOutside{true};
                    for (const auto& vertex : vertices) {
                        if ((edge[0U] * vertex[0U] + edge[1U] * vertex[1U] + edge[2U]) <= 0.0) {
                            allVerticesOutside = false;
                            break;
                        }
                    }
                    if (allVerticesOutside) {
                        return true;
                    }
                }
                return false;
            };
            return !isSeparated(m_edges, polygon.m_vertices) && !isSeparated(polygon.m_edges, m_vertices);
        }

        /**
         * @brief Check whether the AABB of this convex polygon overlaps with a given AABB.
         * @param[in] aabb Axis-aligned bounding box to be checked for overlapping.
         * @return True if AABBs overlap, false otherwise. If this polygon is invalid, false is returned.
         */
        [[nodiscard]] bool OverlapAABB(mpsv::geometry::AABB& aabb) const noexcept {
            return IsValid() && m_aabb.Overlap(aabb);
        }

        /**
         * @brief Calculate the minimum squared distance to all edges of this convex polygon.
         * @param[in] x X coordinate of the point for which to calculate the minimum squared distance.
         * @param[in] y Y coordinate of the point for which to calculate the minimum squared distance.
         * @return The minimum squared distance to all edges of this convex polygon or infinity, if this convex polygon is invalid.
         */
        [[nodiscard]] double MinimumSquaredDistanceToEdges(double x, double y) const noexcept {
            std::size_t numVertices{m_vertices.size()};
            if (numVertices < 3U) {
                return std::numeric_limits<double>::infinity();
            }
            double ax{x - m_vertices[numVertices - 1U][0U]};
            double ay{y - m_vertices[numVertices - 1U][1U]};
            double bx{m_vertices[0U][0U] - m_vertices[numVertices - 1U][0U]};
            double by{m_vertices[0U][1U] - m_vertices[numVertices - 1U][1U]};
            double aa{ax*ax + ay*ay};
            double ab{ax*bx + ay*by};
            double bb{bx*bx + by*by};
            double lambda{std::clamp(ab / bb, 0.0, 1.0)};
            double minSquaredDistance{aa + (bb*lambda - 2.0 * ab)*lambda};
            for (std::size_t k{1U}; k < numVertices; ++k) {
                ax = x - m_vertices[k - 1U][0U];
                ay = y - m_vertices[k - 1U][1U];
                bx = m_vertices[k][0U] - m_vertices[k - 1U][0U];
                by = m_vertices[k][1U] - m_vertices[k - 1U][1U];
                aa = ax*ax + ay*ay;
                ab = ax*bx + ay*by;
                bb = bx*bx + by*by;
                lambda = std::clamp(ab / bb, 0.0, 1.0);
                minSquaredDistance = std::min(minSquaredDistance, aa + (bb*lambda - 2.0 * ab)*lambda);
            }
            return minSquaredDistance;
        }

        /**
         * @brief Get the farthest distance to all vertices from the origin {0,0}.
         * @return Farthest distance to all vertices from the origin {0,0} or 0, if this polygon is invalid.
         * @details The distance is calculated, if the vertex data is changed, e.g. during @ref Create, @ref Translate, @ref Transform, ...
         */
        [[nodiscard]] double GetFarthestVertexDistance() const noexcept {
            return m_farthestVertexDistance;
        }

        /**
         * @brief Get a copy of the internal vertices.
         * @return Copy of the internal vertices.
         */
        [[nodiscard]] std::vector<std::array<double, 2U>> GetVertices() const noexcept {
            return m_vertices;
        }

        /**
         * @brief Get the 2D axis-aligned bounding box of this convex polygon.
         * @return 2D axis-aligned bounding box of this convex polygon.
         */
        [[nodiscard]] mpsv::geometry::AABB GetAABB() const noexcept {
            return m_aabb;
        }

        /**
         * @brief Swap this convex polygon with a given convex polygon.
         * @param[inout] polygon The polygon to be used for swapping.
         */
        void Swap(ConvexPolygon& polygon) noexcept {
            m_vertices.swap(polygon.m_vertices);
            m_edges.swap(polygon.m_edges);
            m_aabb.Swap(polygon.m_aabb);
            std::swap(m_farthestVertexDistance, polygon.m_farthestVertexDistance);
        }

    protected:
        std::vector<std::array<double, 2U>> m_vertices;   // Container of vertices. One data point indicates vertex coordinates (x, y).
        std::vector<std::array<double, 3U>> m_edges;      // Container of edges. One data point (a,b,c) indicates edge parameters (function coefficient a of implicit equation a*x + b*y + c = 0). The size of this container is equal to that of @ref m_vertices. Edge [i] goes from vertex [i] to vertex [i+1].
        mpsv::geometry::AABB m_aabb;                     // The 2D axis-aligned bounding box of the polygon.
        double m_farthestVertexDistance;                 // Distance to the farthest vertex from the origin. The distance is calculated, if the vertex data is changed, e.g. during @ref Create, @ref Translate, @ref Transform, ...

        /**
         * @brief Rebuild edge parameters based on vertex indices.
         * @param[in] idx Index of the vertex where the edge starts.
         * @param[in] idxNext Index of the next vertex where the edge ends.
         */
        void RebuildEdge(std::size_t idx, std::size_t idxNext) noexcept {
            double nx{m_vertices[idxNext][1U] - m_vertices[idx][1U]};
            double ny{m_vertices[idx][0U] - m_vertices[idxNext][0U]};
            double len{std::sqrt(nx*nx + ny*ny)};
            double invlen{1.0 / (len + static_cast<double>(len == 0.0))};
            m_edges[idx][0U] = invlen * nx;
            m_edges[idx][1U] = invlen * ny;
            m_edges[idx][2U] = -m_edges[idx][0U] * m_vertices[idx][0U] - m_edges[idx][1U] * m_vertices[idx][1U];
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */
