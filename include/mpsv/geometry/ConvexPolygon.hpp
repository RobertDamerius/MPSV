#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace geometry {


/* Forward declaration */
class ConvexPolygon;
template <class T> ConvexPolygon QuickHull2DPolygon(const std::vector<T>& refPoints) noexcept;


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
        explicit ConvexPolygon(std::vector<std::array<double, 2>> vertices) noexcept {
            Create(vertices);
        }

        /**
         * @brief Create the convex polygon based on vertices.
         * @param[inout] vertices Reference to a container that contains vertices. This container is empty after a successful polygon creation.
         * @note The vertex order may be changed to ensure a valid math. positive order of vertices. If the given vertices do not indicate a valid
         * polygon, the internal vertex storage of this convex polygon remains empty.
         */
        void Create(std::vector<std::array<double, 2>>& vertices) noexcept {
            // Clear existing data
            m_vertices.clear();
            m_edges.clear();
            m_farthestVertexDistance = 0.0;
            m_aabb.Reset({0.0, 0.0});

            // Validate input
            const auto& vRef = vertices;
            if(vRef.size() < 3){
                return;
            }
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            uint32_t N = static_cast<uint32_t>(vRef.size());
            int32_t order = 0;
            int32_t numSignChangesX = 0;
            int32_t numSignChangesY = 0;
            for(uint32_t k = 0; k < N; ++k){
                const auto& v0 = vRef[(k + N - 1) % N];
                const auto& v1 = vRef[k];
                const auto& v2 = vRef[(k + 1) % N];
                double dx1 = v1[0] - v0[0];
                double dy1 = v1[1] - v0[1];
                if(!std::isfinite(v1[0]) || !std::isfinite(v1[1])){
                    return; // non-finite vertex
                }
                if((dx1*dx1 + dy1*dy1) <= eps){
                    return; // zero-length edge: duplicate point -> invalid polygon
                }
                double dx2 = v2[0] - v1[0];
                double dy2 = v2[1] - v1[1];
                double z = dx1*dy2 - dy1*dx2;
                if(!order){
                    order = (z >= 0.0) ? 1 : -1;
                }
                if(((order > 0) && (z < eps)) || ((order < 0) && (z > eps))){
                    return; // invalid vertex order
                }
                bool sx1 = (dx1 >= 0.0);
                bool sy1 = (dy1 >= 0.0);
                bool sx2 = (dx2 >= 0.0);
                bool sy2 = (dy2 >= 0.0);
                numSignChangesX += static_cast<int32_t>(sx1 != sx2);
                numSignChangesY += static_cast<int32_t>(sy1 != sy2);
            }
            if((2 != numSignChangesX) || (2 != numSignChangesY)){
                return; // line or spiral -> invalid polygon
            }
            if(order < 0){
                std::reverse(vertices.begin(), vertices.end()); // ensure math. positive order
            }

            // Store vertices and set container sizes
            m_vertices.swap(vertices);
            size_t numVertices = m_vertices.size();
            m_edges.resize(numVertices);

            // Update all properties of the convex polygon (AABB, edges, ...)
            m_farthestVertexDistance = std::sqrt(m_vertices[0][0]*m_vertices[0][0] + m_vertices[0][1]*m_vertices[0][1]);
            m_aabb.Reset(m_vertices[0]);
            for(size_t k = 1; k < numVertices; ++k){
                RebuildEdge(k - 1, k);
                m_aabb += m_vertices[k];
                m_farthestVertexDistance = std::max(m_farthestVertexDistance, std::sqrt(m_vertices[k][0]*m_vertices[k][0] + m_vertices[k][1]*m_vertices[k][1]));
            }
            RebuildEdge(numVertices - 1, 0);
        }

        /**
         * @brief Check if this polygon is valid, e.g. contains valid data.
         * @return True if this polygon represents a valid convex polygon, false otherwise.
         */
        bool IsValid(void) const noexcept { return (m_vertices.size() > 2); }

        /**
         * @brief Check whether a point is inside the convex polygon or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The query point. The array must contain at least 2 elements {x,y}.
         * @return True if point is inside the convex polygon (closed set, including borders), false otherwise. If this polygon is invalid, false is returned.
         */
        template <class T> bool IsInside(const T point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            if(m_edges.empty()){
                return false;
            }
            bool outside = false;
            for(auto&& edgeParameter : m_edges){
                outside |= ((edgeParameter[0] * point[0] + edgeParameter[1] * point[1] + edgeParameter[2]) > 0.0);
            }
            return !outside;
        }

        /**
         * @brief Transform all vertices of the convex polygon. The transformation is specified by a rotation around the origin {0,0} followed by a translation.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be used for transformation.
         * @details All edges are rebuild after the transformation of all vertices. @ref m_farthestVertexDistance ist updated.
         * @note This member function has no effect, if this polygon is not valid.
         */
        template <class T> void Transform(T pose) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            size_t numVertices = m_vertices.size();
            bool validInput = std::isfinite(pose[0]) && std::isfinite(pose[1]) && std::isfinite(pose[2]);
            if(validInput && (numVertices > 2)){
                double cosPsi = std::cos(pose[2]);
                double sinPsi = std::sin(pose[2]);
                double vx = m_vertices[0][0];
                double vy = m_vertices[0][1];
                m_vertices[0][0] = pose[0] + cosPsi * vx - sinPsi * vy;
                m_vertices[0][1] = pose[1] + sinPsi * vx + cosPsi * vy;
                m_aabb.Reset(m_vertices[0]);
                m_farthestVertexDistance = std::sqrt(m_vertices[0][0]*m_vertices[0][0] + m_vertices[0][1]*m_vertices[0][1]);
                for(size_t k = 1; k < numVertices; ++k){
                    vx = m_vertices[k][0];
                    vy = m_vertices[k][1];
                    m_vertices[k][0] = pose[0] + cosPsi * vx - sinPsi * vy;
                    m_vertices[k][1] = pose[1] + sinPsi * vx + cosPsi * vy;
                    RebuildEdge(k - 1, k);
                    m_aabb += m_vertices[k];
                    m_farthestVertexDistance = std::max(m_farthestVertexDistance, std::sqrt(m_vertices[k][0]*m_vertices[k][0] + m_vertices[k][1]*m_vertices[k][1]));
                }
                RebuildEdge(numVertices - 1, 0);
            }
        }

        /**
         * @brief Create a convex polygon from all transformed vertices.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be used for transformation.
         * @return A convex polygon that indicates the transformed convex polygon. If this convex polygon contains less than 3 vertices, a copy of this convex polygon without transformation is returned.
         */
        template <class T> ConvexPolygon CreateTransformedPolygon(T pose) const noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
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
        ConvexPolygon CreateTransformedConvexHull(double x0, double y0, double cosPsi0, double sinPsi0, double x1, double y1, double cosPsi1, double sinPsi1) const noexcept {
            // Transform vertices by two configurations and create point cloud
            size_t numVertices = m_vertices.size();
            std::vector<std::array<double,2>> points;
            points.resize(2 * numVertices);
            for(size_t k = 0; k < numVertices; ++k){
                points[k][0] = x0 + cosPsi0 * m_vertices[k][0] - sinPsi0 * m_vertices[k][1];
                points[k][1] = y0 + sinPsi0 * m_vertices[k][0] + cosPsi0 * m_vertices[k][1];
                points[k + numVertices][0] = x1 + cosPsi1 * m_vertices[k][0] - sinPsi1 * m_vertices[k][1];
                points[k + numVertices][1] = y1 + sinPsi1 * m_vertices[k][0] + cosPsi1 * m_vertices[k][1];
            }

            // Compute the convex hull
            return mpsv::geometry::QuickHull2DPolygon(points);
        }

        /**
         * @brief Check whether a convex polygon overlaps with this convex polygon.
         * @param[in] polygon The convex polygon for which to check collision or overlapping.
         * @return True if the convex polygon overlaps with this convex polygon, false otherwise. If one polygon is invalid, false is returned.
         */
        bool Overlap(const ConvexPolygon& polygon) const noexcept {
            // Check for validity
            if(!IsValid() || !polygon.IsValid()){
                return false;
            }

            // Check if AABBs overlap
            if(!m_aabb.Overlap(polygon.m_aabb)){
                return false;
            }

            // Type Edge-Vertex (edges of this with vertices of polygon input)
            size_t numVertices = polygon.m_vertices.size();
            size_t thisNumVertices = m_vertices.size();
            double vx, vy, v1x, v1y, v2x, v2y;
            bool inCFree = false;
            for(size_t n = 0; (n < thisNumVertices) && !inCFree; ++n){
                const double& nx = m_edges[n][0];
                const double& ny = m_edges[n][1];
                const double& x1 = m_vertices[n][0];
                const double& y1 = m_vertices[n][1];
                vx = polygon.m_vertices[0][0] - x1;
                vy = polygon.m_vertices[0][1] - y1;
                v1x = polygon.m_vertices[numVertices-1][0] - polygon.m_vertices[0][0];
                v1y = polygon.m_vertices[numVertices-1][1] - polygon.m_vertices[0][1];
                v2x = polygon.m_vertices[1][0] - polygon.m_vertices[0][0];
                v2y = polygon.m_vertices[1][1] - polygon.m_vertices[0][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                for(size_t k = 1; (k < (numVertices - 1)) && !inCFree; k++){
                    vx = polygon.m_vertices[k][0] - x1;
                    vy = polygon.m_vertices[k][1] - y1;
                    v1x = polygon.m_vertices[k-1][0] - polygon.m_vertices[k][0];
                    v1y = polygon.m_vertices[k-1][1] - polygon.m_vertices[k][1];
                    v2x = polygon.m_vertices[k+1][0] - polygon.m_vertices[k][0];
                    v2y = polygon.m_vertices[k+1][1] - polygon.m_vertices[k][1];
                    inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                }
                vx = polygon.m_vertices[numVertices-1][0] - x1;
                vy = polygon.m_vertices[numVertices-1][1] - y1;
                v1x = polygon.m_vertices[numVertices-2][0] - polygon.m_vertices[numVertices-1][0];
                v1y = polygon.m_vertices[numVertices-2][1] - polygon.m_vertices[numVertices-1][1];
                v2x = polygon.m_vertices[0][0] - polygon.m_vertices[numVertices-1][0];
                v2y = polygon.m_vertices[0][1] - polygon.m_vertices[numVertices-1][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
            }

            // Type Vertex-Edge (vertices of this with edges of polygon input)
            for(size_t n = 0; (n < numVertices) && !inCFree; ++n){
                const double& nx = polygon.m_edges[n][0];
                const double& ny = polygon.m_edges[n][1];
                const double& x1 = polygon.m_vertices[n][0];
                const double& y1 = polygon.m_vertices[n][1];
                vx = m_vertices[0][0] - x1;
                vy = m_vertices[0][1] - y1;
                v1x = m_vertices[thisNumVertices-1][0] - m_vertices[0][0];
                v1y = m_vertices[thisNumVertices-1][1] - m_vertices[0][1];
                v2x = m_vertices[1][0] - m_vertices[0][0];
                v2y = m_vertices[1][1] - m_vertices[0][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                for(size_t k = 1; (k < (thisNumVertices - 1)) && !inCFree; k++){
                    vx = m_vertices[k][0] - x1;
                    vy = m_vertices[k][1] - y1;
                    v1x = m_vertices[k-1][0] - m_vertices[k][0];
                    v1y = m_vertices[k-1][1] - m_vertices[k][1];
                    v2x = m_vertices[k+1][0] - m_vertices[k][0];
                    v2y = m_vertices[k+1][1] - m_vertices[k][1];
                    inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                }
                vx = m_vertices[thisNumVertices-1][0] - x1;
                vy = m_vertices[thisNumVertices-1][1] - y1;
                v1x = m_vertices[thisNumVertices-2][0] - m_vertices[thisNumVertices-1][0];
                v1y = m_vertices[thisNumVertices-2][1] - m_vertices[thisNumVertices-1][1];
                v2x = m_vertices[0][0] - m_vertices[thisNumVertices-1][0];
                v2y = m_vertices[0][1] - m_vertices[thisNumVertices-1][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
            }

            // It's a collision if no configuration is in CFree
            return !inCFree;
        }

        /**
         * @brief Check whether the AABB of this convex polygon overlaps with a given AABB.
         * @param[in] aabb Axis-aligned bounding box to be checked for overlapping.
         * @return True if AABBs overlap, false otherwise. If this polygon is invalid, false is returned.
         */
        bool OverlapAABB(mpsv::geometry::AABB& aabb) const noexcept { return IsValid() && m_aabb.Overlap(aabb); }

        /**
         * @brief Calculate the minimum squared distance to all edges of this convex polygon.
         * @param[in] x X coordinate of the point for which to calculate the minimum squared distance.
         * @param[in] y Y coordinate of the point for which to calculate the minimum squared distance.
         * @return The minimum squared distance to all edges of this convex polygon or infinity, if this convex polygon is invalid.
         */
        double MinimumSquaredDistanceToEdges(double x, double y) const noexcept {
            size_t numVertices = m_vertices.size();
            if(numVertices < 3){
                return std::numeric_limits<double>::infinity();
            }
            double ax = x - m_vertices[numVertices - 1][0];
            double ay = y - m_vertices[numVertices - 1][1];
            double bx = m_vertices[0][0] - m_vertices[numVertices - 1][0];
            double by = m_vertices[0][1] - m_vertices[numVertices - 1][1];
            double aa = ax*ax + ay*ay;
            double ab = ax*bx + ay*by;
            double bb = bx*bx + by*by;
            double lambda = std::clamp(ab / bb, 0.0, 1.0);
            double minSquaredDistance = aa + (bb*lambda - 2.0 * ab)*lambda;
            for(size_t k = 1; k < numVertices; ++k){
                ax = x - m_vertices[k - 1][0];
                ay = y - m_vertices[k - 1][1];
                bx = m_vertices[k][0] - m_vertices[k - 1][0];
                by = m_vertices[k][1] - m_vertices[k - 1][1];
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
        double GetFarthestVertexDistance(void) const noexcept { return m_farthestVertexDistance; }

        /**
         * @brief Get a copy of the internal vertices.
         * @return Copy of the internal vertices.
         */
        std::vector<std::array<double, 2>> GetVertices(void) const noexcept { return m_vertices; }

        /**
         * @brief Get the 2D axis-aligned bounding box of this convex polygon.
         * @return 2D axis-aligned bounding box of this convex polygon.
         */
        mpsv::geometry::AABB GetAABB(void) noexcept { return m_aabb; }

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
        std::vector<std::array<double, 2>> m_vertices;   // Container of vertices. One data point indicates vertex coordinates (x, y).
        std::vector<std::array<double, 3>> m_edges;      // Container of edges. One data point (a,b,c) indicates edge parameters (function coefficient a of implicit equation a*x + b*y + c = 0). The size of this container is equal to that of @ref m_vertices. Edge [i] goes from vertex [i] to vertex [i+1].
        mpsv::geometry::AABB m_aabb;                     // The 2D axis-aligned bounding box of the polygon.
        double m_farthestVertexDistance;                 // Distance to the farthest vertex from the origin. The distance is calculated, if the vertex data is changed, e.g. during @ref Create, @ref Translate, @ref Transform, ...

        /**
         * @brief Rebuild edge parameters based on vertex indices.
         * @param[in] idx Index of the vertex where the edge starts.
         * @param[in] idxNext Index of the next vertex where the edge ends.
         */
        void RebuildEdge(size_t idx, size_t idxNext) noexcept {
            double nx = m_vertices[idxNext][1] - m_vertices[idx][1];
            double ny = m_vertices[idx][0] - m_vertices[idxNext][0];
            double len = std::sqrt(nx*nx + ny*ny);
            double invlen = 1.0 / (len + static_cast<double>(len == 0.0));
            m_edges[idx][0] = invlen * nx;
            m_edges[idx][1] = invlen * ny;
            m_edges[idx][2] = -m_edges[idx][0] * m_vertices[idx][0] - m_edges[idx][1] * m_vertices[idx][1];
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */
