#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace geometry {


/* Forward declaration */
class ConvexPolygon;
template <class T> ConvexPolygon QuickHull2DPolygon(const std::vector<T>& refPoints) noexcept;


/**
 * @brief This class represents a convex polygon.
 * IMPORTANT NOTE: All member functions assume, that the vertex data represents a valid convex polygon. The construction of a convex polygon
 * will not fail due to invalid input data. You MUST make sure that you created a valid convex polygon with AT LEAST 3 vertices. You can check
 * the vertex data for convexity and finiteness using the @ref IsConvex and @ref IsFinite member functions, respectively.
 */
class ConvexPolygon {
    public:
        /**
         * @brief Default construction for a convex polygon.
         */
        ConvexPolygon() noexcept {
            farthestVertexDistance = 0.0;
        }

        /**
         * @brief Construct a convex polygon based on given vertices.
         * @param[in] vertices Container of vertices.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        explicit ConvexPolygon(std::vector<std::array<double, 2>> vertices) noexcept {
            Create(vertices);
        }

        /**
         * @brief Create the convex polygon based on vertices.
         * @param[in] vertices Reference to a container that contains vertices. This container will be empty after this operation.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         * This member function only swap containers and rebuild edges.
         */
        void Create(std::vector<std::array<double, 2>>& vertices) noexcept {
            if(vertices.size() < 3){
                this->vertices.clear();
                vertices.clear();
                edges.clear();
                farthestVertexDistance = 0.0;
                aabb.Reset({0.0, 0.0});
                return;
            }

            // Get vertices and set container sizes
            this->vertices.swap(vertices);
            size_t numVertices = this->vertices.size();
            edges.resize(numVertices);

            // Update all properties of the convex polygon (AABB, edges, ...)
            farthestVertexDistance = std::sqrt(this->vertices[0][0]*this->vertices[0][0] + this->vertices[0][1]*this->vertices[0][1]);
            aabb.Reset(this->vertices[0]);
            for(size_t k = 1; k < numVertices; ++k){
                RebuildEdge(k - 1, k);
                aabb += this->vertices[k];
                farthestVertexDistance = std::max(farthestVertexDistance, std::sqrt(this->vertices[k][0]*this->vertices[k][0] + this->vertices[k][1]*this->vertices[k][1]));
            }
            RebuildEdge(numVertices - 1, 0);
        }

        /**
         * @brief Ensure the correct vertex order by possibly reversing the order of the internal vertex container.
         * @return True if polygon is convex and the vertex order is correct or has been corrected, false otherwise.
         */
        bool EnsureCorrectVertexOrder(void) noexcept {
            // There must be at least 3 vertices
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            uint32_t numVertices = (uint32_t)vertices.size();
            if(numVertices < 3){
                return false;
            }

            // Go through all vertices and check correct order
            int32_t order = 0;
            int32_t numSignChangesX = 0;
            int32_t numSignChangesY = 0;
            for(uint32_t k = 0; k < numVertices; ++k){
                uint32_t kPrev = (k + numVertices - 1) % numVertices;
                uint32_t kNext = (k + 1) % numVertices;
                const std::array<double,2>& v0 = vertices[kPrev];
                const std::array<double,2>& v1 = vertices[k];
                const std::array<double,2>& v2 = vertices[kNext];
                double dx1 = v1[0] - v0[0];
                double dy1 = v1[1] - v0[1];
                double L1 = std::sqrt(dx1*dx1 + dy1*dy1);
                bool sx1 = mpsv::math::signi(dx1) >= 0;
                bool sy1 = mpsv::math::signi(dy1) >= 0;
                dx1 /= L1;
                dy1 /= L1;
                double dx2 = v2[0] - v1[0];
                double dy2 = v2[1] - v1[1];
                double L2 = std::sqrt(dx2*dx2 + dy2*dy2);
                bool sx2 = mpsv::math::signi(dx2) >= 0;
                bool sy2 = mpsv::math::signi(dy2) >= 0;
                dx2 /= L2;
                dy2 /= L2;
                double z = dx1*dy2 - dy1*dx2;
                numSignChangesX += static_cast<int32_t>(sx1 != sx2);
                numSignChangesY += static_cast<int32_t>(sy1 != sy2);
                if(!order){
                    order = (z > 0) ? 1 : -1;
                }
                if((L1 <= eps) || (L2 <= eps) || (std::fabs(z) <= eps) || ((order > 0) && (z < -eps)) || ((order < 0) && (z > eps))){
                    return false;
                }
            }
            if(order < 0){
                std::reverse(vertices.begin(), vertices.end());
                for(size_t k = 1; k < numVertices; ++k){
                    RebuildEdge(k - 1, k);
                }
                RebuildEdge(numVertices - 1, 0);
            }
            return (2 == numSignChangesX) && (2 == numSignChangesY);
        }

        /**
         * @brief Check if @ref edges represent a convex polygon.
         * @return True if polygon is convex, false otherwise. The polygon is convex if the order of vertices is correct (positive rotation around z) and there are no duplicated points or multiple rings or spirals of edges.
         */
        bool IsConvex(void) const noexcept {
            // There must be at least 3 vertices
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            uint32_t numVertices = static_cast<uint32_t>(vertices.size());
            if(numVertices < 3){
                return false;
            }

            // Go through all vertices and check correct order
            int32_t numSignChangesX = 0;
            int32_t numSignChangesY = 0;
            for(uint32_t k = 0; k < numVertices; ++k){
                uint32_t kPrev = (k + numVertices - 1) % numVertices;
                uint32_t kNext = (k + 1) % numVertices;
                auto&& v0 = vertices[kPrev];
                auto&& v1 = vertices[k];
                auto&& v2 = vertices[kNext];
                double dx1 = v1[0] - v0[0];
                double dy1 = v1[1] - v0[1];
                double L1 = std::sqrt(dx1*dx1 + dy1*dy1);
                bool sx1 = mpsv::math::signi(dx1) >= 0;
                bool sy1 = mpsv::math::signi(dy1) >= 0;
                dx1 /= L1;
                dy1 /= L1;
                double dx2 = v2[0] - v1[0];
                double dy2 = v2[1] - v1[1];
                double L2 = std::sqrt(dx2*dx2 + dy2*dy2);
                bool sx2 = mpsv::math::signi(dx2) >= 0;
                bool sy2 = mpsv::math::signi(dy2) >= 0;
                dx2 /= L2;
                dy2 /= L2;
                double z = dx1*dy2 - dy1*dx2;
                numSignChangesX += static_cast<int32_t>(sx1 != sx2);
                numSignChangesY += static_cast<int32_t>(sy1 != sy2);
                if((L1 <= eps) || (L2 <= eps) || (z <= eps)){
                    return false;
                }
            }
            return (2 == numSignChangesX) && (2 == numSignChangesY);
        }

        /**
         * @brief Check whether all internal values are finite.
         * @return True if internal values are finite, false otherwise. If the number of vertices is lower than 3, false is returned.
         */
        bool IsFinite(void) const noexcept {
            bool finite = true;
            for(auto&& v : vertices){
                finite &= std::isfinite(v[0]) && std::isfinite(v[1]);
            }
            for(auto&& e : edges){
                finite &= std::isfinite(e[0]) && std::isfinite(e[1]) && std::isfinite(e[2]);
            }
            finite &= std::isfinite(aabb.lowerBound[0]) && std::isfinite(aabb.lowerBound[1]) && std::isfinite(aabb.upperBound[0]) && std::isfinite(aabb.upperBound[1]);
            finite &= std::isfinite(farthestVertexDistance);
            return finite && (vertices.size() > 2);
        }

        /**
         * @brief Check whether a point is inside the convex polygon or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The query point. The array must contain at least 2 elements {x,y}.
         * @return True if point is inside the convex polygon (closed set, including borders), false otherwise.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT!
         */
        template <class T> bool IsInside(const T point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            bool outside = false;
            for(auto&& edgeParameter : edges){
                outside |= ((edgeParameter[0] * point[0] + edgeParameter[1] * point[1] + edgeParameter[2]) > 0.0);
            }
            return !outside;
        }

        /**
         * @brief Translate all vertices of the convex polygon.
         * @param[in] x Translation in the x direction.
         * @param[in] y Translation in the y direction.
         * @details All edges are rebuild after the translation of all vertices. @ref farthestVertexDistance ist updated.
         * @note This member function has no effect, if the number of vertices is lower than 3!
         */
        void Translate(double x, double y) noexcept {
            size_t numVertices = vertices.size();
            if(numVertices > 2){
                vertices[0][0] += x;
                vertices[0][1] += y;
                aabb.Reset(vertices[0]);
                farthestVertexDistance = std::sqrt(vertices[0][0]*vertices[0][0] + vertices[0][1]*vertices[0][1]);
                for(size_t k = 1; k < numVertices; ++k){
                    vertices[k][0] += x;
                    vertices[k][1] += y;
                    RebuildEdge(k - 1, k);
                    aabb += vertices[k];
                    farthestVertexDistance = std::max(farthestVertexDistance, std::sqrt(vertices[k][0]*vertices[k][0] + vertices[k][1]*vertices[k][1]));
                }
                RebuildEdge(numVertices - 1, 0);
            }
        }

        /**
         * @brief Transform all vertices of the convex polygon. The transformation is specified by a rotation around the origin {0,0} followed by a translation.
         * @param[in] x Translation in the x direction.
         * @param[in] y Translation in the y direction.
         * @param[in] cosPsi The cosine of rotation angle psi.
         * @param[in] sinPsi The sine of rotation angle psi.
         * @details All edges are rebuild after the transformation of all vertices. @ref farthestVertexDistance ist updated.
         * @note This member function has no effect, if the number of vertices is lower than 3!
         */
        void Transform(double x, double y, double cosPsi, double sinPsi) noexcept {
            size_t numVertices = vertices.size();
            if(numVertices > 2){
                double vx = vertices[0][0];
                double vy = vertices[0][1];
                vertices[0][0] = x + cosPsi * vx - sinPsi * vy;
                vertices[0][1] = y + sinPsi * vx + cosPsi * vy;
                aabb.Reset(vertices[0]);
                farthestVertexDistance = std::sqrt(vertices[0][0]*vertices[0][0] + vertices[0][1]*vertices[0][1]);
                for(size_t k = 1; k < numVertices; ++k){
                    vx = vertices[k][0];
                    vy = vertices[k][1];
                    vertices[k][0] = x + cosPsi * vx - sinPsi * vy;
                    vertices[k][1] = y + sinPsi * vx + cosPsi * vy;
                    RebuildEdge(k - 1, k);
                    aabb += vertices[k];
                    farthestVertexDistance = std::max(farthestVertexDistance, std::sqrt(vertices[k][0]*vertices[k][0] + vertices[k][1]*vertices[k][1]));
                }
                RebuildEdge(numVertices - 1, 0);
            }
        }

        /**
         * @brief Create a convex polygon from all transformed vertices.
         * @param[in] x Translation in the x direction.
         * @param[in] y Translation in the y direction.
         * @param[in] cosPsi The cosine of rotation angle psi.
         * @param[in] sinPsi The sine of rotation angle psi.
         * @return A convex polygon that indicates the transformed convex polygon. If this convex polygon contains less than 3 vertices, a copy of this convex polygon without transformation is returned.
         */
        ConvexPolygon CreateTransformedPolygon(double x, double y, double cosPsi, double sinPsi) const noexcept {
            ConvexPolygon result = *this;
            result.Transform(x, y, cosPsi, sinPsi);
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
            // Two points are treated as the same point if their coordinates (x,y) are closer than this value
            constexpr double thresholdSamePoint = 100.0 * std::numeric_limits<double>::epsilon();

            // Transform vertices by two configurations and create point cloud
            // Do not add duplicated points, all points in the cloud must be unique
            size_t numVertices = vertices.size();
            std::vector<std::array<double,2>> points;
            points.reserve(2 * numVertices);
            bool uniquePoint;
            double vx, vy;
            for(size_t k = 0; k < numVertices; ++k){
                vx = x0 + cosPsi0 * vertices[k][0] - sinPsi0 * vertices[k][1];
                vy = y0 + sinPsi0 * vertices[k][0] + cosPsi0 * vertices[k][1];
                uniquePoint = true;
                for(auto&& p : points){
                    if((std::fabs(vx - p[0]) < thresholdSamePoint) && (std::fabs(vy - p[1]) < thresholdSamePoint)){
                        uniquePoint = false;
                        break;
                    }
                }
                if(uniquePoint){
                    points.push_back({vx,vy});
                }
                vx = x1 + cosPsi1 * vertices[k][0] - sinPsi1 * vertices[k][1];
                vy = y1 + sinPsi1 * vertices[k][0] + cosPsi1 * vertices[k][1];
                uniquePoint = true;
                for(auto&& p : points){
                    if((std::fabs(vx - p[0]) < thresholdSamePoint) && (std::fabs(vy - p[1]) < thresholdSamePoint)){
                        uniquePoint = false;
                        break;
                    }
                }
                if(uniquePoint){
                    points.push_back({vx,vy});
                }
            }

            // Compute the convex hull
            return mpsv::geometry::QuickHull2DPolygon(points);
        }

        /**
         * @brief Check whether a convex polygon overlaps with this convex polygon.
         * @param[in] polygon The convex polygon for which to check collision or overlapping.
         * @return True if the convex polygon overlaps with this convex polygon, false otherwise.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER ANY POLYGON IS CONVEX OR NOT!
         */
        bool Overlap(const ConvexPolygon& polygon) const noexcept {
            // Check if AABBs overlap
            if(!aabb.Overlap(polygon.aabb)){
                return false;
            }

            // Type Edge-Vertex (edges of this with vertices of polygon input)
            size_t numVertices = polygon.vertices.size();
            size_t thisNumVertices = vertices.size();
            double vx, vy, v1x, v1y, v2x, v2y;
            bool inCFree = false;
            for(size_t n = 0; (n < thisNumVertices) && !inCFree; ++n){
                const double& nx = edges[n][0];
                const double& ny = edges[n][1];
                const double& x1 = vertices[n][0];
                const double& y1 = vertices[n][1];
                vx = polygon.vertices[0][0] - x1;
                vy = polygon.vertices[0][1] - y1;
                v1x = polygon.vertices[numVertices-1][0] - polygon.vertices[0][0];
                v1y = polygon.vertices[numVertices-1][1] - polygon.vertices[0][1];
                v2x = polygon.vertices[1][0] - polygon.vertices[0][0];
                v2y = polygon.vertices[1][1] - polygon.vertices[0][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                for(size_t k = 1; (k < (numVertices - 1)) && !inCFree; k++){
                    vx = polygon.vertices[k][0] - x1;
                    vy = polygon.vertices[k][1] - y1;
                    v1x = polygon.vertices[k-1][0] - polygon.vertices[k][0];
                    v1y = polygon.vertices[k-1][1] - polygon.vertices[k][1];
                    v2x = polygon.vertices[k+1][0] - polygon.vertices[k][0];
                    v2y = polygon.vertices[k+1][1] - polygon.vertices[k][1];
                    inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                }
                vx = polygon.vertices[numVertices-1][0] - x1;
                vy = polygon.vertices[numVertices-1][1] - y1;
                v1x = polygon.vertices[numVertices-2][0] - polygon.vertices[numVertices-1][0];
                v1y = polygon.vertices[numVertices-2][1] - polygon.vertices[numVertices-1][1];
                v2x = polygon.vertices[0][0] - polygon.vertices[numVertices-1][0];
                v2y = polygon.vertices[0][1] - polygon.vertices[numVertices-1][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
            }

            // Type Vertex-Edge (vertices of this with edges of polygon input)
            for(size_t n = 0; (n < numVertices) && !inCFree; ++n){
                const double& nx = polygon.edges[n][0];
                const double& ny = polygon.edges[n][1];
                const double& x1 = polygon.vertices[n][0];
                const double& y1 = polygon.vertices[n][1];
                vx = vertices[0][0] - x1;
                vy = vertices[0][1] - y1;
                v1x = vertices[thisNumVertices-1][0] - vertices[0][0];
                v1y = vertices[thisNumVertices-1][1] - vertices[0][1];
                v2x = vertices[1][0] - vertices[0][0];
                v2y = vertices[1][1] - vertices[0][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                for(size_t k = 1; (k < (thisNumVertices - 1)) && !inCFree; k++){
                    vx = vertices[k][0] - x1;
                    vy = vertices[k][1] - y1;
                    v1x = vertices[k-1][0] - vertices[k][0];
                    v1y = vertices[k-1][1] - vertices[k][1];
                    v2x = vertices[k+1][0] - vertices[k][0];
                    v2y = vertices[k+1][1] - vertices[k][1];
                    inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                }
                vx = vertices[thisNumVertices-1][0] - x1;
                vy = vertices[thisNumVertices-1][1] - y1;
                v1x = vertices[thisNumVertices-2][0] - vertices[thisNumVertices-1][0];
                v1y = vertices[thisNumVertices-2][1] - vertices[thisNumVertices-1][1];
                v2x = vertices[0][0] - vertices[thisNumVertices-1][0];
                v2y = vertices[0][1] - vertices[thisNumVertices-1][1];
                inCFree |= (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
            }

            // It's a collision if no configuration is in CFree
            return !inCFree;
        }

        /**
         * @brief Check whether the AABB of this convex polygon overlaps with a given AABB.
         * @param[in] aabb Axis-aligned bounding box to be checked for overlapping.
         * @return True if AABBs overlap, false otherwise.
         */
        bool OverlapAABB(mpsv::geometry::AABB& aabb) const noexcept { return this->aabb.Overlap(aabb); }

        /**
         * @brief Calculate the minimum squared distance to all edges of this convex polygon.
         * @param[in] x X coordinate of the point for which to calculate the minimum squared distance.
         * @param[in] y Y coordinate of the point for which to calculate the minimum squared distance.
         * @return The minimum squared distance to all edges of this convex polygon or infinity, if this convex polygon contains less than 3 vertices.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THIS POLYGON IS VALID OR NOT!
         */
        double MinimumSquaredDistanceToEdges(double x, double y) const noexcept {
            size_t numVertices = vertices.size();
            if(numVertices < 3){
                return std::numeric_limits<double>::infinity();
            }
            double ax = x - vertices[numVertices - 1][0];
            double ay = y - vertices[numVertices - 1][1];
            double bx = vertices[0][0] - vertices[numVertices - 1][0];
            double by = vertices[0][1] - vertices[numVertices - 1][1];
            double aa = ax*ax + ay*ay;
            double ab = ax*bx + ay*by;
            double bb = bx*bx + by*by;
            double lambda = std::clamp(ab / bb, 0.0, 1.0);
            double minSquaredDistance = aa + (bb*lambda - 2.0 * ab)*lambda;
            for(size_t k = 1; k < numVertices; ++k){
                ax = x - vertices[k - 1][0];
                ay = y - vertices[k - 1][1];
                bx = vertices[k][0] - vertices[k - 1][0];
                by = vertices[k][1] - vertices[k - 1][1];
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
         * @return Farthest distance to all vertices from the origin {0,0}.
         * @details The distance is calculated, if the vertex data is changed, e.g. during @ref Create, @ref Translate, @ref Transform, ...
         * If this convex polygon does not contain vertices, zero is returned.
         */
        double GetFarthestVertexDistance(void) const noexcept { return farthestVertexDistance; }

        /**
         * @brief Get a copy of the internal vertices.
         * @return Copy of the internal vertices.
         */
        std::vector<std::array<double, 2>> GetVertices(void) const noexcept { return this->vertices; }

        /**
         * @brief Get the 2D axis-aligned bounding box of this convex polygon.
         * @return 2D axis-aligned bounding box of this convex polygon.
         */
        mpsv::geometry::AABB GetAABB(void) noexcept { return aabb; }

        /**
         * @brief Swap this convex polygon with a given convex polygon.
         * @param[inout] polygon The polygon to be used for swapping.
         */
        void Swap(ConvexPolygon& polygon) noexcept {
            vertices.swap(polygon.vertices);
            edges.swap(polygon.edges);
            aabb.Swap(polygon.aabb);
            std::swap(farthestVertexDistance, polygon.farthestVertexDistance);
        }

    protected:
        std::vector<std::array<double, 2>> vertices;   // Container of vertices. One data point indicates vertex coordinates (x, y).
        std::vector<std::array<double, 3>> edges;      // Container of edges. One data point (a,b,c) indicates edge parameters (function coefficient a of implicit equation a*x + b*y + c = 0). The size of this container is equal to that of @ref vertices. Edge [i] goes from vertex [i] to vertex [i+1].
        mpsv::geometry::AABB aabb;                     // The 2D axis-aligned bounding box of the polygon.
        double farthestVertexDistance;                 // Distance to the farthest vertex from the origin. The distance is calculated, if the vertex data is changed, e.g. during @ref Create, @ref Translate, @ref Transform, ...

        /**
         * @brief Rebuild edge parameters based on vertex indices.
         * @param[in] idx Index of the vertex where the edge starts.
         * @param[in] idxNext Index of the next vertex where the edge ends.
         */
        void RebuildEdge(size_t idx, size_t idxNext) noexcept {
            double nx = vertices[idxNext][1] - vertices[idx][1];
            double ny = vertices[idx][0] - vertices[idxNext][0];
            double len = std::sqrt(nx*nx + ny*ny);
            len = 1.0 / (len + static_cast<double>(len == 0.0));
            edges[idx][0] = len * nx;
            edges[idx][1] = len * ny;
            edges[idx][2] = -edges[idx][0] * vertices[idx][0] - edges[idx][1] * vertices[idx][1];
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */

