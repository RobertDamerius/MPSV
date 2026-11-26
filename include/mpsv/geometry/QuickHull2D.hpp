#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>


namespace mpsv::geometry {


/**
 * @brief Compute the convex hull for a set of 2D points using the QuickHull algorithm.
 * @tparam Tpoint This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @param[in] refPoints The reference points (2D) to be used by the algorithm. A point must be at least of dimension 2.
 * @return Indices to input reference points that form the convex hull. The result is empty if the input has fewer than three points or the computed hull has fewer than three vertices (for example, if the points are collinear).
 * @note The ring is not closed, that is the last index in the list is not equal to the first one.
 */
template <class Tpoint> [[nodiscard]] inline std::vector<std::int32_t> QuickHull2DIndices(const std::vector<Tpoint>& refPoints) noexcept {
    static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {refPoints} must be of type std::vector<std::array<double,N>> with N >= 2!");
    std::vector<std::int32_t> indices;
    std::int32_t numPoints{static_cast<std::int32_t>(refPoints.size())};
    if (numPoints < 3) {
        return indices;
    }
    indices.reserve(numPoints);

    // Find min/max points
    std::int32_t idxMin{0}, idxMax{0};
    for (std::int32_t i{1}; i < numPoints; i++) {
        if (refPoints[i][0U] < refPoints[idxMin][0U]) {
            idxMin = i;
        } else if (refPoints[i][0U] > refPoints[idxMax][0U]) {
            idxMax = i;
        }
    }

    // Edge of min/max points
    double a{refPoints[idxMax][1U] - refPoints[idxMin][1U]};
    double b{refPoints[idxMin][0U] - refPoints[idxMax][0U]};
    double c{-a * refPoints[idxMin][0U] - b * refPoints[idxMin][1U]};

    // Divide points in two segments positive (P) and negative (N)
    std::vector<std::int32_t> indicesPos, indicesNeg;
    std::int32_t idxPosFarthest{-1}, idxNegFarthest{-1};
    double dPosFarthest{0.0}, dNegFarthest{0.0};
    for (std::int32_t k{0}; k < numPoints; k++) {
        if ((k == idxMin) || (k == idxMax)) {
            continue;
        }
        double d{a * refPoints[k][0U] + b * refPoints[k][1U] + c};
        if (d < 0) {
            indicesNeg.push_back(k);
            if (d < dNegFarthest) {
                dNegFarthest = d;
                idxNegFarthest = k;
            }
        } else {
            indicesPos.push_back(k);
            if (d > dPosFarthest) {
                dPosFarthest = d;
                idxPosFarthest = k;
            }
        }
    }

    // Process both sets iteratively in the same traversal
    indices.push_back(idxMin);
    struct Action {
        std::vector<std::int32_t> points;
        std::int32_t idxA;
        std::int32_t idxB;
        std::int32_t idxFarthest;
        bool emit;
    };
    std::vector<Action> stack;
    if (idxNegFarthest >= 0) {
        stack.push_back({std::move(indicesNeg), idxMax, idxMin, idxNegFarthest, false});
    }
    stack.push_back({{}, 0, 0, idxMax, true});
    if (idxPosFarthest >= 0) {
        stack.push_back({std::move(indicesPos), idxMin, idxMax, idxPosFarthest, false});
    }
    while (!stack.empty()) {
        Action action = std::move(stack.back());
        stack.pop_back();
        if (action.emit) {
            indices.push_back(action.idxFarthest);
            continue;
        }

        // Skip if no points are left to process
        std::int32_t N{static_cast<std::int32_t>(action.points.size())};
        if (N < 1) {
            continue;
        }

        // Compute edges of triangle formed by points A, B and Farthest
        double aEdgeA{refPoints[action.idxFarthest][1U] - refPoints[action.idxA][1U]};
        double bEdgeA{refPoints[action.idxA][0U] - refPoints[action.idxFarthest][0U]};
        double cEdgeA{-aEdgeA*refPoints[action.idxA][0U] - bEdgeA*refPoints[action.idxA][1U]};
        double aEdgeB{refPoints[action.idxB][1U] - refPoints[action.idxFarthest][1U]};
        double bEdgeB{refPoints[action.idxFarthest][0U] - refPoints[action.idxB][0U]};
        double cEdgeB{-aEdgeB*refPoints[action.idxFarthest][0U] - bEdgeB*refPoints[action.idxFarthest][1U]};

        // Split points into two sets, one for each edge of the triangle
        std::vector<std::int32_t> idxPointsA, idxPointsB;
        idxPointsA.reserve(N);
        idxPointsB.reserve(N);
        double dmaxA = 0.0, dmaxB = 0.0;
        std::int32_t idxFarthestA = -1, idxFarthestB = -1;
        for (std::int32_t k{0}; k < N; k++) {
            std::int32_t pointIndex{action.points[k]};
            if (action.idxFarthest == pointIndex) {
                continue;
            }
            double dA{aEdgeA * refPoints[pointIndex][0U] + bEdgeA * refPoints[pointIndex][1U] + cEdgeA};
            double dB{aEdgeB * refPoints[pointIndex][0U] + bEdgeB * refPoints[pointIndex][1U] + cEdgeB};
            if (dA > 0) {
                idxPointsA.push_back(pointIndex);
                if (dA > dmaxA) {
                    dmaxA = dA;
                    idxFarthestA = pointIndex;
                }
            }
            if (dB > 0) {
                idxPointsB.push_back(pointIndex);
                if (dB > dmaxB) {
                    dmaxB = dB;
                    idxFarthestB = pointIndex;
                }
            }
        }
        // Push new actions to the stack
        if (idxFarthestB >= 0) {
            stack.push_back({std::move(idxPointsB), action.idxFarthest, action.idxB, idxFarthestB, false});
        }
        stack.push_back({{}, 0, 0, action.idxFarthest, true});
        if (idxFarthestA >= 0) {
            stack.push_back({std::move(idxPointsA), action.idxA, action.idxFarthest, idxFarthestA, false});
        }
    }

    // Convexity check
    if (indices.size() < 3U) {
        indices.clear();
    } else {
        constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
        std::int32_t N{static_cast<std::int32_t>(indices.size())};
        std::int32_t numKept{0};
        std::int32_t firstIndex{indices[0U]};
        double curNormalX, curNormalY;
        double prevNormalX{refPoints[indices[0U]][1U] - refPoints[indices.back()][1U]};
        double prevNormalY{refPoints[indices.back()][0U] - refPoints[indices[0U]][0U]};
        double len{std::sqrt(prevNormalX*prevNormalX + prevNormalY*prevNormalY)};
        len = 1.0 / (len + double(len == 0.0));
        prevNormalX *= len;
        prevNormalY *= len;
        for (std::int32_t i{0}; i < N; i++) {
            std::int32_t iNext{(i + 1) % N};
            std::int32_t idx{indices[i]};
            std::int32_t idxNext{(iNext == 0) ? firstIndex : indices[iNext]};
            curNormalX = refPoints[idxNext][1U] - refPoints[idx][1U];
            curNormalY = refPoints[idx][0U] - refPoints[idxNext][0U];
            len = std::sqrt(curNormalX*curNormalX + curNormalY*curNormalY);
            len = 1.0 / (len + double(len == 0.0));
            curNormalX *= len;
            curNormalY *= len;
            if (std::fabs(prevNormalX * curNormalY - prevNormalY * curNormalX) > eps) {
                indices[numKept++] = idx;
                prevNormalX = curNormalX;
                prevNormalY = curNormalY;
            }
        }
        indices.resize(static_cast<std::size_t>(numKept));
        if (indices.size() < 3U) {
            indices.clear();
        }
    }
    return indices;
}


/**
 * @brief Compute the convex hull for a set of 2D points using the QuickHull algorithm.
 * @tparam Tpoint This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @param[in] refPoints The reference points (2D) to be used by the algorithm. A point must be at least of dimension 2.
 * @return The convex polygon representing the convex hull.
 */
template <class Tpoint> [[nodiscard]] inline mpsv::geometry::ConvexPolygon QuickHull2DPolygon(const std::vector<Tpoint>& refPoints) noexcept {
    static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {refPoints} must be of type std::vector<std::array<double,N>> with N >= 2!");
    mpsv::geometry::ConvexPolygon hull;
    std::vector<std::int32_t> indices = mpsv::geometry::QuickHull2DIndices(refPoints);
    std::size_t N{indices.size()};
    if (N < 3U) {
        return hull;
    }
    std::vector<std::array<double, 2U>> hullVertices;
    hullVertices.reserve(N);
    for (std::size_t n{0U}; n < N; ++n) {
        hullVertices.push_back({refPoints[indices[n]][0U], refPoints[indices[n]][1U]});
    }
    hull.Create(hullVertices);
    return hull;
}


} /* namespace: mpsv::geometry */
