#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>


namespace mpsv {


namespace geometry {


namespace details {


/**
 * @brief Recursive helper function for the QuickHull algorithm.
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @param[out] indices The container where to store the indices to reference points that form the convex hull.
 * @param[in] refPoints The reference points (2D) to be used.
 * @param[in] idxPoints The indices to reference points from which to determine the convex hull.
 * @param[in] idxA Index to the first point of a line that divides the point set.
 * @param[in] idxB Index to the second point of a line that divides the point set.
 * @param[in] idxFarthest Index to the reference point that is farthest away from the line dividing the point set.
 */
template<class T> inline void QuickHull2DRecursive(std::vector<int32_t>& indices, const std::vector<T>& refPoints, std::vector<int32_t>& idxPoints, int32_t idxA, int32_t idxB, int32_t idxFarthest) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {refPoints} must be of type std::vector<std::array<double,N>> with N >= 2!");

    // Terminate if no points in the list
    int32_t N = static_cast<int32_t>(idxPoints.size());
    if(!N) return;

    // Edge from A to farthest
    double aEdgeA = refPoints[idxFarthest][1] - refPoints[idxA][1];
    double bEdgeA = refPoints[idxA][0] - refPoints[idxFarthest][0];
    double cEdgeA = -aEdgeA*refPoints[idxA][0] - bEdgeA*refPoints[idxA][1];

    // Edge from farthest to B
    double aEdgeB = refPoints[idxB][1] - refPoints[idxFarthest][1];
    double bEdgeB = refPoints[idxFarthest][0] - refPoints[idxB][0];
    double cEdgeB = -aEdgeB*refPoints[idxFarthest][0] - bEdgeB*refPoints[idxFarthest][1];

    // Use points on positive side only
    std::vector<int32_t> idxPointsA, idxPointsB;
    idxPointsA.reserve(N);
    idxPointsB.reserve(N);
    double dmaxA = 0.0, dmaxB = 0.0;
    int32_t idxFarthestA = -1, idxFarthestB = -1;
    for(int32_t k = 0; k < N; k++){
        if(idxFarthest == idxPoints[k]) continue;
        double dA = aEdgeA * refPoints[idxPoints[k]][0] + bEdgeA * refPoints[idxPoints[k]][1] + cEdgeA;
        double dB = aEdgeB * refPoints[idxPoints[k]][0] + bEdgeB * refPoints[idxPoints[k]][1] + cEdgeB;
        if(dA > 0){
            idxPointsA.push_back(idxPoints[k]);
            if(dA > dmaxA){
                dmaxA = dA;
                idxFarthestA = idxPoints[k];
            }
        }
        if(dB > 0){
            idxPointsB.push_back(idxPoints[k]);
            if(dB > dmaxB){
                dmaxB = dB;
                idxFarthestB = idxPoints[k];
            }
        }
    }

    // Find hulls recursively
    if(idxFarthestA >= 0) QuickHull2DRecursive(indices, refPoints, idxPointsA, idxA, idxFarthest, idxFarthestA);
    indices.push_back(idxFarthest);
    if(idxFarthestB >= 0) QuickHull2DRecursive(indices, refPoints, idxPointsB, idxFarthest, idxB, idxFarthestB);
}


} /* namespace: details */


/**
 * @brief Compute the convex hull for a set of 2D points using the QuickHull algorithm.
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @param[in] refPoints The reference points (2D) to be used by the algorithm. A point must be at least of dimension 2.
 * @return Indices to input reference points that form the convex hull. The indices output container will be empty if the number of 2D points is less than three!
 * @note The ring is not closed, that is the last index in the list is not equal to the first one.
 */
template <class T> std::vector<int32_t> QuickHull2DIndices(const std::vector<T>& refPoints) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {refPoints} must be of type std::vector<std::array<double,N>> with N >= 2!");
    std::vector<int32_t> indices;
    int32_t numPoints = static_cast<int32_t>(refPoints.size());
    if(numPoints < 3) return indices;
    indices.reserve(numPoints);

    // Find min/max points
    int32_t idxMin = 0, idxMax = 0;
    for(int32_t i = 1; i < numPoints; i++){
        if(refPoints[i][0] < refPoints[idxMin][0]){
            idxMin = i;
        }
        else if(refPoints[i][0] > refPoints[idxMax][0]){
            idxMax = i;
        }
    }

    // Edge of min/max points
    double a = refPoints[idxMax][1] - refPoints[idxMin][1];
    double b = refPoints[idxMin][0] - refPoints[idxMax][0];
    double c = -a * refPoints[idxMin][0] - b * refPoints[idxMin][1];

    // Divide points in two segments positive (P) and negative (N)
    std::vector<int32_t> indicesPos, indicesNeg;
    int32_t idxPosFarthest = -1, idxNegFarthest = -1;
    double dPosFarthest = 0.0, dNegFarthest = 0.0;
    for(int32_t k = 0; k < numPoints; k++){
        if((k == idxMin) || (k == idxMax)) continue;
        double d = a * refPoints[k][0] + b * refPoints[k][1] + c;
        if(d < 0){
            indicesNeg.push_back(k);
            if(d < dNegFarthest){
                dNegFarthest = d;
                idxNegFarthest = k;
            }
        }
        else{
            indicesPos.push_back(k);
            if(d > dPosFarthest){
                dPosFarthest = d;
                idxPosFarthest = k;
            }
        }
    }

    // Find convex hull for both sets
    indices.push_back(idxMin);
    if(idxPosFarthest >= 0) mpsv::geometry::details::QuickHull2DRecursive(indices, refPoints, indicesPos, idxMin, idxMax, idxPosFarthest);
    indices.push_back(idxMax);
    if(idxNegFarthest >= 0) mpsv::geometry::details::QuickHull2DRecursive(indices, refPoints, indicesNeg, idxMax, idxMin, idxNegFarthest);

    // Convexity check
    if(indices.size() < 3){
        indices.clear();
    }
    else{
        constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
        std::vector<int32_t> keep;
        int32_t N = static_cast<int32_t>(indices.size());
        keep.reserve(N);
        double curNormalX, curNormalY;
        double prevNormalX = refPoints[indices[0]][1] - refPoints[indices.back()][1];
        double prevNormalY = refPoints[indices.back()][0] - refPoints[indices[0]][0];
        double len = std::sqrt(prevNormalX*prevNormalX + prevNormalY*prevNormalY);
        len = 1.0 / (len + double(len == 0.0));
        prevNormalX *= len;
        prevNormalY *= len;
        for(int32_t i = 0; i < N; i++){
            int32_t iNext = (i + 1) % N;
            curNormalX = refPoints[indices[iNext]][1] - refPoints[indices[i]][1];
            curNormalY = refPoints[indices[i]][0] - refPoints[indices[iNext]][0];
            len = std::sqrt(curNormalX*curNormalX + curNormalY*curNormalY);
            len = 1.0 / (len + double(len == 0.0));
            curNormalX *= len;
            curNormalY *= len;
            if(std::fabs(prevNormalX * curNormalY - prevNormalY * curNormalX) > eps){
                keep.push_back(indices[i]);
                prevNormalX = curNormalX;
                prevNormalY = curNormalY;
            }
        }
        indices.swap(keep);
        if(indices.size() < 3){
            indices.clear();
        }
    }
    return indices;
}


/**
 * @brief Compute the convex hull for a set of 2D points using the QuickHull algorithm.
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @param[in] refPoints The reference points (2D) to be used by the algorithm. A point must be at least of dimension 2.
 * @return The convex polygon representing the convex hull.
 */
template <class T> mpsv::geometry::ConvexPolygon QuickHull2DPolygon(const std::vector<T>& refPoints) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {refPoints} must be of type std::vector<std::array<double,N>> with N >= 2!");
    mpsv::geometry::ConvexPolygon hull;
    std::vector<int32_t> indices = mpsv::geometry::QuickHull2DIndices(refPoints);
    size_t N = indices.size();
    if(N < 3){
        return hull;
    }
    std::vector<std::array<double, 2>> hullVertices;
    hullVertices.reserve(N);
    for(size_t n = 0; n < N; ++n){
        hullVertices.push_back({refPoints[indices[n]][0],refPoints[indices[n]][1]});
    }
    hull.Create(hullVertices);
    return hull;
}


} /* namespace: geometry */


} /* namespace: mpsv */

