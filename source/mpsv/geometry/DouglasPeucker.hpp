#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace geometry {


namespace details {


/**
 * @brief Template helper function for recursive douglas-peucker algorithm for SE2.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 */
template<class T> inline void DouglasPeuckerSE2Recursive(std::vector<size_t>& indices, const std::vector<T>& poses, double thresholdPosition, double thresholdAngle, const size_t idxStart, const size_t idxEnd) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {poses} must be of type std::vector<std::array<double,N>> with N >= 3!");

    // Just a line: nothing to do
    if((idxStart + 1) >= idxEnd){
        return;
    }

    // Implicit edge function
    double xStart = poses[idxStart][0];
    double yStart = poses[idxStart][1];
    double xEnd = poses[idxEnd][0];
    double yEnd = poses[idxEnd][1];
    double a = yEnd - yStart;
    double b = xStart - xEnd;
    double len = std::sqrt(a*a + b*b);
    size_t ip = 0;
    double dmax = -1.0;
    double amax = -1.0;
    if(len > 0.0){
        len = 1.0 / len;
        a *= len;
        b *= len;
        double c = -a * xStart - b * yStart;

        // Search farthest point for position and largest angle error
        for(size_t i = idxStart + 1; i < idxEnd; i++){
            len = std::fabs(a*poses[i][0] + b*poses[i][1] + c);
            amax = std::max(amax, std::fabs(mpsv::math::SymmetricalAngle(poses[i][2] - poses[idxStart][2])));
            if(len > dmax){
                dmax = len;
                ip = i;
            }
        }
    }
    else{
        // Search farthest point for position and largest angle error
        for(size_t i = idxStart + 1; i < idxEnd; i++){
            xEnd = poses[i][0] - xStart;
            yEnd = poses[i][0] - yStart;
            len = xEnd*xEnd + yEnd*yEnd;
            amax = std::max(amax, std::fabs(mpsv::math::SymmetricalAngle(poses[i][2] - poses[idxStart][2])));
            if(len > dmax){
                dmax = len;
                ip = i;
            }
        }
    }

    // If position error is too large, divide according to position
    if(dmax >= thresholdPosition){
        DouglasPeuckerSE2Recursive(indices, poses, thresholdPosition, thresholdAngle, idxStart, ip);
        indices.push_back(ip);
        DouglasPeuckerSE2Recursive(indices, poses, thresholdPosition, thresholdAngle, ip, idxEnd);
    }
    // Else if angle error is too large, divide remaining interval
    else if(amax >= thresholdAngle){
        size_t ia = idxStart + (idxEnd - idxStart) / 2;
        DouglasPeuckerSE2Recursive(indices, poses, thresholdPosition, thresholdAngle, idxStart, ia);
        indices.push_back(ia);
        DouglasPeuckerSE2Recursive(indices, poses, thresholdPosition, thresholdAngle, ia, idxEnd);
    }
}


/**
 * @brief Helper function for recursive douglas-peucker algorithm for 2D.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
 */
template <class T> inline void DouglasPeucker2DRecursive(std::vector<size_t>& indices, const std::vector<T>& points, double threshold, const size_t idxStart, const size_t idxEnd) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");

    // Just a line: nothing to do
    if((idxStart + 1) >= idxEnd){
        return;
    }

    // Implicit edge function
    double xStart = points[idxStart][0];
    double yStart = points[idxStart][1];
    double xEnd = points[idxEnd][0];
    double yEnd = points[idxEnd][1];
    double a = yEnd - yStart;
    double b = xStart - xEnd;
    double len = std::sqrt(a*a + b*b);
    size_t imax = 0;
    double dmax = -1.0;
    if(len > 0.0){
        len = 1.0 / len;
        a *= len;
        b *= len;
        double c = -a * xStart - b * yStart;

        // Search farthest point
        for(size_t i = idxStart + 1; i < idxEnd; i++){
            len = std::fabs(a*points[i][0] + b*points[i][1] + c);
            if(len > dmax){
                dmax = len;
                imax = i;
            }
        }
    }
    else{
        // Search farthest point
        for(size_t i = idxStart + 1; i < idxEnd; i++){
            xEnd = points[i][0] - xStart;
            yEnd = points[i][0] - yStart;
            len = xEnd*xEnd + yEnd*yEnd;
            if(len > dmax){
                dmax = len;
                imax = i;
            }
        }
    }

    // Check threshold and divide
    if(dmax >= threshold){
        DouglasPeucker2DRecursive(indices, points, threshold, idxStart, imax);
        indices.push_back(imax);
        DouglasPeucker2DRecursive(indices, points, threshold, imax, idxEnd);
    }
}


} /* namespace: details */


/**
 * @brief Run the douglas-peucker line simplification algorithm for a sequential list of 2-dimensional points.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
 * @param[in] points Reference to a container of 2-dimensional points that represent the input line to be simplified.
 * @param[in] threshold The maximum error to be allowed for line simplification. This value must be positive.
 * @return The output container that will contain the indices to those points indicating the simplified line. This container is empty if there're less than two input points.
 */
template <class T> inline std::vector<size_t> DouglasPeucker2D(const std::vector<T>& points, double threshold) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {points} must be of type std::vector<std::array<double,N>> with N >= 2!");
    std::vector<size_t> indices;
    size_t numPoints = points.size();
    if(numPoints > 1){
        indices.push_back(0);
        mpsv::geometry::details::DouglasPeucker2DRecursive(indices, points, std::fabs(threshold), 0, numPoints - 1);
        indices.push_back(numPoints - 1);
    }
    return indices;
}


/**
 * @brief Run the douglas-peucker line simplification algorithm for a sequential list of poses {x,y,psi}.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] poses Reference to a container of poses that represent the input line to be simplified. A pose type must be at least of dimension 3.
 * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. This value must be positive.
 * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. This value must be positive.
 * @return The output container that will contain the indices to those poses indicating the simplified line. This container is empty if there're less than two input poses.
 * @details As long as the 2D position error is larger than the threshold, the algorithm behaves equal to the @ref DouglasPeucker2D algorithm. If the position error of a simplified line
 * segment is smaller than the specified @ref thresholdPosition, then it is checked whether the angle error stays under the specified threshold. If not, the line segment is further divided.
 */
template <class T> inline std::vector<size_t> DouglasPeuckerSE2(const std::vector<T>& poses, double thresholdPosition, double thresholdAngle) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {poses} must be of type std::vector<std::array<double,N>> with N >= 3!");
    std::vector<size_t> indices;
    size_t numPoses = poses.size();
    if(numPoses > 1){
        indices.push_back(0);
        mpsv::geometry::details::DouglasPeuckerSE2Recursive(indices, poses, std::fabs(thresholdPosition), std::fabs(thresholdAngle), 0, numPoses - 1);
        indices.push_back(numPoses - 1);
    }
    return indices;
}


/**
 * @brief Simplify the poses of a path by breaking the path into linear pose segments.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] path The input path to be simplified. A pose type must be at least of dimension 3.
 * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
 * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. MAKE SURE THAT THIS VALUE IS POSITIVE!
 * @return The list of poses that represent the simplified path. Each pose is given by {x,y,psi}. The output may be empty or may contain only one element.
 * @details The douglas-peucker algorithm for SE2 is used to simplify the poses of the path using the two given threshold parameters.
 */
template <class T> inline std::vector<std::array<double,3>> PoseSimplification(const std::vector<T>& path, double thresholdPosition, double thresholdAngle) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {path} must be of type std::vector<std::array<double,N>> with N >= 3!");
    std::vector<std::array<double,3>> result;

    // Simplify poses using the douglas-peucker algorithm
    std::vector<size_t> indices = mpsv::geometry::DouglasPeuckerSE2(path, thresholdPosition, thresholdAngle);

    // If there are no indices, then the input may contain less than two points
    if(indices.empty()){
        if(!path.empty()){
            result.push_back({path[0][0], path[0][1], path[0][2]});
        }
        return result;
    }

    // Otherwise copy poses to output
    result.reserve(indices.size());
    for(size_t n = 0; n < indices.size(); ++n){
        result.push_back({path[indices[n]][0],path[indices[n]][1],path[indices[n]][2]});
    }
    return result;
}


} /* namespace: geometry */


} /* namespace: mpsv */

