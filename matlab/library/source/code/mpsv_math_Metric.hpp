#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_math_Additional.hpp>


namespace mpsv {


namespace math {


/**
 * @brief Compute the distance metric between two poses in SE2.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] a The first pose to be used to compute the distance metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] b The second pose to be used to compute the distance metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] weightPsi Scalar weighting factor for psi, defaults to 1.0.
 * @return The distance metric between the two given poses, calculated by sqrt(dx^2 + dy^2 + (weightPsi*dpsi)^2).
 */
template <class T, class U> inline double DistanceMetricSE2(const T& a, const U& b, const double weightPsi = 1.0) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {a} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {b} must be of type std::array<double,N> with N >= 3!");
    double dx = b[0] - a[0];
    double dy = b[1] - a[1];
    double dpsi = weightPsi * mpsv::math::SymmetricalAngle(b[2] - a[2]);
    return std::sqrt(dx*dx + dy*dy + dpsi*dpsi);
}


/**
 * @brief The metric function that weights the squared sine of the angular displacement of the vehicles heading from the direction of the line along the line, e.g. the sway movement.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] a The first pose to be used to compute the sway metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] b The second pose to be used to compute the sway metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] weight A scalar value that weights the resulting sway metric.
 * @return The sway metric between the two given poses, calculated by integral of sin^2(psi0 + (s/L)*(psiL - psi0) - alpha) from s = 0 to L, with alpha
 * being the angle of the line and psi0 and psiL being the yaw angle of pose a and b, respectively. The result is scaled with the given weight.
 */
template <class T, class U> inline double SwayMetric(const T& a, const U& b, double weight) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {a} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {b} must be of type std::array<double,N> with N >= 3!");
    constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
    double dx = b[0] - a[0];
    double dy = b[1] - a[1];
    double L = std::sqrt(dx*dx + dy*dy);
    double alpha = std::atan2(dy,dx);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(alpha)){
        alpha = 0.0;
    }
    #endif
    double psiL = a[2] + mpsv::math::SymmetricalAngle(b[2] - a[2]);
    double d = a[2] - psiL;
    if(std::fabs(d) <= eps){
        double s = std::sin(a[2] - alpha);
        return weight * s * s * L;
    }
    return weight * std::fabs((L * (2.0 * d + std::sin(2.0 * (alpha - a[2])) - std::sin(2.0 * (alpha - psiL)))) / (4.0 * d));
}


} /* namespace: math */


} /* namespace: mpsv */

