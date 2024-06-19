#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/math/Additional.hpp>


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
 * @brief The metric function that weights the the angular deviation of the vehicles heading from the direction of the line along the line, e.g. the sway and/or reverse movement.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] a The first pose to be used to compute the sway metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] b The second pose to be used to compute the sway metric between two poses. Array must contain at least three elements {x,y,psi}.
 * @param[in] weightSway A scalar value that weights the sway motion via a squared sine function.
 * @param[in] weightReverseScale A scalar value that weights the sway and reverse motion via a squared sigmoid function.
 * @param[in] weightReverseDecay A decay factor of the squared sigmoid function used for the sway metric.
 * @return The sway metric between the two given poses, calculated by two terms. First, the integral of weightSway * sin^2(psi0 + (s/L)*(psiL - psi0) - alpha) from s = 0 to L, with alpha
 * being the angle of the line and psi0 and psiL being the yaw angle of pose a and b, respectively. Second, the integral of weightReverseScale * (weightReverseDecay*e)^2 / (1 + (weightReverseDecay*e)^2)
 * from s = 0 to L with e(s) = psi0 + (s/L)*(psiL - psi0) - alpha.
 */
template <class T, class U> inline double SwayMetric(const T& a, const U& b, double weightSway, double weightReverseScale, double weightReverseDecay) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {a} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {b} must be of type std::array<double,N> with N >= 3!");
    constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
    double dx = b[0] - a[0];
    double dy = b[1] - a[1];
    double dpsi = mpsv::math::SymmetricalAngle(b[2] - a[2]);
    double L = std::sqrt(dx*dx + dy*dy);
    double alpha = std::atan2(dy,dx);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(alpha)){
        alpha = 0.0;
    }
    #endif

    // symmetric sway metric
    double psiL = a[2] + dpsi;
    double d = a[2] - psiL;
    double result = 0.0;
    if(std::fabs(d) <= eps){
        double s = std::sin(a[2] - alpha);
        result += weightSway * s * s * L;
    }
    else{
        result += weightSway * std::fabs((L * (2.0 * d + std::sin(2.0 * (alpha - a[2])) - std::sin(2.0 * (alpha - psiL)))) / (4.0 * d));
    }

    // asymmetric sway metric
    double psiA = mpsv::math::SymmetricalAngle(a[2] - alpha);
    double psiB = mpsv::math::SymmetricalAngle(b[2] - alpha);
    double bpsiA = weightReverseDecay * psiA;
    double bpsiAsq = bpsiA * bpsiA;
    double bpsiB = weightReverseDecay * psiB;
    double bpi = weightReverseDecay * 3.14159265358979323;
    double bpisq = bpi * bpi;
    if(std::fabs(dpsi) < eps){
        result += weightReverseScale*L*bpsiAsq / (1.0 + bpsiAsq);
    }
    else{
        if((psiA + dpsi) >= 3.14159265358979323){
            double delta = 3.14159265358979323 - psiA;
            double L1 = L * delta / dpsi;
            if(std::fabs(delta) < eps){
                result += weightReverseScale*L1*bpsiAsq / (1.0 + bpsiAsq);
            }
            else{
                result += weightReverseScale*L1*(1.0 + (std::atan(bpsiA) - std::atan(bpi))/(weightReverseDecay*delta));
            }
            delta = psiB + 3.14159265358979323;
            if(std::fabs(delta) < eps){
                result += weightReverseScale*(L - L1)*(bpisq / (1.0 + bpisq));
            }
            else{
                result += weightReverseScale*(L - L1)*(1.0 + (std::atan(-bpi) - std::atan(bpsiB))/(weightReverseDecay*delta));
            }
        }
        else if((psiA + dpsi) < -3.14159265358979323){
            double delta = -3.14159265358979323 - psiA;
            double L1 = L * delta / dpsi;
            if(std::fabs(delta) < eps){
                result += weightReverseScale*L1*bpsiAsq / (1.0 + bpsiAsq);
            }
            else{
                result += weightReverseScale*L1*(1.0 + (std::atan(bpsiA) - std::atan(-bpi))/(weightReverseDecay*delta));
            }
            delta = psiB - 3.14159265358979323;
            if(std::fabs(delta) < eps){
                result += weightReverseScale*(L - L1)*(bpisq / (1.0 + bpisq));
            }
            else{
                result += weightReverseScale*(L - L1)*(1.0 + (std::atan(bpi) - std::atan(bpsiB))/(weightReverseDecay*delta));
            }
        }
        else{
            result += weightReverseScale*L*(1.0 + (std::atan(bpsiA) - std::atan(bpsiB))/(weightReverseDecay*dpsi));
        }
    }
    return result;
}


} /* namespace: math */


} /* namespace: mpsv */

