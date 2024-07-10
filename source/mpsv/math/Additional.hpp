#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


namespace math {


/**
 * @brief Compute the symmetrical angle from a given angle.
 * @param[in] d The input angle in radians.
 * @return The symmetrical angle in radians in range [-pi, +pi).
 */
inline double SymmetricalAngle(double d) noexcept {
    d -= 6.28318530717959 * std::trunc(d * 0.159154943091895);
    return (d + 6.28318530717959 * (static_cast<double>(d < -3.14159265358979) - static_cast<double>(d >= 3.14159265358979)));
}


/**
 * @brief Convert an angle given in degrees to an angle in radians.
 * @param[in] angle Input angle in degrees.
 * @return Output angle in radians.
 */
inline double deg2rad(double angle) noexcept {
    return 0.0174532925199433 * angle;
}


/**
 * @brief Signum function for datatype double.
 * @param[in] value Input value for which to calculate the signum function.
 * @return -1.0 if value is less than zero, +1.0 if value is greater than zero, 0.0 otherwise.
 */
inline double signd(double value) noexcept {
    return static_cast<double>(static_cast<int32_t>(0.0 < value) - static_cast<int32_t>(value < 0.0));
}


/**
 * @brief Signum function for datatype double with integer return value.
 * @param[in] value Input value for which to calculate the signum function.
 * @return -1 if value is less than zero, +1 if value is greater than zero, 0 otherwise.
 */
inline int32_t signi(double value) noexcept {
    return (static_cast<int32_t>(0.0 < value) - static_cast<int32_t>(value < 0.0));
}


/**
 * @brief Calculate the inverse of a 3-by-3 matrix.
 * @param[in] matA 3-by-3 input matrix in row-major order.
 * @return <0> True if success, false otherwise. The inversion fails, if the inverse is not finite.
 * @return <1> Resulting 3-by-3 matrix in row-major order.
 */
inline std::tuple<bool,std::array<double,9>> MatrixInverse3x3(const std::array<double,9>& matA) noexcept {
    std::tuple<bool,std::array<double,9>> result;
    bool& success = std::get<0>(result);
    std::array<double,9>& matInvA = std::get<1>(result);
    std::array<double,9> x = matA;
    int32_t p1 = 0;
    int32_t p2 = 3;
    int32_t p3 = 6;
    int32_t itmp;
    double absx11 = std::fabs(matA[0]);
    double absx21 = std::fabs(matA[3]);
    double absx31 = std::fabs(matA[6]);
    if((absx21 > absx11) && (absx21 > absx31)){
        p1 = 3;
        p2 = 0;
        x[0] = matA[3];
        x[1] = matA[0];
        x[3] = matA[4];
        x[4] = matA[1];
        x[6] = matA[5];
        x[7] = matA[2];
    }
    else if(absx31 > absx11){
        p1 = 6;
        p3 = 0;
        x[0] = matA[6];
        x[2] = matA[0];
        x[3] = matA[7];
        x[5] = matA[1];
        x[6] = matA[8];
        x[8] = matA[2];
    }
    x[1] /= x[0];
    x[2] /= x[0];
    x[4] -= x[1] * x[3];
    x[5] -= x[2] * x[3];
    x[7] -= x[1] * x[6];
    x[8] -= x[2] * x[6];
    if(std::fabs(x[5]) > std::fabs(x[4])){
        itmp = p2;
        p2 = p3;
        p3 = itmp;
        absx11 = x[1];
        x[1] = x[2];
        x[2] = absx11;
        absx11 = x[4];
        x[4] = x[5];
        x[5] = absx11;
        absx11 = x[7];
        x[7] = x[8];
        x[8] = absx11;
    }
    x[5] /= x[4];
    x[8] -= x[5] * x[7];
    absx11 = (x[1] * x[5] - x[2]) / x[8];
    absx21 = -(x[7] * absx11 + x[1]) / x[4];
    matInvA[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
    matInvA[p1 + 1] = absx21;
    matInvA[p1 + 2] = absx11;
    absx11 = -x[5] / x[8];
    absx21 = (1.0 - x[7] * absx11) / x[4];
    matInvA[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
    matInvA[p2 + 1] = absx21;
    matInvA[p2 + 2] = absx11;
    absx11 = 1.0 / x[8];
    absx21 = -x[7] * absx11 / x[4];
    matInvA[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
    matInvA[p3 + 1] = absx21;
    matInvA[p3 + 2] = absx11;
    success = std::isfinite(matInvA[0]) && std::isfinite(matInvA[1]) && std::isfinite(matInvA[2]) && std::isfinite(matInvA[3]) && std::isfinite(matInvA[4]) && std::isfinite(matInvA[5]) && std::isfinite(matInvA[6]) && std::isfinite(matInvA[7]) && std::isfinite(matInvA[8]);
    if(!success){
        matInvA.fill(0.0);
    }
    return result;
}


/**
 * @brief Calculate the pose difference (a - b) between two poses a and b by taking the symmetrical angle range into account.
 * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
 * @tparam U Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] a First pose.
 * @param[in] b Second pose.
 * @return Difference pose (a - b) where the difference angle is ensured to be in a symmetrical range.
 */
template <class T, class U> inline std::array<double,3> PoseDifference(const T& a, const U& b) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {a} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {b} must be of type std::array<double,N> with N >= 3!");
    std::array<double,3> result;
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = mpsv::math::SymmetricalAngle(a[2] - b[2]);
    return result;
}


} /* namespace: math */


} /* namespace: mpsv */

