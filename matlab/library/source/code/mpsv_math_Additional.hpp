#pragma once


#include <mpsv_core_MPSVCommon.hpp>


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
 * @return <0> True if success, false otherwise. The inversion fails, if the determinant of A is close to zero.
 * @return <1> Resulting 3-by-3 matrix in row-major order.
 */
inline std::tuple<bool,std::array<double,9>> MatrixInverse3x3(const std::array<double,9>& matA) noexcept {
    std::tuple<bool,std::array<double,9>> result;
    bool& success = std::get<0>(result);
    std::array<double,9>& matInvA = std::get<1>(result);
    constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
    double detA = matA[0]*matA[4]*matA[8] + matA[1]*matA[5]*matA[6] + matA[2]*matA[3]*matA[7] - matA[0]*matA[5]*matA[7] - matA[1]*matA[3]*matA[8]- matA[2]*matA[4]*matA[6];
    if((success = (std::fabs(detA) > eps))){
        matInvA[0] = (matA[4]*matA[8] - matA[5]*matA[7]) / detA;
        matInvA[1] = (matA[2]*matA[7] - matA[1]*matA[8]) / detA;
        matInvA[2] = (matA[1]*matA[5] - matA[2]*matA[4]) / detA;
        matInvA[3] = (matA[5]*matA[6] - matA[3]*matA[8]) / detA;
        matInvA[4] = (matA[0]*matA[8] - matA[2]*matA[6]) / detA;
        matInvA[5] = (matA[2]*matA[3] - matA[0]*matA[5]) / detA;
        matInvA[6] = (matA[3]*matA[7] - matA[4]*matA[6]) / detA;
        matInvA[7] = (matA[1]*matA[6] - matA[0]*matA[7]) / detA;
        matInvA[8] = (matA[0]*matA[4] - matA[1]*matA[3]) / detA;
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

