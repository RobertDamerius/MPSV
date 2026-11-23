#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace math {


/**
 * @brief Compute the symmetrical angle from a given angle.
 * @param[in] d The input angle in radians.
 * @return The symmetrical angle in radians in range [-pi, +pi).
 */
[[nodiscard]] inline double SymmetricalAngle(double d) noexcept {
    constexpr double pi{std::numbers::pi};
    constexpr double pi2{2.0 * std::numbers::pi};
    constexpr double inv_pi2{std::numbers::inv_pi / 2.0};
    d -= pi2 * std::trunc(d * inv_pi2);
    return (d + pi2 * (static_cast<double>(d < -pi) - static_cast<double>(d >= pi)));
}


/**
 * @brief Convert an angle given in degrees to an angle in radians.
 * @param[in] angle Input angle in degrees.
 * @return Output angle in radians.
 */
[[nodiscard]] constexpr double deg2rad(double angle) noexcept {
    constexpr double pi_over_180{std::numbers::pi / 180.0};
    return pi_over_180 * angle;
}


/**
 * @brief Calculate the inverse of a 3-by-3 matrix.
 * @param[in] matA 3-by-3 input matrix in row-major order.
 * @return <0> True if success, false otherwise. The inversion fails, if the inverse is not finite.
 * @return <1> Resulting 3-by-3 matrix in row-major order.
 */
[[nodiscard]] inline std::tuple<bool, std::array<double, 9U>> MatrixInverse3x3(const std::array<double, 9U>& matA) noexcept {
    std::tuple<bool, std::array<double, 9U>> result;
    bool& success = std::get<0U>(result);
    std::array<double, 9U>& matInvA = std::get<1U>(result);
    std::array<double, 9U> x = matA;
    std::int32_t p1{0};
    std::int32_t p2{3};
    std::int32_t p3{6};
    std::int32_t itmp;
    double absx11{std::fabs(matA[0U])};
    double absx21{std::fabs(matA[3U])};
    double absx31{std::fabs(matA[6U])};
    if ((absx21 > absx11) && (absx21 > absx31)) {
        p1 = 3;
        p2 = 0;
        x[0U] = matA[3U];
        x[1U] = matA[0U];
        x[3U] = matA[4U];
        x[4U] = matA[1U];
        x[6U] = matA[5U];
        x[7U] = matA[2U];
    } else if (absx31 > absx11) {
        p1 = 6;
        p3 = 0;
        x[0U] = matA[6U];
        x[2U] = matA[0U];
        x[3U] = matA[7U];
        x[5U] = matA[1U];
        x[6U] = matA[8U];
        x[8U] = matA[2U];
    }
    x[1U] /= x[0U];
    x[2U] /= x[0U];
    x[4U] -= x[1U] * x[3U];
    x[5U] -= x[2U] * x[3U];
    x[7U] -= x[1U] * x[6U];
    x[8U] -= x[2U] * x[6U];
    if (std::fabs(x[5U]) > std::fabs(x[4U])) {
        itmp = p2;
        p2 = p3;
        p3 = itmp;
        absx11 = x[1U];
        x[1U] = x[2U];
        x[2U] = absx11;
        absx11 = x[4U];
        x[4U] = x[5U];
        x[5U] = absx11;
        absx11 = x[7U];
        x[7U] = x[8U];
        x[8U] = absx11;
    }
    x[5U] /= x[4U];
    x[8U] -= x[5U] * x[7U];
    absx11 = (x[1U] * x[5U] - x[2U]) / x[8U];
    absx21 = -(x[7U] * absx11 + x[1U]) / x[4U];
    matInvA[p1] = ((1.0 - x[3U] * absx21) - x[6U] * absx11) / x[0U];
    matInvA[p1 + 1] = absx21;
    matInvA[p1 + 2] = absx11;
    absx11 = -x[5U] / x[8U];
    absx21 = (1.0 - x[7U] * absx11) / x[4U];
    matInvA[p2] = -(x[3U] * absx21 + x[6U] * absx11) / x[0U];
    matInvA[p2 + 1] = absx21;
    matInvA[p2 + 2] = absx11;
    absx11 = 1.0 / x[8U];
    absx21 = -x[7U] * absx11 / x[4U];
    matInvA[p3] = -(x[3U] * absx21 + x[6U] * absx11) / x[0U];
    matInvA[p3 + 1] = absx21;
    matInvA[p3 + 2] = absx11;
    success = std::isfinite(matInvA[0U]) && std::isfinite(matInvA[1U]) && std::isfinite(matInvA[2U]) && std::isfinite(matInvA[3U]) && std::isfinite(matInvA[4U]) && std::isfinite(matInvA[5U]) && std::isfinite(matInvA[6U]) && std::isfinite(matInvA[7U]) && std::isfinite(matInvA[8U]);
    if (!success) {
        matInvA.fill(0.0);
    }
    return result;
}


/**
 * @brief Calculate the pose difference (a - b) between two poses a and b by taking the symmetrical angle range into account.
 * @tparam TposeA Template parameter must be of type std::array<double,N> with N >= 3.
 * @tparam TposeB Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] a First pose.
 * @param[in] b Second pose.
 * @return Difference pose (a - b) where the difference angle is ensured to be in a symmetrical range.
 */
template <class TposeA, class TposeB> [[nodiscard]] inline std::array<double, 3U> PoseDifference(const TposeA& a, const TposeB& b) noexcept {
    static_assert(mpsv::is_vec3<TposeA>::value, "Argument {a} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<TposeB>::value, "Argument {b} must be of type std::array<double,N> with N >= 3!");
    std::array<double, 3U> result;
    result[0U] = a[0U] - b[0U];
    result[1U] = a[1U] - b[1U];
    result[2U] = mpsv::math::SymmetricalAngle(a[2U] - b[2U]);
    return result;
}


} /* namespace: math */


} /* namespace: mpsv */
