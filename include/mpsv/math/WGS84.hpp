#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv::math::wgs84 {


/**
 * @brief Convert LLA coordinates (WGS84) to local NED frame coordinates. This implementation uses the algorithm proposed by
 *     S. P. Drake
 *     Converting GPS Coordinates to Navigation Coordinates
 *     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
 *     April, 2002
 * @tparam TLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam ToriginLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] LLA Latitude (rad), longitude (rad) and altitude (m) of the position to be converted. The altitude is defined to be positive upwards.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return North, east and down value of the NED position in meters.
 */
template <class TLLA, class ToriginLLA> [[nodiscard]] inline std::array<double, 3U> LLA2NED(const TLLA LLA, const ToriginLLA originLLA) noexcept {
    static_assert(mpsv::is_vec3<TLLA>::value, "Argument {LLA} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<ToriginLLA>::value, "Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double, 3U> NED;

    // Constants according to WGS84
    constexpr double a{6378137.0};
    constexpr double e2{0.00669437999014132};

    // Delta values
    double dphi{LLA[0U] - originLLA[0U]};
    double dlam{mpsv::math::SymmetricalAngle(LLA[1U] - originLLA[1U])};
    double dh{LLA[2U] - originLLA[2U]};

    // Some useful definitions
    double cp{std::cos(originLLA[0U])};
    double sp{std::sin(originLLA[0U])};
    double sp2{sp*sp};
    double spcp_half{0.5*sp*cp};
    double cpcp_half{0.5*cp*cp};
    double tmp{std::sqrt(1.0 - e2*sp2)};
    double a_over_tmp{a / tmp};
    double a_over_tmp3{a / (tmp*tmp*tmp)};
    double dlam2{dlam*dlam};
    double dhpi2{dphi*dphi};

    // Final transformation
    NED[0U] = ((1.0 - e2)*a_over_tmp3 + originLLA[2U])*dphi + 3.0*spcp_half*a*e2*dhpi2 + sp2*dh*dphi + spcp_half*(a_over_tmp + originLLA[2U])*dlam2;
    NED[1U] = (a_over_tmp + originLLA[2U])*cp*dlam - ((1.0 - e2)*a_over_tmp3 + originLLA[2U])*sp*dphi*dlam + cp*dlam*dh;
    NED[2U] = -(dh - 0.5*(a - 3.0*a*e2*cpcp_half + 0.5*a*e2 + originLLA[2U])*dhpi2 - cpcp_half*(a_over_tmp - originLLA[2U])*dlam2);
    return NED;
}


/**
 * @brief Convert LLA coordinates (WGS84) to local NE frame coordinates. This implementation uses the algorithm proposed by
 *     S. P. Drake
 *     Converting GPS Coordinates to Navigation Coordinates
 *     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
 *     April, 2002
 * @tparam TLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam ToriginLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] LLA Latitude (rad), longitude (rad) and altitude (m) of the position to be converted. The altitude is defined to be positive upwards.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return North and east value of the NE position in meters.
 */
template <class TLLA, class ToriginLLA> [[nodiscard]] inline std::array<double, 2U> LLA2NE(const TLLA LLA, const ToriginLLA originLLA) noexcept {
    static_assert(mpsv::is_vec3<TLLA>::value, "Argument {LLA} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<ToriginLLA>::value, "Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double, 2U> NE;

    // Constants according to WGS84
    constexpr double a{6378137.0};
    constexpr double e2{0.00669437999014132};

    // Delta values
    double dphi{LLA[0U] - originLLA[0U]};
    double dlam{mpsv::math::SymmetricalAngle(LLA[1U] - originLLA[1U])};
    double dh{LLA[2U] - originLLA[2U]};

    // Some useful definitions
    double cp{std::cos(originLLA[0U])};
    double sp{std::sin(originLLA[0U])};
    double sp2{sp*sp};
    double spcp_half{0.5*sp*cp};
    double tmp{std::sqrt(1.0 - e2*sp2)};
    double a_over_tmp{a / tmp};
    double a_over_tmp3{a / (tmp*tmp*tmp)};

    // Final transformation
    NE[0U] = ((1.0 - e2)*a_over_tmp3 + originLLA[2U])*dphi + 3.0*spcp_half*a*e2*dphi*dphi + sp2*dh*dphi + spcp_half*(a_over_tmp + originLLA[2U])*dlam*dlam;
    NE[1U] = (a_over_tmp + originLLA[2U])*cp*dlam - ((1.0 - e2)*a_over_tmp3 + originLLA[2U])*sp*dphi*dlam + cp*dlam*dh;
    return NE;
}


/**
 * @brief Convert local NED frame coordinates to LLA coordinates (WGS84). The down position is assumed to be zero. The output altitude is not computed.
 * @tparam TNE This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @tparam ToriginLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] NE North and east position in meters to be converted.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return Latitude and longitude in radians of the converted position.
 */
template <class TNE, class ToriginLLA> [[nodiscard]] inline std::array<double, 2U> NE2LL(const TNE NE, const ToriginLLA originLLA) noexcept {
    static_assert(mpsv::is_vec2<TNE>::value, "Argument {NE} must be of type std::array<double,N> with N >= 2!");
    static_assert(mpsv::is_vec3<ToriginLLA>::value, "Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double, 2U> LL;

    // Note: some calculation has been removed from the conversion due to the assumption D = 0
    double slat{std::sin(originLLA[0U])};
    double clat{std::cos(originLLA[0U])};
    double slon{std::sin(originLLA[1U])};
    double clon{std::cos(originLLA[1U])};
    double slatPow2{slat * slat};
    constexpr double a{6378137.0};
    constexpr double f{1.0 / 298.257223563};
    constexpr double b{(1.0 - f) * a};
    constexpr double e2{f * (2.0 - f)};
    constexpr double ep2{e2 / (1.0 - e2)};
    constexpr double ae2{a * e2};
    double Nval{a / std::sqrt(1.0 - e2 * slatPow2)};
    double rho{(Nval + originLLA[2U]) * clat};
    double x0{rho * clon};
    double y0{rho * slon};
    double z0{(Nval*(1.0 - e2) + originLLA[2U]) * slat};

    // Longitude
    double t{-slat * NE[0U]};
    double x{x0 + clon * t - slon * NE[1U]};
    double y{y0 + slon * t + clon * NE[1U]};
    double z{z0 + clat * NE[0U]};
    LL[1U] = std::atan2(y, x);

    // Latitude: fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
    rho = std::hypot(x, y);
    double beta{std::atan2(z, (1.0 - f) * rho)};
    double sbeta{std::sin(beta)};
    double cbeta{std::cos(beta)};
    LL[0U] = std::atan2(z + b * ep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    for (int i{0}; i < 5; ++i) {
        beta = std::atan2((1.0 - f)*std::sin(LL[0U]), std::cos(LL[0U]));
        sbeta = std::sin(beta);
        cbeta = std::cos(beta);
        LL[0U] = std::atan2(z + b * ep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    }
    return LL;
}


/**
 * @brief Convert local NED frame coordinates to LLA coordinates (WGS84).
 * @tparam TNED This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam ToriginLLA This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] NED North, east and down position in meters to be converted.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return Latitude and longitude in radians and altitude in meters of the converted position.
 */
template <class TNED, class ToriginLLA> [[nodiscard]] inline std::array<double, 3U> NED2LLA(const TNED NED, const ToriginLLA originLLA) noexcept {
    static_assert(mpsv::is_vec3<TNED>::value, "Argument {NE} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<ToriginLLA>::value, "Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double, 3U> LLA;

    // Constants and spheroid properties
    constexpr double a{6378137.0};
    constexpr double oneMinusF{0.996647189335253};
    constexpr double e2{0.00669437999014132};
    constexpr double ae2{42697.67270718};
    constexpr double bep2{42841.3115133136};

    // ECEF: Computation of (x,y,z) = (x0,y0,z0) + (dx,dy,dz)
    double slat{std::sin(originLLA[0U])};
    double clat{std::cos(originLLA[0U])};
    double slon{std::sin(originLLA[1U])};
    double clon{std::cos(originLLA[1U])};
    double Nval{a / std::sqrt(1.0 - e2 * slat*slat)};
    double rho{(Nval + originLLA[2U]) * clat};
    double x0{rho * clon};
    double y0{rho * slon};
    double z0{(Nval*(1.0 - e2) + originLLA[2U]) * slat};
    double t{-clat * NED[2U] - slat * NED[0U]};
    double dz{-slat * NED[2U] + clat * NED[0U]};
    double dx{clon * t - slon * NED[1U]};
    double dy{slon * t + clon * NED[1U]};
    double x{x0 + dx};
    double y{y0 + dy};
    double z{z0 + dz};
    LLA[1U] = std::atan2(y, x);

    // Latitude: fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
    rho = std::hypot(x, y);
    double beta{std::atan2(z, oneMinusF * rho)};
    double sbeta{std::sin(beta)};
    double cbeta{std::cos(beta)};
    LLA[0U] = std::atan2(z + bep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    for (int i{0}; i < 5; ++i) {
        beta = std::atan2(oneMinusF*std::sin(LLA[0U]), std::cos(LLA[0U]));
        sbeta = std::sin(beta);
        cbeta = std::cos(beta);
        LLA[0U] = std::atan2(z + bep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    }

    // Ellipsoidal height from final value for latitude
    slat = std::sin(LLA[0U]);
    Nval = a / std::sqrt(1.0 - e2 * slat * slat);
    LLA[2U] = rho * std::cos(LLA[0U]) + (z + e2 * Nval * slat) * slat - Nval;
    return LLA;
}


} /* namespace: mpsv::math::wgs84 */
