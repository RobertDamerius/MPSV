#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_math_Additional.hpp>


namespace mpsv {


namespace math {


namespace wgs84 {


/**
 * @brief Convert LLA coordinates (WGS84) to local NED frame coordinates. This implementation uses the algorithm proposed by
 *     S. P. Drake
 *     Converting GPS Coordinates to Navigation Coordinates
 *     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
 *     April, 2002
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam U This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] LLA Latitude (rad), longitude (rad) and altitude (m) of the position to be converted. The altitude is defined to be positive upwards.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return North, east and down value of the NED position in meters.
 */
template <class T, class U> inline std::array<double,3> LLA2NED(const T LLA, const U originLLA) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {LLA} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double,3> NED;

    // Constants according to WGS84
    constexpr double a = 6378137.0;
    constexpr double e2 = 0.00669437999014132;

    // Delta values
    double dphi = LLA[0] - originLLA[0];
    double dlam = mpsv::math::SymmetricalAngle(LLA[1] - originLLA[1]);
    double dh = LLA[2] - originLLA[2];

    // Some useful definitions
    double cp = std::cos(originLLA[0]);
    double sp = std::sin(originLLA[0]);
    double sp2 = sp*sp;
    double spcp_half = 0.5*sp*cp;
    double cpcp_half = 0.5*cp*cp;
    double tmp = std::sqrt(1.0 - e2*sp2);
    double a_over_tmp = a / tmp;
    double a_over_tmp3 = a / (tmp*tmp*tmp);
    double dlam2 = dlam*dlam;
    double dhpi2 = dphi*dphi;

    // Final transformation
    NED[0] = ((1.0 - e2)*a_over_tmp3 + originLLA[2])*dphi + 3.0*spcp_half*a*e2*dhpi2 + sp2*dh*dphi + spcp_half*(a_over_tmp + originLLA[2])*dlam2;
    NED[1] = (a_over_tmp + originLLA[2])*cp*dlam - ((1.0 - e2)*a_over_tmp3 + originLLA[2])*sp*dphi*dlam + cp*dlam*dh;
    NED[2] = -(dh - 0.5*(a - 3.0*a*e2*cpcp_half + 0.5*a*e2 + originLLA[2])*dhpi2 - cpcp_half*(a_over_tmp - originLLA[2])*dlam2);
    return NED;
}


/**
 * @brief Convert LLA coordinates (WGS84) to local NE frame coordinates. This implementation uses the algorithm proposed by
 *     S. P. Drake
 *     Converting GPS Coordinates to Navigation Coordinates
 *     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
 *     April, 2002
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam U This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] LLA Latitude (rad), longitude (rad) and altitude (m) of the position to be converted. The altitude is defined to be positive upwards.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return North and east value of the NE position in meters.
 */
template <class T, class U> inline std::array<double,2> LLA2NE(const T LLA, const U originLLA) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {LLA} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double,2> NE;

    // Constants according to WGS84
    constexpr double a = 6378137.0;
    constexpr double e2 = 0.00669437999014132;

    // Delta values
    double dphi = LLA[0] - originLLA[0];
    double dlam = mpsv::math::SymmetricalAngle(LLA[1] - originLLA[1]);
    double dh = LLA[2] - originLLA[2];

    // Some useful definitions
    double cp = std::cos(originLLA[0]);
    double sp = std::sin(originLLA[0]);
    double sp2 = sp*sp;
    double spcp_half = 0.5*sp*cp;
    double tmp = std::sqrt(1.0 - e2*sp2);
    double a_over_tmp = a / tmp;
    double a_over_tmp3 = a / (tmp*tmp*tmp);

    // Final transformation
    NE[0] = ((1.0 - e2)*a_over_tmp3 + originLLA[2])*dphi + 3.0*spcp_half*a*e2*dphi*dphi + sp2*dh*dphi + spcp_half*(a_over_tmp + originLLA[2])*dlam*dlam;
    NE[1] = (a_over_tmp + originLLA[2])*cp*dlam - ((1.0 - e2)*a_over_tmp3 + originLLA[2])*sp*dphi*dlam + cp*dlam*dh;
    return NE;
}


/**
 * @brief Convert local NED frame coordinates to LLA coordinates (WGS84). The down position is assumed to be zero. The output altitude is not computed.
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 1.
 * @tparam U This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] NE North and east position in meters to be converted.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return Latitude and longitude in radians of the converted position.
 */
template <class T, class U> inline std::array<double,2> NE2LL(const T NE, const U originLLA) noexcept {
    static_assert(mpsv::is_vec2<T>::value,"Argument {NE} must be of type std::array<double,N> with N >= 2!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double,2> LL;

    // Note: some calculation has been removed from the conversion due to the assumption D = 0
    double slat = std::sin(originLLA[0]);
    double clat = std::cos(originLLA[0]);
    double slon = std::sin(originLLA[1]);
    double clon = std::cos(originLLA[1]);
    double slatPow2 = slat * slat;
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double b = (1.0 - f) * a;
    constexpr double e2 = f * (2.0 - f);
    constexpr double ep2 = e2 / (1.0 - e2);
    constexpr double ae2 = a * e2;
    double Nval  = a / std::sqrt(1.0 - e2 * slatPow2);
    double rho = (Nval + originLLA[2]) * clat;
    double x0 = rho * clon;
    double y0 = rho * slon;
    double z0 = (Nval*(1.0 - e2) + originLLA[2]) * slat;

    // Longitude
    double t = -slat * NE[0];
    double x = x0 + clon * t - slon * NE[1];
    double y = y0 + slon * t + clon * NE[1];
    double z = z0 + clat * NE[0];
    LL[1] = std::atan2(y,x);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(LL[1])){
        LL[1] = 0.0;
    }
    #endif

    // Latitude: fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
    rho = std::hypot(x, y);
    double beta = std::atan2(z, (1.0 - f) * rho);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(beta)){
        beta = 0.0;
    }
    #endif
    double sbeta = std::sin(beta);
    double cbeta = std::cos(beta);
    LL[0] = std::atan2(z + b * ep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(LL[0])){
        LL[0] = 0.0;
    }
    #endif
    for(int i = 0; i < 5; ++i){
        beta = std::atan2((1.0 - f)*std::sin(LL[0]), std::cos(LL[0]));
        #ifdef MPSV_SAFE_ATAN2
        if(!std::isfinite(beta)){
            beta = 0.0;
        }
        #endif
        sbeta = std::sin(beta);
        cbeta = std::cos(beta);
        LL[0] = std::atan2(z + b * ep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
        #ifdef MPSV_SAFE_ATAN2
        if(!std::isfinite(LL[0])){
            LL[0] = 0.0;
        }
        #endif
    }
    return LL;
}


/**
 * @brief Convert local NED frame coordinates to LLA coordinates (WGS84).
 * @tparam T This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @tparam U This template represents the type of a point and must be of type std::array<double,N> with N being greater than 2.
 * @param[in] NED North, east and down position in meters to be converted.
 * @param[in] originLLA Latitude (rad), longitude (rad) and altitude (m) that indicate the geographic origin for conversion. The altitude is defined to be positive upwards.
 * @return Latitude and longitude in radians and altitude in meters of the converted position.
 */
template <class T, class U> inline std::array<double,3> NED2LLA(const T NED, const U originLLA) noexcept {
    static_assert(mpsv::is_vec3<T>::value,"Argument {NE} must be of type std::array<double,N> with N >= 3!");
    static_assert(mpsv::is_vec3<U>::value,"Argument {originLLA} must be of type std::array<double,N> with N >= 3!");
    std::array<double,3> LLA;

    // Constants and spheroid properties
    constexpr double a = 6378137.0;
    constexpr double oneMinusF = 0.996647189335253;
    constexpr double e2 = 0.00669437999014132;
    constexpr double ae2 = 42697.67270718;
    constexpr double bep2 = 42841.3115133136;

    // ECEF: Computation of (x,y,z) = (x0,y0,z0) + (dx,dy,dz)
    double slat = std::sin(originLLA[0]);
    double clat = std::cos(originLLA[0]);
    double slon = std::sin(originLLA[1]);
    double clon = std::cos(originLLA[1]);
    double Nval = a / std::sqrt(1.0 - e2 * slat*slat);
    double rho = (Nval + originLLA[2]) * clat;
    double x0 = rho * clon;
    double y0 = rho * slon;
    double z0 = (Nval*(1.0 - e2) + originLLA[2]) * slat;
    double t = -clat * NED[2] - slat * NED[0];
    double dz = -slat * NED[2] + clat * NED[0];
    double dx = clon * t - slon * NED[1];
    double dy = slon * t + clon * NED[1];
    double x = x0 + dx;
    double y = y0 + dy;
    double z = z0 + dz;
    LLA[1] = std::atan2(y,x);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(LLA[1])){
        LLA[1] = 0.0;
    }
    #endif

    // Latitude: fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
    rho = std::hypot(x, y);
    double beta = std::atan2(z, oneMinusF * rho);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(beta)){
        beta = 0.0;
    }
    #endif
    double sbeta = std::sin(beta);
    double cbeta = std::cos(beta);
    LLA[0] = std::atan2(z + bep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
    #ifdef MPSV_SAFE_ATAN2
    if(!std::isfinite(LLA[0])){
        LLA[0] = 0.0;
    }
    #endif
    for(int i = 0; i < 5; ++i){
        beta = std::atan2(oneMinusF*std::sin(LLA[0]), std::cos(LLA[0]));
        #ifdef MPSV_SAFE_ATAN2
        if(!std::isfinite(beta)){
            beta = 0.0;
        }
        #endif
        sbeta = std::sin(beta);
        cbeta = std::cos(beta);
        LLA[0] = std::atan2(z + bep2 * sbeta*sbeta*sbeta, rho - ae2 * cbeta*cbeta*cbeta);
        #ifdef MPSV_SAFE_ATAN2
        if(!std::isfinite(LLA[0])){
            LLA[0] = 0.0;
        }
        #endif
    }

    // Ellipsoidal height from final value for latitude
    slat = std::sin(LLA[0]);
    Nval = a / std::sqrt(1.0 - e2 * slat * slat);
    LLA[2] = rho * std::cos(LLA[0]) + (z + e2 * Nval * slat) * slat - Nval;
    return LLA;
}


} /* namespace: wgs84 */


} /* namespace: math */


} /* namespace: mpsv */

