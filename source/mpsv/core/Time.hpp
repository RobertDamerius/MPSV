#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


namespace core {


/**
 * @brief Get the UTC timestamp which is defined to be the current second of the day (UTC).
 * @return UTC timestamp: seconds of the day.
 */
inline double GetTimestampUTC(void) noexcept {
    auto timePoint = std::chrono::high_resolution_clock::now();
    std::time_t time = std::chrono::high_resolution_clock::to_time_t(timePoint);
    std::tm* gmTime = std::gmtime(&time);
    auto duration = timePoint.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    duration -= seconds;
    return 3600.0 * static_cast<double>(gmTime->tm_hour) + 60.0 * static_cast<double>(gmTime->tm_min) + static_cast<double>(gmTime->tm_sec) + 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}


/**
 * @brief Add two UTC timestamps.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Time in seconds to be added to the first UTC timestamp. Note that this value must be positive!
 * @return Result of (a + b). The result is in range [0, 86400).
 */
inline double AddTimestampUTC(double a, double b) noexcept {
    return std::fmod(a + b, 86400.0);
}


/**
 * @brief Substract a UTC timestamp from another.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Second UTC timestamp in range [0, 86400).
 * @return Result of (a - b). The result is in range [0, 86400).
 */
inline double SubstractTimestampUTC(double a, double b) noexcept {
    return std::fmod(a - b + 86400.0, 86400.0);
}


/**
 * @brief Calculate the time difference between two UTC timestamps.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Second UTC timestamp in range [0, 86400).
 * @return Result of (a - b) where the time difference is in range [-43200, 43200).
 */
inline double DifferenceTimestampUTC(double a, double b) noexcept {
    double dt = std::fmod(a - b + 86400.0, 86400.0);
    return (dt >= 43200.0) ? (dt - 86400.0) : dt;
}


} /* namespace: core */


} /* namespace: mpsv */

