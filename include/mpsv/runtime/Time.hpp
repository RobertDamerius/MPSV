#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace runtime {


/**
 * @brief Get the UTC timestamp which is defined to be the current second of the day (UTC, no leap seconds).
 * @return UTC timestamp: seconds of the day.
 */
[[nodiscard]] inline double GetTimestampUTC(void) noexcept {
    const auto timePoint = std::chrono::system_clock::now();
    const auto secondTimePoint = std::chrono::floor<std::chrono::seconds>(timePoint);
    const std::time_t time = std::chrono::system_clock::to_time_t(secondTimePoint);
    const std::tm* gmTime = std::gmtime(&time); //#TODO: use std::gmtime_r for thread-safety when C++26 is fully supported
    if(!gmTime){
        return 0.0;
    }
    const double fractionalSeconds = 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint - secondTimePoint).count());
    return 3600.0 * static_cast<double>(gmTime->tm_hour) + 60.0 * static_cast<double>(gmTime->tm_min) + static_cast<double>(gmTime->tm_sec) + fractionalSeconds;
}


/**
 * @brief Add two UTC timestamps.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Time in seconds to be added to the first UTC timestamp. Note that this value must be positive!
 * @return Result of (a + b). The result is in range [0, 86400).
 */
[[nodiscard]] inline double AddTimestampUTC(double a, double b) noexcept {
    return std::fmod(a + b, 86400.0);
}


/**
 * @brief Substract a UTC timestamp from another.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Second UTC timestamp in range [0, 86400).
 * @return Result of (a - b). The result is in range [0, 86400).
 */
[[nodiscard]] inline double SubstractTimestampUTC(double a, double b) noexcept {
    return std::fmod(a - b + 86400.0, 86400.0);
}


/**
 * @brief Calculate the time difference between two UTC timestamps.
 * @param[in] a First UTC timestamp in range [0, 86400).
 * @param[in] b Second UTC timestamp in range [0, 86400).
 * @return Result of (a - b) where the time difference is in range [-43200, 43200).
 */
[[nodiscard]] inline double DifferenceTimestampUTC(double a, double b) noexcept {
    double dt = std::fmod(a - b + 86400.0, 86400.0);
    return (dt >= 43200.0) ? (dt - 86400.0) : dt;
}


} /* namespace: runtime */


} /* namespace: mpsv */
