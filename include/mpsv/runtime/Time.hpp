#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace runtime {


/**
 * @brief Get the UTC timestamp which is defined to be the current second of the day (UTC, no leap seconds).
 * @return UTC timestamp: seconds of the day.
 */
[[nodiscard]] inline double GetTimestampUTC() noexcept {
    namespace chr = std::chrono;

    // capture current wall-clock time since Unix epoch (1970-01-01 00:00:00 UTC)
    const chr::system_clock::time_point timeNow{chr::system_clock::now()};
    const chr::system_clock::duration sinceEpoch{timeNow.time_since_epoch()};

    // extract whole seconds and sub-second remainder
    const chr::seconds totalSeconds{chr::duration_cast<chr::seconds>(sinceEpoch)};
    const chr::system_clock::duration subseconds{sinceEpoch - totalSeconds};

    // compute seconds elapsed since UTC midnight (86400 seconds per UTC day)
    constexpr std::int64_t kSecondsPerDay{86400};
    std::int64_t secondsToday{totalSeconds.count() % kSecondsPerDay};
    if (secondsToday < 0) {
        secondsToday += kSecondsPerDay; // handle pre-1970 timestamps safely
    }

    // combine whole seconds of the day with fractional seconds
    constexpr double kSecondsPerDayDouble{86400.0};
    const chr::duration<double> fractional_seconds{chr::duration_cast<chr::duration<double>>(subseconds)};
    double result{static_cast<double>(secondsToday) + fractional_seconds.count()};
    if (result >= kSecondsPerDayDouble) {
        result = 0.0; // boundary guard: if floating point arithmetic would lead to exactly 86400.0
    }
    return result;
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
    double dt{std::fmod(a - b + 86400.0, 86400.0)};
    return (dt >= 43200.0) ? (dt - 86400.0) : dt;
}


} /* namespace: runtime */


} /* namespace: mpsv */
