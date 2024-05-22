#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


namespace core {


/**
 * @brief The performance counter class is used to measure the execution time. A steady clock is used.
 */
class PerformanceCounter {
    public:
        /**
         * @brief Construct a new performance counter object and reset the internal time point to 1970-01-01.
         */
        PerformanceCounter() noexcept { Reset(); }

        /**
         * @brief Reset the internal time point to 1970-01-01.
         */
        void Reset(void) noexcept { t0 = std::chrono::time_point<std::chrono::steady_clock>(); }

        /**
         * @brief Start the performance counter by setting the internal timepoint to now.
         */
        void Start(void) noexcept { t0 = std::chrono::steady_clock::now(); }

        /**
         * @brief Get the time (in seconds) to the internal timepoint. If the performance counter is reset, this time
         * corresponds to the seconds to the initial timepoint of 1970-01-01. If @ref Start has been called, this time
         * corresponds to the seconds to the last @ref Start call.
         * @return The time to the internal timepoint in seconds. The internal timepoint is set by @ref Reset or @ref Start.
         */
        double TimeToStart(void) noexcept {
            auto t1 = std::chrono::steady_clock::now();
            return 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
        }

    protected:
        std::chrono::time_point<std::chrono::steady_clock> t0;   // Time point when performance counter was started.
};


} /* namespace: core */


} /* namespace: mpsv */

