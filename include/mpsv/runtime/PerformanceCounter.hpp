#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace runtime {


/**
 * @brief The performance counter class is used to measure the execution time. A steady clock is used.
 */
class PerformanceCounter {
    public:
        /**
         * @brief Construct a new performance counter object.
         * @param[in] start If true, set the internal time point to now. If false, set it to the default-constructed steady-clock time point. The default value is true.
         */
        explicit PerformanceCounter(bool start = true) noexcept { m_t0 = start ? std::chrono::steady_clock::now() : std::chrono::time_point<std::chrono::steady_clock>(); }

        /**
         * @brief Set the internal time point to the default-constructed steady-clock time point.
         */
        void Reset(void) noexcept { m_t0 = std::chrono::time_point<std::chrono::steady_clock>(); }

        /**
         * @brief Start the performance counter by setting the internal timepoint to now.
         */
        void Start(void) noexcept { m_t0 = std::chrono::steady_clock::now(); }

        /**
         * @brief Get the elapsed time in seconds since the internal time point.
         * @return The time since the internal time point, which is set by @ref Reset or @ref Start. The steady-clock epoch is unspecified.
         */
        [[nodiscard]] double TimeToStart(void) const noexcept {
            auto t1 = std::chrono::steady_clock::now();
            return 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - m_t0).count());
        }

    protected:
        std::chrono::time_point<std::chrono::steady_clock> m_t0;   // Time point when performance counter was started.
};


} /* namespace: runtime */


} /* namespace: mpsv */
