#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv::sampler {


/**
 * @brief This class generates deterministic samples according to the halton sequence for up to 16 dimensions.
 * The samples are stored in a look-up table (LUT) and are accessed via indexing the LUT.
 * @tparam Tdimension The number of dimensions to be used. Must be in range [1,16].
 */
template<std::size_t Tdimension> class HaltonSequence {
    static_assert((Tdimension >= 1U) && (Tdimension <= 16U), "Halton sequence dimension must be in range [1,16]!");

    public:
        /**
         * @brief Generate the Halton sequence and build a look-up table of samples.
         * @param[in] maxNumSamples Requested maximum number of samples to be generated. The value is clamped to the safe maximum.
         * @param[in] scale Optional scale value for samples. Samples are to be generated in range [0,scale]. Scale defaults to 1.0.
         */
        void Generate(std::size_t maxNumSamples, double scale = 1.0) noexcept {
            maxNumSamples = std::clamp(maxNumSamples, std::size_t{1U}, m_samples.max_size());
            m_samples.resize(maxNumSamples);
            constexpr std::size_t primes[] = {2U, 3U, 5U, 7U, 11U, 13U, 17U, 19U, 23U, 29U, 31U, 37U, 41U, 43U, 47U, 53U};
            #pragma omp parallel for if (maxNumSamples > 100000U)
            for (std::size_t s = 0U; s < maxNumSamples; s++) {
                for (std::size_t dim{0U}; dim < Tdimension; dim++) {
                    std::size_t base{primes[dim]};
                    double invBase{1.0 / static_cast<double>(base)};
                    std::size_t n{s + 1U};
                    double q{0.0};
                    double bk{invBase};
                    while (n > 0U) {
                        q += bk * static_cast<double>(n % base);
                        n = std::size_t(static_cast<double>(n) * invBase);
                        bk *= invBase;
                    }
                    m_samples[s][dim] = scale * q;
                }
            }
            m_samples.shrink_to_fit();
        }

        /**
         * @brief Get a complete sample.
         * @param[in] idxSample Index of the sample.
         * @return The sample values in range [0,scale], where scale is set by the @ref Generate member function.
         */
        [[nodiscard]] std::array<double, Tdimension> Sample(std::size_t idxSample) const noexcept {
            std::array<double, Tdimension> sample{};
            if (!m_samples.empty()) {
                idxSample %= m_samples.size();
                return m_samples[idxSample];
            }
            return sample;
        }

    protected:
        std::vector<std::array<double, Tdimension>> m_samples;   // Precalculated samples storage.
};


} /* namespace: mpsv::sampler */
