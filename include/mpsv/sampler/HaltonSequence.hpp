#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class generates deterministic samples according to the halton sequence for up to 16 dimensions.
 * The samples are stored in a look-up table (LUT) and are accessed via indexing the LUT.
 * @tparam Tdimension The number of dimensions to be used. Must be in range [1,16].
 */
template<size_t Tdimension> class HaltonSequence {
    static_assert((Tdimension >= 1) && (Tdimension <= 16), "Halton sequence dimension must be in range [1,16]!");

    public:
        /**
         * @brief Initialize the Halton sequence and build the look-up tables.
         * @param[in] maxNumSamples Requested maximum number of samples to be used. The value is clamped to the safe maximum.
         * @param[in] scale Optional scale value for samples. Samples are to be generated in range [0,scale]. Scale defaults to 1.0.
         */
        void Initialize(size_t maxNumSamples, double scale = 1.0) noexcept {
            Terminate();
            maxNumSamples = std::clamp(maxNumSamples, size_t{1}, m_samples.max_size());
            m_samples.resize(maxNumSamples);
            constexpr size_t primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53};
            #pragma omp parallel for if (maxNumSamples > 100000)
            for(size_t s = 0; s < maxNumSamples; s++){
                for (size_t dim = 0; dim < Tdimension; dim++) {
                    size_t base = primes[dim];
                    double invBase = 1.0 / static_cast<double>(base);
                    size_t n = s + 1;
                    double q = 0.0;
                    double bk = invBase;
                    while(n > 0){
                        q += bk * static_cast<double>(n % base);
                        n = size_t(static_cast<double>(n) * invBase);
                        bk *= invBase;
                    }
                    m_samples[s][dim] = scale * q;
                }
            }
            m_samples.shrink_to_fit();
        }

        /**
         * @brief Terminate the halton sequence. This removes all precalculated samples.
         */
        void Terminate(void) noexcept {
            m_samples.clear();
            m_samples.shrink_to_fit();
        }

        /**
         * @brief Get a complete sample.
         * @param[in] idxSample Index of the sample.
         * @return The sample values in range [0,scale], where scale is set by the @ref Initialize member function.
         */
        [[nodiscard]] std::array<double, Tdimension> Sample(size_t idxSample) const noexcept {
            std::array<double, Tdimension> sample{};
            if(!m_samples.empty()){
                idxSample %= m_samples.size();
                return m_samples[idxSample];
            }
            return sample;
        }

    protected:
        std::vector<std::array<double, Tdimension>> m_samples;   // Precalculated samples storage.
};


} /* namespace: sampler */


} /* namespace: mpsv */
