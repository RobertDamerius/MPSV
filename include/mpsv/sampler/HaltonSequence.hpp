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
         * @param[in] maxNumSamples Maximum number of samples to be used.
         * @param[in] scale Optional scale value for samples. Samples are to be generated in range [0,scale]. Scale defaults to 1.0.
         * @return True if success, false otherwise.
         */
        bool Initialize(size_t maxNumSamples, double scale = 1.0) noexcept {
            Terminate();
            m_maxNumSamples = std::max<size_t>(1, maxNumSamples);
            try {
                m_samples.resize(Tdimension * m_maxNumSamples);
            } catch(...) {
                return false;
            }
            constexpr size_t primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53};
            #pragma omp parallel for if (m_maxNumSamples > 100000)
            for(size_t s = 0; s < m_maxNumSamples; s++){
                size_t sample_offset = s * Tdimension;
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
                    m_samples[sample_offset + dim] = scale * q;
                }
            }
            m_samples.shrink_to_fit();
            return true;
        }

        /**
         * @brief Terminate the halton sequence. This removes all precalculated samples.
         */
        void Terminate(void) noexcept {
            m_samples.clear();
            m_samples.shrink_to_fit();
            m_maxNumSamples = 0;
        }

        /**
         * @brief Get the sample of a specific dimension.
         * @param[in] idxDimension Index of the dimension.
         * @param[in] idxSample Index of the sample.
         * @return The sample value in range [0,scale], where scale is set by the @ref Initialize member function.
         */
        double Sample(size_t idxDimension, size_t idxSample) const noexcept {
            if(!m_maxNumSamples){
                return 0.0;
            }
            idxDimension %= Tdimension;
            idxSample %= m_maxNumSamples;
            return m_samples[idxSample * Tdimension + idxDimension];
        }

    protected:
        std::vector<double> m_samples;   // Flattened precalculated samples storage.
        size_t m_maxNumSamples{0};       // Max samples per dimension.
};


} /* namespace: sampler */


} /* namespace: mpsv */
