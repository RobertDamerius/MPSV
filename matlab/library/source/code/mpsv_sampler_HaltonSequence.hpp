#pragma once


#include <mpsv_core_MPSVCommon.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class generates deterministic samples according to the halton sequence for up to 16 dimensions.
 * The samples are stored in a look-up table (LUT) and are accessed via indexing the LUT.
 */
class HaltonSequence {
    public:
        /**
         * @brief Initialize the Halton sequence and build the look-up tables.
         * @param[in] dimension The number of dimensions in range [1,16].
         * @param[in] maxNumSamples Maximum number of samples to be used. Must be at least one.
         * @param[in] scale Optional scale value for samples. Samples are to be generated in range [0,scale]. Scale defaults to 1.0.
         * @return True if success, false otherwise. If input arguments are invalid, false is returned.
         */
        bool Initialize(size_t dimension, size_t maxNumSamples, double scale = 1.0) noexcept {
            samples.clear();
            if(!dimension || (dimension > 16) || !maxNumSamples)
                return false;
            constexpr size_t primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53};
            this->samples.resize(dimension);
            for(size_t dim = 0; dim < dimension; dim++){
                this->samples[dim].resize(maxNumSamples);
                size_t base = primes[dim];
                double invBase = 1.0 / static_cast<double>(base);
                #ifndef MPSV_DONT_USE_OMP
                #pragma omp parallel for shared(samples) if (maxNumSamples > 100000)
                for(size_t s = 0; s < maxNumSamples; s++){
                    size_t n = s + 1;
                    double q = 0.0;
                    double bk = invBase;
                    while(n > 0){
                        q += bk * static_cast<double>(n % base);
                        n = size_t(static_cast<double>(n) * invBase);
                        bk *= invBase;
                    }
                    this->samples[dim][s] = scale * q;
                }
                #else
                for(size_t s = 0; s < maxNumSamples; s++){
                    size_t n = s + 1;
                    double q = 0.0;
                    double bk = invBase;
                    while(n > 0){
                        q += bk * static_cast<double>(n % base);
                        n = size_t(static_cast<double>(n) * invBase);
                        bk *= invBase;
                    }
                    this->samples[dim][s] = scale * q;
                }
                #endif
                this->samples[dim].shrink_to_fit();
            }
            samples.shrink_to_fit();
            return true;
        }

        /**
         * @brief Terminate the halton sequence. This removes all precalculated samples.
         */
        void Terminate(void) noexcept {
            samples.clear();
            samples.shrink_to_fit();
        }

        /**
         * @brief Get the sample of a specific dimension.
         * @param[in] idxDimension Index of the dimension.
         * @param[in] idxSample Index of the sample.
         * @return The sample value in range [0,scale], where scale is set by the @ref Initialize member function.
         * @note IMPORTANT: THE INDEX VALUES ACCESS THE INTERNAL CONTAINER DIRECTLY. MAKE SURE THE VALUES ARE IN RANGE TO PREVENT ACCESS VIOLATION!
         */
        double Sample(size_t idxDimension, size_t idxSample) noexcept { return samples[idxDimension][idxSample]; }

    protected:
        std::vector<std::vector<double>> samples;   // Precalculated samples for all dimensions in range [0,1].
};


} /* namespace: sampler */


} /* namespace: mpsv */

