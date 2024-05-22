#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class represents a uniform ball sampler, that samples random points from a 3D sphere.
 */
class UniformBallSampler3D {
    public:
        /**
         * @brief Create a uniform ball sampler for dimension 3 using the dropped dimension method.
         * @details The default seed is set to zero.
         */
        UniformBallSampler3D() noexcept { Initialize(0,0); }

        /**
         * @brief Initialize the uniform ball sampler.
         * @param[in] maxNumSamples Maximum number of samples.
         * @param[in] seed New seed to be used, defaults to 0.
         */
        void Initialize(size_t maxNumSamples, unsigned seed = 0) noexcept {
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            normalDistribution.reset();
            generator.seed(seed);
            samples.clear();
            samples.reserve(maxNumSamples);
            for(size_t k = 0; k < maxNumSamples; k++){
                double s0 = normalDistribution(generator);
                double s1 = normalDistribution(generator);
                double s2 = normalDistribution(generator);
                double s3 = normalDistribution(generator);
                double s4 = normalDistribution(generator);
                double n = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3 + s4*s4);
                if(n <= eps){
                    samples.push_back({0.0, 0.0, 0.0});
                    continue;
                }
                n = 1.0 / n;
                samples.push_back({s0 * n, s1 * n, s2 * n});
            }
        }

        /**
         * @brief Get a uniform random sample in a 3D ball.
         * @param[in] idxSample Index of the sample to obtain.
         * @return Resulting sample value.
         * @note IMPORTANT: THE INDEX VALUES ACCESS THE INTERNAL CONTAINER DIRECTLY. MAKE SURE THE VALUES ARE IN RANGE TO PREVENT ACCESS VIOLATION!
         */
        std::array<double,3> Sample(size_t idxSample) noexcept { return samples[idxSample]; }

    protected:
        std::mt19937 generator;                                // Mersenne Twister pseudo-random number generator with a state size of 19937 bits.
        std::normal_distribution<double> normalDistribution;   // Normal distribution with mean = 0.0 and stddev = 1.0.
        std::vector<std::array<double, 3>> samples;            // List of samples.
};


} /* namespace: sampler */


} /* namespace: mpsv */

