#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_sampler_HaltonSequence.hpp>
#include <mpsv_math_Additional.hpp>
#include <mpsv_geometry_OrientedBox.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class represents a uniform box sampler, that samples random poses from an SE2 space.
 * The halton sequence is used to draw deterministic uniform samples.
 */
class UniformBoxSamplerSE2 {
    public:
        /**
         * @brief Construct a uniform box sampler for SE2.
         */
        UniformBoxSamplerSE2() noexcept {
            R[0] = R[3] = invR[0] = invR[3] = 1.0;
            R[1] = R[2] = invR[1] = invR[2] = 0.0;
            offset[0] = offset[1] = 0.0;
        }

        /**
         * @brief Initialize the uniform box sampler.
         * @param[in] maxNumSamples Maximum number of samples to be used. Must be at least one.
         * @return True if success, false otherwise.
         */
        bool Initialize(size_t maxNumSamples) noexcept {
            R[0] = R[3] = invR[0] = invR[3] = 1.0;
            R[1] = R[2] = invR[1] = invR[2] = 0.0;
            offset[0] = offset[1] = 0.0;
            return haltonSequence.Initialize(3, maxNumSamples);
        }

        /**
         * @brief Terminate the uniform box sampler. All precalculated samples are removed and internal values are set to default values.
         */
        void Terminate(void) noexcept {
            haltonSequence.Terminate();
            R[0] = R[3] = invR[0] = invR[3] = 1.0;
            R[1] = R[2] = invR[1] = invR[2] = 0.0;
            offset[0] = offset[1] = 0.0;
        }

        /**
         * @brief Set the size of the uniform box sampler. The dimension MUST NOT be zero!
         * @param[in] box A valid oriented box with non-zero dimension.
         * @note IMPORTANT: ENSURE THAT THE INPUT BOX IS A VALID BOX WITH NON-ZERO DIMENSION!
         */
        void SetSize(const mpsv::geometry::OrientedBox& box) noexcept {
            offset = box.GetPosition();
            std::array<double,2> dimension = box.GetDimension();
            std::array<double,2> normal = box.GetNormal();
            R[0] = normal[0] * dimension[0];
            R[1] = -normal[1] * dimension[1];
            R[2] = normal[1] * dimension[0];
            R[3] = normal[0] * dimension[1];
            double invDetR = 1.0 / (R[0]*R[3] - R[1]*R[2]);
            invR[0] = R[3] * invDetR;
            invR[1] = -R[1] * invDetR;
            invR[2] = -R[2] * invDetR;
            invR[3] = R[0] * invDetR;
        }

        /**
         * @brief Get a uniform sample from the box.
         * @param[in] idxSample Index of the sample.
         * @return The output sample given as {x, y, psi}.
         * @note IMPORTANT: THE INDEX VALUE ACCESS THE INTERNAL CONTAINER DIRECTLY. MAKE SURE THE VALUES ARE IN RANGE TO PREVENT ACCESS VIOLATION!
         */
        std::array<double, 3> Sample(size_t idxSample) noexcept {
            std::array<double, 3> result;
            double s0 = haltonSequence.Sample(0, idxSample);
            double s1 = haltonSequence.Sample(1, idxSample);
            double s2 = haltonSequence.Sample(2, idxSample);
            result[0] = R[0]*s0 + R[1]*s1 + offset[0];
            result[1] = R[2]*s0 + R[3]*s1 + offset[1];
            result[2] = mpsv::math::SymmetricalAngle(6.28318530717959 * s2);
            return result;
        }

        /**
         * @brief Check whether a given pose is inside the sampling area or not.
         * @param[in] pose The pose to be checked. Only the position (x,y) is checked.
         * @return True if given pose is inside the sampling area, false otherwise.
         */
        bool IsInside(const std::array<double,3>& pose) noexcept {
            double x = pose[0] - offset[0];
            double y = pose[1] - offset[1];
            double sx = invR[0]*x + invR[1]*y;
            double sy = invR[2]*x + invR[3]*y;
            return (sx >= 0.0) && (sx <= 1.0) && (sy >= 0.0) && (sy <= 1.0);
        }

    protected:
        mpsv::sampler::HaltonSequence haltonSequence;   // Internal halton sequence sampler.
        std::array<double, 4> R;                        // Scaled rotation matrix (row major order). A sample within the box is calculated by R*sample + offset.
        std::array<double, 4> invR;                     // Inverse scaled rotation matrix (row major order).
        std::array<double, 2> offset;                   // The origin of the box. This is the lower-left corner in the box frame.
};


} /* namespace: sampler */


} /* namespace: mpsv */

