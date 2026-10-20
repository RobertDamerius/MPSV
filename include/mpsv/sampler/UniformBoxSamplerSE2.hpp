#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/sampler/HaltonSequence.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/geometry/OrientedBox.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class represents a uniform box sampler, that samples random poses from an SE2 space.
 * The halton sequence is used to draw deterministic uniform samples.
 */
class UniformBoxSamplerSE2 {
    public:
        /**
         * @brief Initialize the uniform box sampler.
         * @param[in] maxNumSamples Maximum number of samples to be used. Must be at least one.
         * @return True if success, false otherwise.
         */
        bool Initialize(size_t maxNumSamples) noexcept {
            m_R[0] = m_R[3] = m_invR[0] = m_invR[3] = 1.0;
            m_R[1] = m_R[2] = m_invR[1] = m_invR[2] = 0.0;
            m_offset[0] = m_offset[1] = 0.0;
            return m_haltonSequence.Initialize(maxNumSamples);
        }

        /**
         * @brief Terminate the uniform box sampler. All precalculated samples are removed and internal values are set to default values.
         */
        void Terminate(void) noexcept {
            m_haltonSequence.Terminate();
            m_R[0] = m_R[3] = m_invR[0] = m_invR[3] = 1.0;
            m_R[1] = m_R[2] = m_invR[1] = m_invR[2] = 0.0;
            m_offset[0] = m_offset[1] = 0.0;
        }

        /**
         * @brief Set the size of the uniform box sampler. The dimension MUST NOT be zero!
         * @param[in] box A valid oriented box with non-zero dimension.
         */
        void SetSize(const mpsv::geometry::OrientedBox& box) noexcept {
            m_offset = box.GetPosition();
            std::array<double,2> dimension = box.GetDimension();
            std::array<double,2> normal = box.GetNormal();
            dimension[0] = std::max(dimension[0], 1.0e-6);
            dimension[1] = std::max(dimension[1], 1.0e-6);
            m_R[0] = normal[0] * dimension[0];
            m_R[1] = -normal[1] * dimension[1];
            m_R[2] = normal[1] * dimension[0];
            m_R[3] = normal[0] * dimension[1];
            double invDetR = 1.0 / (m_R[0]*m_R[3] - m_R[1]*m_R[2]);
            m_invR[0] = m_R[3] * invDetR;
            m_invR[1] = -m_R[1] * invDetR;
            m_invR[2] = -m_R[2] * invDetR;
            m_invR[3] = m_R[0] * invDetR;
        }

        /**
         * @brief Get a uniform sample from the box.
         * @param[in] idxSample Index of the sample.
         * @return The output sample given as {x, y, psi}.
         */
        std::array<double, 3> Sample(size_t idxSample) noexcept {
            std::array<double, 3> sample = m_haltonSequence.Sample(idxSample);
            return {
                m_R[0]*sample[0] + m_R[1]*sample[1] + m_offset[0],
                m_R[2]*sample[0] + m_R[3]*sample[1] + m_offset[1],
                mpsv::math::SymmetricalAngle(6.28318530717959 * sample[2])
            };
        }

        /**
         * @brief Check whether a given point is inside the sampling area or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The point to be checked.
         * @return True if given point is inside the sampling area, false otherwise.
         */
        template <class T> bool IsInside(const T& point) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {point} must be of type std::array<double,N> with N >= 2!");
            double x = point[0] - m_offset[0];
            double y = point[1] - m_offset[1];
            double sx = m_invR[0]*x + m_invR[1]*y;
            double sy = m_invR[2]*x + m_invR[3]*y;
            return (sx >= 0.0) && (sx <= 1.0) && (sy >= 0.0) && (sy <= 1.0);
        }

    protected:
        mpsv::sampler::HaltonSequence<3> m_haltonSequence;   // Internal halton sequence sampler.
        std::array<double, 4> m_R{1.0, 0.0, 0.0, 1.0};       // Scaled rotation matrix (row major order). A sample within the box is calculated by R*sample + offset.
        std::array<double, 4> m_invR{1.0, 0.0, 0.0, 1.0};    // Inverse scaled rotation matrix (row major order).
        std::array<double, 2> m_offset{0.0, 0.0};            // The origin of the box. This is the lower-left corner in the box frame.
};


} /* namespace: sampler */


} /* namespace: mpsv */
