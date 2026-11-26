#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/sampler/HaltonSequence.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/geometry/OrientedBox.hpp>


namespace mpsv::sampler {


/**
 * @brief This class represents a uniform box sampler, that samples random poses from an SE2 space.
 * The halton sequence is used to draw deterministic uniform samples.
 */
class UniformBoxSamplerSE2 {
    public:
        /**
         * @brief Generate samples for the uniform box sampler.
         * @param[in] maxNumSamples Maximum number of samples to be generated.
         */
        void Generate(std::size_t maxNumSamples) noexcept {
            m_R[0U] = m_R[3U] = m_invR[0U] = m_invR[3U] = 1.0;
            m_R[1U] = m_R[2U] = m_invR[1U] = m_invR[2U] = 0.0;
            m_offset[0U] = m_offset[1U] = 0.0;
            m_haltonSequence.Generate(maxNumSamples);
        }

        /**
         * @brief Set the size of the uniform box sampler. The dimension MUST NOT be zero!
         * @param[in] box A valid oriented box with non-zero dimension.
         */
        void SetSize(const mpsv::geometry::OrientedBox& box) noexcept {
            m_offset = box.GetPosition();
            std::array<double, 2U> dimension = box.GetDimension();
            std::array<double, 2U> normal = box.GetNormal();
            dimension[0U] = std::max(dimension[0U], 1.0e-6);
            dimension[1U] = std::max(dimension[1U], 1.0e-6);
            m_R[0U] = normal[0U] * dimension[0U];
            m_R[1U] = -normal[1U] * dimension[1U];
            m_R[2U] = normal[1U] * dimension[0U];
            m_R[3U] = normal[0U] * dimension[1U];
            double invDetR{1.0 / (m_R[0U]*m_R[3U] - m_R[1U]*m_R[2U])};
            m_invR[0U] = m_R[3U] * invDetR;
            m_invR[1U] = -m_R[1U] * invDetR;
            m_invR[2U] = -m_R[2U] * invDetR;
            m_invR[3U] = m_R[0U] * invDetR;
        }

        /**
         * @brief Get a uniform sample from the box.
         * @param[in] idxSample Index of the sample.
         * @return The output sample given as {x, y, psi}.
         */
        [[nodiscard]] std::array<double, 3U> Sample(std::size_t idxSample) const noexcept {
            constexpr double pi2{2.0 * std::numbers::pi};
            std::array<double, 3U> sample = m_haltonSequence.Sample(idxSample);
            return {
                m_R[0U]*sample[0U] + m_R[1U]*sample[1U] + m_offset[0U],
                m_R[2U]*sample[0U] + m_R[3U]*sample[1U] + m_offset[1U],
                mpsv::math::SymmetricalAngle(pi2 * sample[2U])
            };
        }

        /**
         * @brief Check whether a given point is inside the sampling area or not.
         * @tparam Tpoint Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] point The point to be checked.
         * @return True if given point is inside the sampling area, false otherwise.
         */
        template <class Tpoint> [[nodiscard]] bool IsInside(const Tpoint& point) const noexcept {
            static_assert(mpsv::is_vec2<Tpoint>::value, "Argument {point} must be of type std::array<double,N> with N >= 2!");
            double x{point[0U] - m_offset[0U]};
            double y{point[1U] - m_offset[1U]};
            double sx{m_invR[0U]*x + m_invR[1U]*y};
            double sy{m_invR[2U]*x + m_invR[3U]*y};
            return (sx >= 0.0) && (sx <= 1.0) && (sy >= 0.0) && (sy <= 1.0);
        }

    protected:
        mpsv::sampler::HaltonSequence<3U> m_haltonSequence;   // Internal halton sequence sampler.
        std::array<double, 4U> m_R{1.0, 0.0, 0.0, 1.0};       // Scaled rotation matrix (row major order). A sample within the box is calculated by R*sample + offset.
        std::array<double, 4U> m_invR{1.0, 0.0, 0.0, 1.0};    // Inverse scaled rotation matrix (row major order).
        std::array<double, 2U> m_offset{0.0, 0.0};            // The origin of the box. This is the lower-left corner in the box frame.
};


} /* namespace: mpsv::sampler */
