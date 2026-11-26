#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/sampler/HaltonSequence.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv::sampler {


/**
 * @brief This class represents a sampler to be used for sampling poses around a given path.
 */
class PathSampler {
    public:
        /**
         * @brief Generate samples for the path sampler and build the look-up-tables.
         * @param[in] maxNumSamples Maximum number of samples to be generated.
         */
        void Generate(std::size_t maxNumSamples) noexcept {
            m_haltonSequence.Generate(maxNumSamples);
        }

        /**
         * @brief Reset the path sampler by specifiying a new path and sampling range.
         * @param[in] path The input path, where each pose is indicated by [x, y, psi]. The path must contain at least one pose.
         * @param[in] rangePosition Maximum range for position sampling around the path.
         * @param[in] rangeAngle Maximum range for angle sampling around the path.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function. This value is used to compute the volume of the sampling area in SE(2).
         * @details THE PATH MUST CONTAIN AT LEAST ONE NODE. OTHERWISE SAMPLING WONT BE POSSIBLE!
         */
        void Reset(const std::vector<std::array<double, 3U>>& path, double rangePosition, double rangeAngle, double metricWeightPsi) noexcept {
            m_segments.clear();
            if (1U == path.size()) {
                m_segments.push_back(Segment());
                m_segments.back().A[0U] = 2.0 * rangePosition;
                m_segments.back().A[1U] = 0.0;
                m_segments.back().A[2U] = 0.0;
                m_segments.back().A[3U] = 0.0;
                m_segments.back().A[4U] = 2.0 * rangePosition;
                m_segments.back().A[5U] = 0.0;
                m_segments.back().A[6U] = 0.0;
                m_segments.back().A[7U] = 0.0;
                m_segments.back().A[8U] = 2.0 * rangeAngle;
                m_segments.back().invA[0U] = 1.0 / m_segments.back().A[0U];
                m_segments.back().invA[1U] = 0.0;
                m_segments.back().invA[2U] = 0.0;
                m_segments.back().invA[3U] = 0.0;
                m_segments.back().invA[4U] = 1.0 / m_segments.back().A[4U];
                m_segments.back().invA[5U] = 0.0;
                m_segments.back().invA[6U] = 0.0;
                m_segments.back().invA[7U] = 0.0;
                m_segments.back().invA[8U] = 1.0 / m_segments.back().A[8U];
                m_segments.back().b[0U] = path[0U][0U] - rangePosition;
                m_segments.back().b[1U] = path[0U][1U] - rangePosition;
                m_segments.back().b[2U] = path[0U][2U] - rangeAngle;
                m_segments.back().ratio = 1.0;
                m_volume = 8.0 * rangePosition * rangePosition * rangeAngle * metricWeightPsi;
            } else {
                constexpr double eps{std::numeric_limits<double>::epsilon()};
                std::array<double, 3U> p, nu, nv, nw;
                double L, d;
                double V{0.0};
                for (std::size_t i{1U}; i < path.size(); ++i) {
                    p = mpsv::math::PoseDifference(path[i], path[i - 1U]);
                    p[0U] /= rangePosition;
                    p[1U] /= rangePosition;
                    p[2U] /= rangeAngle;
                    L = std::sqrt(p[0U]*p[0U] + p[1U]*p[1U] + p[2U]*p[2U]);
                    V += 4.0 * L + 8.0;
                    nu = {1.0, 0.0, 0.0};
                    if (L > eps) {
                        nu[0U] = p[0U] / L;
                        nu[1U] = p[1U] / L;
                        nu[2U] = p[2U] / L;
                    }
                    nv[0U] = -nu[1U];
                    nv[1U] = nu[0U];
                    nv[2U] = 0.0;
                    d = std::sqrt(nv[0U]*nv[0U] + nv[1U]*nv[1U]);
                    if (d > eps) {
                        nv[0U] /= d;
                        nv[1U] /= d;
                        nv[2U] /= d;
                    } else {
                        nv = {1.0, 0.0, 0.0};
                    }
                    nw[0U] = -nu[2U]*nv[1U];
                    nw[1U] = nu[2U]*nv[0U];
                    nw[2U] = nu[0U]*nv[1U] - nu[1U]*nv[0U];
                    m_segments.push_back(Segment());
                    m_segments.back().ratio = V;
                    m_segments.back().b[0U] = path[i - 1U][0U] - rangePosition * (nu[0U] + nv[0U] + nw[0U]);
                    m_segments.back().b[1U] = path[i - 1U][1U] - rangePosition * (nu[1U] + nv[1U] + nw[1U]);
                    m_segments.back().b[2U] = path[i - 1U][2U] - rangeAngle * (nu[2U] + nv[2U] + nw[2U]);
                    d = (2.0 + L);
                    nu[0U] *= d;
                    nu[1U] *= d;
                    nu[2U] *= d;
                    nv[0U] *= 2.0;
                    nv[1U] *= 2.0;
                    nv[2U] *= 2.0;
                    nw[0U] *= 2.0;
                    nw[1U] *= 2.0;
                    nw[2U] *= 2.0;
                    m_segments.back().A[0U] = rangePosition * nu[0U];
                    m_segments.back().A[1U] = rangePosition * nv[0U];
                    m_segments.back().A[2U] = rangePosition * nw[0U];
                    m_segments.back().A[3U] = rangePosition * nu[1U];
                    m_segments.back().A[4U] = rangePosition * nv[1U];
                    m_segments.back().A[5U] = rangePosition * nw[1U];
                    m_segments.back().A[6U] = rangeAngle * nu[2U];
                    m_segments.back().A[7U] = rangeAngle * nv[2U];
                    m_segments.back().A[8U] = rangeAngle * nw[2U];
                    std::tie(std::ignore, m_segments.back().invA) = mpsv::math::MatrixInverse3x3(m_segments.back().A);
                }
                for (auto&& segment : m_segments) {
                    segment.ratio /= V;
                }
                m_volume = V * rangePosition * rangePosition * rangeAngle * metricWeightPsi;
            }
        }

        /**
         * @brief Get the sample.
         * @param[in] idxSample Index of the sample.
         * @return Resulting sample around the path. The elements are [x,y,psi].
         * @details A random path segment is selected based on the halton sequence (dimension 1). The random sample is generated in a 3-dimensional box around this
         * path segment by the halton sequence (dimension 2,3,4). The dimension of the bos corresponds to rangePosition and rangeAngle, respectively.
         */
        [[nodiscard]] std::array<double, 3U> Sample(std::size_t idxSample) const noexcept {
            std::array<double, 4U> haltonSample = m_haltonSequence.Sample(idxSample);
            std::size_t i{0U};
            for (; (i < m_segments.size()) && (m_segments[i].ratio < haltonSample[3U]); ++i);
            return {
                m_segments[i].b[0U] + m_segments[i].A[0U] * haltonSample[0U] + m_segments[i].A[1U] * haltonSample[1U] + m_segments[i].A[2U] * haltonSample[2U],
                m_segments[i].b[1U] + m_segments[i].A[3U] * haltonSample[0U] + m_segments[i].A[4U] * haltonSample[1U] + m_segments[i].A[5U] * haltonSample[2U],
                mpsv::math::SymmetricalAngle(m_segments[i].b[2U] + m_segments[i].A[6U] * haltonSample[0U] + m_segments[i].A[7U] * haltonSample[1U] + m_segments[i].A[8U] * haltonSample[2U])
            };
        }

        /**
         * @brief Check whether a given pose is inside the sampling area or not.
         * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be checked.
         * @return true True if the given pose is inside the sampling area, false otherwise.
         */
        template <class Tpose> [[nodiscard]] bool IsInside(const Tpose pose) const noexcept {
            static_assert(mpsv::is_vec3<Tpose>::value, "Argument {pose} must be of type std::array<double,N> with N >= 3!");
            bool isInside{false};
            std::array<double, 3U> tmp, s;
            for (auto&& box : m_segments) {
                tmp[0U] = pose[0U] - box.b[0U];
                tmp[1U] = pose[1U] - box.b[1U];
                tmp[2U] = mpsv::math::SymmetricalAngle(pose[2U] - box.b[2U]);
                s[0U] = box.invA[0U]*tmp[0U] + box.invA[1U]*tmp[1U] + box.invA[2U]*tmp[2U];
                s[1U] = box.invA[3U]*tmp[0U] + box.invA[4U]*tmp[1U] + box.invA[5U]*tmp[2U];
                s[2U] = box.invA[6U]*tmp[0U] + box.invA[7U]*tmp[1U] + box.invA[8U]*tmp[2U];
                isInside |= (s[0U] > 0.0) && (s[0U] < 1.0) && (s[1U] > 0.0) && (s[1U] < 1.0) && (s[2U] > 0.0) && (s[2U] < 1.0);
            }
            return isInside;
        }

        /**
         * @brief Get the volume of the whole sampling area in SE(2).
         * @return Volume of the sampling area.
         * @details The volume corresponds to (x) * (y) * (wpsi * psi), where wpsi denotes the weighting for psi in the metric distance function.
         * Note that this value is the sum of volumes of all overlapping sampling boxes around the path. Due to overlapping, the true volume would be smaller.
         */
        [[nodiscard]] double GetVolume() const noexcept {
            return m_volume;
        }

    protected:
        struct Segment {
            double ratio;                                     // Ratio of the volume of this path segment compared to the volume of all segments in range [0, 1].
            std::array<double, 9U> A;                         // Matrix (row-major order) for sampling of this path segment according to A*s + b.
            std::array<double, 3U> b;                         // Vector for sampling of this path segment according to A*s + b.
            std::array<double, 9U> invA;                      // Inverse matrix (row-major order) of @ref A.
        };
        double m_volume;                                      // Volume of the whole sampling area.
        std::vector<Segment> m_segments;                      // List of path segments.
        mpsv::sampler::HaltonSequence<4U> m_haltonSequence;   // The internal halton sequence used to sample poses from path segments.
};


} /* namespace: mpsv::sampler */
