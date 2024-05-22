#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_sampler_HaltonSequence.hpp>
#include <mpsv_math_Additional.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief This class represents a sampler to be used for sampling poses around a given path.
 */
class PathSampler {
    public:
        /**
         * @brief Create a new path sampler object.
         */
        PathSampler() noexcept {}

        /**
         * @brief Initialize the path sampler and build the look-up-tables.
         * @param[in] maxNumSamples Maximum number of samples to be used. Must be at least one.
         * @return True if success, false otherwise. If input arguments are invalid, false is returned.
         */
        bool Initialize(size_t maxNumSamples) noexcept {
            return haltonSequence.Initialize(4, maxNumSamples);
        }

        /**
         * @brief Reset the path sampler by specifiying a new path and sampling range.
         * @param[in] path The input path, where each pose is indicated by [x, y, psi]. The path must contain at least one pose.
         * @param[in] rangePosition Maximum range for position sampling around the path.
         * @param[in] rangeAngle Maximum range for angle sampling around the path.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function. This value is used to compute the volume of the sampling area in SE(2).
         * @details THE PATH MUST CONTAIN AT LEAST ONE NODE. OTHERWISE SAMPLING WONT BE POSSIBLE!
         */
        void Reset(const std::vector<std::array<double,3>>& path, double rangePosition, double rangeAngle, double metricWeightPsi) noexcept {
            segments.clear();
            if(1 == path.size()){
                segments.push_back(Segment());
                segments.back().A[0] = 2.0 * rangePosition;
                segments.back().A[1] = 0.0;
                segments.back().A[2] = 0.0;
                segments.back().A[3] = 0.0;
                segments.back().A[4] = 2.0 * rangePosition;
                segments.back().A[5] = 0.0;
                segments.back().A[6] = 0.0;
                segments.back().A[7] = 0.0;
                segments.back().A[8] = 2.0 * rangeAngle;
                segments.back().invA[0] = 1.0 / segments.back().A[0];
                segments.back().invA[1] = 0.0;
                segments.back().invA[2] = 0.0;
                segments.back().invA[3] = 0.0;
                segments.back().invA[4] = 1.0 / segments.back().A[4];
                segments.back().invA[5] = 0.0;
                segments.back().invA[6] = 0.0;
                segments.back().invA[7] = 0.0;
                segments.back().invA[8] = 1.0 / segments.back().A[8];
                segments.back().b[0] = path[0][0] - rangePosition;
                segments.back().b[1] = path[0][1] - rangePosition;
                segments.back().b[2] = path[0][2] - rangeAngle;
                segments.back().ratio = 1.0;
                volume = 8.0 * rangePosition * rangePosition * rangeAngle * metricWeightPsi;
            }
            else{
                constexpr double eps = std::numeric_limits<double>::epsilon();
                std::array<double,3> p, nu, nv, nw;
                double L, d;
                double V = 0.0;
                for(size_t i = 1; i < path.size(); ++i){
                    p = mpsv::math::PoseDifference(path[i], path[i - 1]);
                    p[0] /= rangePosition;
                    p[1] /= rangePosition;
                    p[2] /= rangeAngle;
                    L = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
                    V += 4.0 * L + 8.0;
                    nu = {1.0, 0.0, 0.0};
                    if(L > eps){
                        nu[0] = p[0] / L;
                        nu[1] = p[1] / L;
                        nu[2] = p[2] / L;
                    }
                    nv[0] = -nu[1];
                    nv[1] = nu[0];
                    nv[2] = 0.0;
                    d = std::sqrt(nv[0]*nv[0] + nv[1]*nv[1]);
                    if(d > eps){
                        nv[0] /= d;
                        nv[1] /= d;
                        nv[2] /= d;
                    }
                    else{
                        nv = {1.0, 0.0, 0.0};
                    }
                    nw[0] = -nu[2]*nv[1];
                    nw[1] = nu[2]*nv[0];
                    nw[2] = nu[0]*nv[1] - nu[1]*nv[0];
                    segments.push_back(Segment());
                    segments.back().ratio = V;
                    segments.back().b[0] = path[i-1][0] - rangePosition * (nu[0] + nv[0] + nw[0]);
                    segments.back().b[1] = path[i-1][1] - rangePosition * (nu[1] + nv[1] + nw[1]);
                    segments.back().b[2] = path[i-1][2] - rangeAngle * (nu[2] + nv[2] + nw[2]);
                    d = (2.0 + L);
                    nu[0] *= d;
                    nu[1] *= d;
                    nu[2] *= d;
                    nv[0] *= 2.0;
                    nv[1] *= 2.0;
                    nv[2] *= 2.0;
                    nw[0] *= 2.0;
                    nw[1] *= 2.0;
                    nw[2] *= 2.0;
                    segments.back().A[0] = rangePosition * nu[0];
                    segments.back().A[1] = rangePosition * nv[0];
                    segments.back().A[2] = rangePosition * nw[0];
                    segments.back().A[3] = rangePosition * nu[1];
                    segments.back().A[4] = rangePosition * nv[1];
                    segments.back().A[5] = rangePosition * nw[1];
                    segments.back().A[6] = rangeAngle * nu[2];
                    segments.back().A[7] = rangeAngle * nv[2];
                    segments.back().A[8] = rangeAngle * nw[2];
                    std::tie(std::ignore, segments.back().invA) = mpsv::math::MatrixInverse3x3(segments.back().A);
                }
                for(auto&& segment : segments){
                    segment.ratio /= V;
                }
                volume = V * rangePosition * rangePosition * rangeAngle * metricWeightPsi;
            }
        }

        /**
         * @brief Get the sample.
         * @param[in] idxSample Index of the sample.
         * @return Resulting sample around the path. The elements are [x,y,psi].
         * @note IMPORTANT: THE INDEX VALUES ACCESS THE INTERNAL CONTAINER DIRECTLY. MAKE SURE THE VALUES ARE IN RANGE TO PREVENT ACCESS VIOLATION!
         * @details A random path segment is selected based on the halton sequence (dimension 1). The random sample is generated in a 3-dimensional box around this
         * path segment by the halton sequence (dimension 2,3,4). The dimension of the bos corresponds to rangePosition and rangeAngle, respectively.
         */
        std::array<double,3> Sample(size_t idxSample) noexcept {
            std::array<double, 3> sample;
            double s = haltonSequence.Sample(3, idxSample);
            int32_t N = static_cast<int32_t>(segments.size());
            int32_t i = 0;
            for(i = 0; (i < N) && (segments[i].ratio < s); ++i);
            double sx = haltonSequence.Sample(0, idxSample);
            double sy = haltonSequence.Sample(1, idxSample);
            double sz = haltonSequence.Sample(2, idxSample);
            sample[0] = segments[i].b[0] + segments[i].A[0] * sx + segments[i].A[1] * sy + segments[i].A[2] * sz;
            sample[1] = segments[i].b[1] + segments[i].A[3] * sx + segments[i].A[4] * sy + segments[i].A[5] * sz;
            sample[2] = mpsv::math::SymmetricalAngle(segments[i].b[2] + segments[i].A[6] * sx + segments[i].A[7] * sy + segments[i].A[8] * sz);
            return sample;
        }

        /**
         * @brief Check whether a given pose is inside the sampling area or not.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] pose The pose to be checked.
         * @return true True if the given pose is inside the sampling area, false otherwise.
         */
        template <class T> bool IsInside(const T pose) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            bool isInside = false;
            std::array<double,3> tmp, s;
            for(auto&& box : segments){
                tmp[0] = pose[0] - box.b[0];
                tmp[1] = pose[1] - box.b[1];
                tmp[2] = mpsv::math::SymmetricalAngle(pose[2] - box.b[2]);
                s[0] = box.invA[0]*tmp[0] + box.invA[1]*tmp[1] + box.invA[2]*tmp[2];
                s[1] = box.invA[3]*tmp[0] + box.invA[4]*tmp[1] + box.invA[5]*tmp[2];
                s[2] = box.invA[6]*tmp[0] + box.invA[7]*tmp[1] + box.invA[8]*tmp[2];
                isInside |= (s[0] > 0.0) && (s[0] < 1.0) && (s[1] > 0.0) && (s[1] < 1.0) && (s[2] > 0.0) && (s[2] < 1.0);
            }
            return isInside;
        }

        /**
         * @brief Get the volume of the whole sampling area in SE(2).
         * @return Volume of the sampling area.
         * @details The volume corresponds to (x) * (y) * (wpsi * psi), where wpsi denotes the weighting for psi in the metric distance function.
         */
        double GetVolume(void){ return volume; }

    protected:
        struct Segment {
            double ratio;                               // Ratio of the volume of this path segment compared to the volume of all segments in range [0, 1].
            std::array<double,9> A;                     // Matrix (row-major order) for sampling of this path segment according to A*s + b.
            std::array<double,3> b;                     // Vector for sampling of this path segment according to A*s + b.
            std::array<double,9> invA;                  // Inverse matrix (row-major order) of @ref A.
        };
        double volume;                                  // Volume of the whole sampling area.
        std::vector<Segment> segments;                  // List of path segments.
        mpsv::sampler::HaltonSequence haltonSequence;   // The internal halton sequence used to sample poses from path segments.
};


} /* namespace: sampler */


} /* namespace: mpsv */

