#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv::geometry {


/**
 * @brief Run the douglas-peucker line simplification algorithm for a sequential list of poses {x,y,psi}.
 * @tparam Tpose Template parameter must be of type std::array<double,N> with N >= 3.
 * @param[in] poses Reference to a container of poses that represent the input line to be simplified. A pose type must be at least of dimension 3.
 * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. This value must be positive.
 * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. This value must be positive.
 * @return The output container that will contain the indices to those poses indicating the simplified line. This container is empty if there're less than two input poses.
 * @details As long as the 2D position error is larger than the threshold, the algorithm behaves equal to the @ref DouglasPeucker2D algorithm. If the position error of a simplified line
 * segment is smaller than the specified @ref thresholdPosition, then it is checked whether the angle error stays under the specified threshold. If not, the line segment is further divided.
 */
template <class Tpose> [[nodiscard]] inline std::vector<std::size_t> DouglasPeuckerSE2(const std::vector<Tpose>& poses, double thresholdPosition, double thresholdAngle) noexcept {
    static_assert(mpsv::is_vec3<Tpose>::value, "Argument {poses} must be of type std::vector<std::array<double,N>> with N >= 3!");
    std::vector<std::size_t> indices;
    std::size_t numPoses{poses.size()};
    if (numPoses > 1U) {
        indices.push_back(0U);
        const double thresholdPositionAbs{std::fabs(thresholdPosition)};
        const double thresholdAngleAbs{std::fabs(thresholdAngle)};
        std::vector<std::array<std::size_t, 2U>> intervals{{0U, numPoses - 1U}};
        while (!intervals.empty()) {
            const auto interval = intervals.back();
            intervals.pop_back();
            const std::size_t idxStart{interval[0U]};
            const std::size_t idxEnd{interval[1U]};

            // Just a line: nothing to do
            if ((idxStart + 1U) >= idxEnd) {
                continue;
            }

            // Implicit edge function
            double xStart{poses[idxStart][0U]};
            double yStart{poses[idxStart][1U]};
            double xEnd{poses[idxEnd][0U]};
            double yEnd{poses[idxEnd][1U]};
            double a{yEnd - yStart};
            double b{xStart - xEnd};
            double len{std::sqrt(a*a + b*b)};
            std::size_t ip{0U};
            double dmax{-1.0};
            double amax{-1.0};
            if (len > 0.0) {
                len = 1.0 / len;
                a *= len;
                b *= len;
                const double c{-a * xStart - b * yStart};

                // Search farthest point for position and largest angle error
                for (std::size_t i{idxStart + 1U}; i < idxEnd; i++) {
                    len = std::fabs(a*poses[i][0U] + b*poses[i][1U] + c);
                    amax = std::max(amax, std::fabs(mpsv::math::SymmetricalAngle(poses[i][2U] - poses[idxStart][2U])));
                    if (len > dmax) {
                        dmax = len;
                        ip = i;
                    }
                }
            } else {
                // Search farthest point for position and largest angle error
                for (std::size_t i{idxStart + 1U}; i < idxEnd; i++) {
                    xEnd = poses[i][0U] - xStart;
                    yEnd = poses[i][1U] - yStart;
                    len = std::sqrt(xEnd*xEnd + yEnd*yEnd);
                    amax = std::max(amax, std::fabs(mpsv::math::SymmetricalAngle(poses[i][2U] - poses[idxStart][2U])));
                    if (len > dmax) {
                        dmax = len;
                        ip = i;
                    }
                }
            }

            if (dmax >= thresholdPositionAbs) {
                indices.push_back(ip);
                intervals.push_back({ip, idxEnd});
                intervals.push_back({idxStart, ip});
            } else if (amax >= thresholdAngleAbs) {
                const std::size_t ia{idxStart + (idxEnd - idxStart) / 2U};
                indices.push_back(ia);
                intervals.push_back({ia, idxEnd});
                intervals.push_back({idxStart, ia});
            }
        }
        std::sort(indices.begin(), indices.end());
        indices.push_back(numPoses - 1U);
    }
    return indices;
}


} /* namespace: mpsv::geometry */
