#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/math/Additional.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief This class represents a path smoother that uses B-spline interpolation in SE(2).
 */
class PathSmoother {
    public:
        /**
         * @brief Construct a new path smoother.
         */
        PathSmoother() noexcept {
            M.push_back({0, 1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0}); // [1 t t^2 t^3] * C, for t = 1
        }

        /**
         * @brief Reset the path smoother.
         * @param[in] subdivision The number of subdivisions for a polygonal chain representing a B-spline section.
         */
        void Reset(uint16_t subdivision) noexcept {
            uint32_t numVertices = static_cast<uint32_t>(subdivision) + 1;
            M.resize(numVertices);
            double dt = 1.0 / static_cast<double>(numVertices);
            double t = 0;
            for(uint32_t k = 0; k < numVertices; ++k){
                t += dt;
                double t2 = t * t;
                double t3 = t * t2;
                M[k][0] = (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0;
                M[k][1] = (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0;
                M[k][2] = (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0;
                M[k][3] = t3 / 6.0;
            }
        }

        /**
         * @brief Create a B-spline in SE(2).
         * @param[in] path The input path to be smoothed via B-spline interpolation.
         * @return The polygonal chain representing the B-spline. The number subdivisions for a B-spline section is controlled via the @ref Reset member function.
         */
        std::vector<std::array<double,3>> CreateBSpline(const std::vector<std::array<double,3>>& path) const noexcept {
            std::vector<std::array<double,3>> curve;
            int32_t N = static_cast<int32_t>(path.size());
            if(N < 3){
                curve = path;
            }
            else{
                int32_t NminusOne = N - 1;
                std::array<double,3> p0, p1, p2, p3;
                curve.push_back(path[0]);
                for(int32_t i = -2; i < NminusOne; ++i){
                    const std::array<double,3>& q0 = path[std::clamp(i, 0, NminusOne)];
                    const std::array<double,3>& q1 = path[std::clamp(i + 1, 0, NminusOne)];
                    const std::array<double,3>& q2 = path[std::clamp(i + 2, 0, NminusOne)];
                    const std::array<double,3>& q3 = path[std::clamp(i + 3, 0, NminusOne)];
                    p0 = q0;
                    p1 = q1;
                    p2 = q2;
                    p3 = q3;
                    double pivotAngle = q0[2];
                    p0[2] = 0.0;
                    p1[2] = mpsv::math::SymmetricalAngle(q1[2] - q0[2]);
                    p2[2] = p1[2] + mpsv::math::SymmetricalAngle(q2[2] - q1[2]);
                    p3[2] = p2[2] + mpsv::math::SymmetricalAngle(q3[2] - q2[2]);
                    for(auto&& m : M){
                        double x = m[0]*p0[0] + m[1]*p1[0] + m[2]*p2[0] + m[3]*p3[0];
                        double y = m[0]*p0[1] + m[1]*p1[1] + m[2]*p2[1] + m[3]*p3[1];
                        double psi = mpsv::math::SymmetricalAngle(m[0]*p0[2] + m[1]*p1[2] + m[2]*p2[2] + m[3]*p3[2] + pivotAngle);
                        curve.push_back({x, y, psi});
                    }
                }
            }
            return curve;
        }

    protected:
        std::vector<std::array<double,4>> M; // Matrix M = [1 t t^2 t^3] * C, containing the parameter space t (0, 1] for the polygonal chain of one B-spline section as well as the B-spline coefficient matrix C = 1/6 * [1 4 1 0; -3 0 3 0; 3 -6 3 0; -1 3 -3 1]. Each entry represents a row.
};


} /* namespace: geometry */


} /* namespace: mpsv */

