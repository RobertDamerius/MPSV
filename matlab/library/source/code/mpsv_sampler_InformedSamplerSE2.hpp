#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_math_Additional.hpp>
#include <mpsv_math_SVD3.hpp>
#include <mpsv_sampler_UniformBallSampler3D.hpp>


namespace mpsv {


namespace sampler {


/**
 * @brief The informed sampler to be used for Informed-RRT*. The sampler draws random samples from the SE2 space.
 */
class InformedSamplerSE2 {
    public:
        /**
         * @brief Create an informed sampler for SE2.
         */
        InformedSamplerSE2() noexcept {
            cmin = 1.0;
            matC[0] = matC[1] = matC[2] = matC[3] = matC[4] = matC[5] = matC[6] = matC[7] = matC[8] = 0.0;
            matCL[0] = matCL[1] = matCL[2] = matCL[3] = matCL[4] = matCL[5] = matCL[6] = matCL[7] = matCL[8] = 0.0;
            invMatCL[0] = invMatCL[1] = invMatCL[2] = invMatCL[3] = invMatCL[4] = invMatCL[5] = invMatCL[6] = invMatCL[7] = invMatCL[8] = 0.0;
            center[0] = center[1] = center[2] = 0.0;
            weightYaw = invWeightYaw = 1.0;
        }

        /**
         * @brief Initialize the informed sampler. This will generate look-up tables for random samples.
         * @param[in] maxNumSamples Maximum number of samples to be generated.
         * @param[in] seed Optional seed value for random number generation, defaults to 0.
         */
        void Initialize(size_t maxNumSamples, unsigned seed = 0) noexcept { uniformBallSampler.Initialize(maxNumSamples, seed); }

        /**
         * @brief Reset the sampler.
         * @tparam S Template parameter must be of type std::array<double,N> with N >= 3.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] initialPose Initial pose to be used for the ellipsoid (focal point 1).
         * @param[in] finalPose Final pose to be used for the ellipsoid (focal point 2).
         * @param[in] weightYaw Scalar weighting for yaw angle (position is always weighted with 1), defaults to 1.0.
         * @details This will set the focal points of the ellipsoid.
         */
        template <class S, class T> void Reset(const S& initialPose, const T& finalPose, double weightYaw = 1.0) noexcept {
            static_assert(mpsv::is_vec3<S>::value,"Argument {initialPose} must be of type std::array<double,N> with N >= 3!");
            static_assert(mpsv::is_vec3<T>::value,"Argument {finalPose} must be of type std::array<double,N> with N >= 3!");
            this->weightYaw = std::max(std::fabs(weightYaw), std::numeric_limits<double>::epsilon());
            double a[3];
            matCL[0] = matCL[1] = matCL[2] = matCL[3] = matCL[4] = matCL[5] = matCL[6] = matCL[7] = matCL[8] = 0.0;
            invMatCL[0] = invMatCL[1] = invMatCL[2] = invMatCL[3] = invMatCL[4] = invMatCL[5] = invMatCL[6] = invMatCL[7] = invMatCL[8] = 0.0;
            a[0] = finalPose[0] - initialPose[0];
            a[1] = finalPose[1] - initialPose[1];
            a[2] = this->weightYaw * mpsv::math::SymmetricalAngle(finalPose[2] - initialPose[2]);
            cmin = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            if(cmin > 0.0){
                double n = 1.0 / cmin;
                double U[9], V[9], M[9];
                M[0] = a[0] * n;  M[3] = 0.0;  M[6] = 0.0;
                M[1] = a[1] * n;  M[4] = 0.0;  M[7] = 0.0;
                M[2] = a[2] * n;  M[5] = 0.0;  M[8] = 0.0;
                mpsv::math::SVD3(U, V, M);
                double detUdetV = (U[0]*U[4]*U[8] - U[0]*U[5]*U[7] - U[1]*U[3]*U[8] + U[1]*U[5]*U[6] + U[2]*U[3]*U[7] - U[2]*U[4]*U[6]) * (V[0]*V[4]*V[8] - V[0]*V[5]*V[7] - V[1]*V[3]*V[8] + V[1]*V[5]*V[6] + V[2]*V[3]*V[7] - V[2]*V[4]*V[6]);
                matC[0] = U[0]*V[0] + U[3]*V[3] + detUdetV*U[6]*V[6];
                matC[1] = U[0]*V[1] + U[3]*V[4] + detUdetV*U[6]*V[7];
                matC[2] = U[0]*V[2] + U[3]*V[5] + detUdetV*U[6]*V[8];
                matC[3] = U[1]*V[0] + U[4]*V[3] + detUdetV*U[7]*V[6];
                matC[4] = U[1]*V[1] + U[4]*V[4] + detUdetV*U[7]*V[7];
                matC[5] = U[1]*V[2] + U[4]*V[5] + detUdetV*U[7]*V[8];
                matC[6] = U[2]*V[0] + U[5]*V[3] + detUdetV*U[8]*V[6];
                matC[7] = U[2]*V[1] + U[5]*V[4] + detUdetV*U[8]*V[7];
                matC[8] = U[2]*V[2] + U[5]*V[5] + detUdetV*U[8]*V[8];
            }
            else{
                matC[0] = matC[4] = matC[8] = 1.0;
                matC[1] = matC[2] = matC[3] = matC[5] = matC[6] = matC[7] = 0.0;
            }
            center[0] = 0.5 * (initialPose[0] + finalPose[0]);
            center[1] = 0.5 * (initialPose[1] + finalPose[1]);
            center[2] = mpsv::math::SymmetricalAngle(initialPose[2] + 0.5 * mpsv::math::SymmetricalAngle(finalPose[2] - initialPose[2]));
            this->invWeightYaw = 1.0 / this->weightYaw;
        }

        /**
         * @brief Update the region of the ellipsoid based on the current best cost.
         * @param[in] bestCost The best cost according to the cost function sqrt{(dx)^2 + (dy)^2 + (w*dyaw)^2} where w indicates the weight for the yaw angle.
         * @details This will set the size of the ellipsoid.
         */
        void UpdateRegion(double bestCost) noexcept {
            bestCost = std::max(cmin, bestCost);
            constexpr double minEllipsoidAxis = 100 * std::numeric_limits<double>::epsilon();
            double lMajor = std::max(0.5 * bestCost, minEllipsoidAxis);
            double lMinor = std::max(0.5 * std::sqrt(bestCost*bestCost - cmin*cmin), minEllipsoidAxis);
            double invLMajor = 1.0 / lMajor;
            double invLMinor = 1.0 / lMinor;
            double lMinorW = lMinor * invWeightYaw;
            double invLMinorW = invLMinor * weightYaw;
            matCL[0] = matC[0] * lMajor;
            matCL[1] = matC[1] * lMinor;
            matCL[2] = matC[2] * lMinor;
            matCL[3] = matC[3] * lMajor;
            matCL[4] = matC[4] * lMinor;
            matCL[5] = matC[5] * lMinor;
            matCL[6] = matC[6] * lMajor * invWeightYaw;
            matCL[7] = matC[7] * lMinorW;
            matCL[8] = matC[8] * lMinorW;
            invMatCL[0] = matC[0] * invLMajor;
            invMatCL[1] = matC[3] * invLMajor;
            invMatCL[2] = matC[6] * invLMajor * weightYaw;
            invMatCL[3] = matC[1] * invLMinor;
            invMatCL[4] = matC[4] * invLMinor;
            invMatCL[5] = matC[7] * invLMinorW;
            invMatCL[6] = matC[2] * invLMinor;
            invMatCL[7] = matC[5] * invLMinor;
            invMatCL[8] = matC[8] * invLMinorW;
        }

        /**
         * @brief Get a 3 dimensional sample in the current ellipsoid region.
         * @param[in] idxSample Index of the sample to obtain.
         * @details Make sure you called @ref Initialize to generate look-up table of samples. Call @ref Reset first to set focal points of ellipsoid. Call @ref UpdateRegion at least once to set size of ellipsoid region.
         * @return Resulting sample inside the ellipsoid region.
         * @note IMPORTANT: THE INDEX VALUES ACCESS THE INTERNAL CONTAINER DIRECTLY. MAKE SURE THE VALUES ARE IN RANGE TO PREVENT ACCESS VIOLATION!
         */
        std::array<double,3> Sample(size_t idxSample) noexcept {
            std::array<double,3> tmp = uniformBallSampler.Sample(idxSample);
            std::array<double,3> out;
            out[0] = (matCL[0] * tmp[0] + matCL[1] * tmp[1] + matCL[2] * tmp[2]) + center[0];
            out[1] = (matCL[3] * tmp[0] + matCL[4] * tmp[1] + matCL[5] * tmp[2]) + center[1];
            out[2] = mpsv::math::SymmetricalAngle((matCL[6] * tmp[0] + matCL[7] * tmp[1] + matCL[8] * tmp[2]) + center[2]);
            return out;
        }

        /**
         * @brief Check whether a sample is inside the ellipsoid region or not.
         * @param[in] sample The 3D sample to check.
         * @return True if sample is inside region (including boundary), false otherwise.
         */
        bool IsInsideRegion(const std::array<double,3>& sample) noexcept {
            double x = sample[0] - center[0];
            double y = sample[1] - center[1];
            double z = mpsv::math::SymmetricalAngle(sample[2] - center[2]);
            double nx = invMatCL[0] * x + invMatCL[1] * y + invMatCL[2] * z;
            double ny = invMatCL[3] * x + invMatCL[4] * y + invMatCL[5] * z;
            double nz = invMatCL[6] * x + invMatCL[7] * y + invMatCL[8] * z;
            return ((nx*nx + ny*ny + nz*nz) <= 1.0);
        }

    protected:
        mpsv::sampler::UniformBallSampler3D uniformBallSampler;   // Internal sampler to generate uniform numbers in a 3D ball.
        double cmin;                                              // Internal storage for minimum cost.
        double matC[9];                                           // Ellipsoid rotation matrix C (row-major order).
        double matCL[9];                                          // Ellipsoid rotation matrix C left-multiplied by diagonal ellipsoid dimension matrix L (row-major order). Weighting for yaw is already taken into account.
        double invMatCL[9];                                       // Inverted matrix (C*L)^(-1) (row-major order). Weighting for yaw is already taken into account.
        double center[3];                                         // Center position (x, y, yaw).
        double weightYaw;                                         // Weight for yaw angle used to generate virtual position coordinate z = weight * yaw.
        double invWeightYaw;                                      // Inverse of @ref weightYaw used to recalculate yaw = invWeight * z.
};


} /* namespace: sampler */


} /* namespace: mpsv */

