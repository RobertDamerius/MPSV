#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv::control {


/**
 * @brief The vehicle simulator is used to simulate the vehicle motion in combination with a control and guidance law.
 * The non-linear vehicle model is given by:
 *     (x_dot)     ( cos(psi)  -sin(psi)    0 )   ( u )
 *     (y_dot)   = ( sin(psi)   cos(psi)    0 ) * ( v )
 *     (psi_dot)   (    0          0        1 )   ( r )
 * 
 *     (u_dot)           ( u )        (v*r)        (u^2)        (u^3)       ( X )
 *     (v_dot)   =  F1 * ( v ) + F2 * (u*r) + F3 * (v^2) + F4 * (v^3) + B * ( Y )
 *     (r_dot)           ( r )        (u*v)        (r^2)        (r^3)       ( N )
 * 
 *     (X_dot)      ( -1/TX   0     0   )   ( X )   ( 1/TX  0    0   )   ( Xc )
 *     (Y_dot)   =  (   0   -1/TY   0   ) * ( Y ) + (  0   1/TY  0   ) * ( Yc )
 *     (N_dot)      (   0     0   -1/TN )   ( N )   (  0    0   1/TN )   ( Nc )
 * 
 * The velocity vector is known as nu = (u,v,r)^T. The matrices Fi are concatenated to a matrix F = [F1,F2,F3,F4],
 * such that nu_dot = F * n(nu), with n(nu) = (u,v,r,v*r,u*r,u*v,u^2,v^2,r^2,u^3,v^3,r^3)^T.
 * To prevent steps in the input signal (Xc, Yc, Nc), an additional input filter is added to the system. This input
 * filter is given by:
 *     (Xc_dot)      ( -1/Tf1   0      0    )   ( Xc )   ( 1/Tf1  0     0    )   ( uX )
 *     (Yc_dot)   =  (   0    -1/Tf2   0    ) * ( Yc ) + (  0    1/Tf2  0    ) * ( uY )
 *     (Nc_dot)      (   0      0    -1/Tf3 )   ( Nc )   (  0     0    1/Tf3 )   ( uN )
 */
class VehicleSimulator {
    public:
        /**
         * @brief Set parameters for the dynamic model and the control system.
         * @param[in] modelParameter The model parameter to be set.
         * @param[in] controllerParameter The controller parameter to be set.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code SetParameter(const mpsv::planner::ParameterModel& modelParameter, const mpsv::planner::ParameterController& controllerParameter) noexcept {
            error_code e = modelParameter.IsValid();
            if (error_code::NONE != e) {
                return e;
            }
            e = controllerParameter.IsValid();
            if (error_code::NONE != e) {
                return e;
            }

            // Calculate and check matrix inverse
            auto [success, invMatB] = mpsv::math::MatrixInverse3x3(modelParameter.matB);
            if (!(success &&
                  std::isfinite(invMatB[0U]) &&
                  std::isfinite(invMatB[1U]) &&
                  std::isfinite(invMatB[2U]) &&
                  std::isfinite(invMatB[3U]) &&
                  std::isfinite(invMatB[4U]) &&
                  std::isfinite(invMatB[5U]) &&
                  std::isfinite(invMatB[6U]) &&
                  std::isfinite(invMatB[7U]) &&
                  std::isfinite(invMatB[8U]))) {
                return error_code::PARAM_MODEL_INV_MATB;
            }

            // Set internal parameters
            m_F = modelParameter.matF;
            m_B = modelParameter.matB;
            m_invB.swap(invMatB);
            m_lowerLimitXYN = modelParameter.lowerLimitXYN;
            m_upperLimitXYN = modelParameter.upperLimitXYN;

            // Precalculate parameters
            m_invTXYN[0U] = 1.0 / modelParameter.vecTimeconstantsXYN[0U];
            m_invTXYN[1U] = 1.0 / modelParameter.vecTimeconstantsXYN[1U];
            m_invTXYN[2U] = 1.0 / modelParameter.vecTimeconstantsXYN[2U];
            m_invTf123[0U] = 1.0 / modelParameter.vecTimeconstantsInput[0U];
            m_invTf123[1U] = 1.0 / modelParameter.vecTimeconstantsInput[1U];
            m_invTf123[2U] = 1.0 / modelParameter.vecTimeconstantsInput[2U];
            double TXTf1{modelParameter.vecTimeconstantsXYN[0U] * modelParameter.vecTimeconstantsInput[0U]};
            double TYTf2{modelParameter.vecTimeconstantsXYN[1U] * modelParameter.vecTimeconstantsInput[1U]};
            double TNTf3{modelParameter.vecTimeconstantsXYN[2U] * modelParameter.vecTimeconstantsInput[2U]};
            m_invBBB[0U] = TXTf1 * m_invB[0U];
            m_invBBB[1U] = TXTf1 * m_invB[1U];
            m_invBBB[2U] = TXTf1 * m_invB[2U];
            m_invBBB[3U] = TYTf2 * m_invB[3U];
            m_invBBB[4U] = TYTf2 * m_invB[4U];
            m_invBBB[5U] = TYTf2 * m_invB[5U];
            m_invBBB[6U] = TNTf3 * m_invB[6U];
            m_invBBB[7U] = TNTf3 * m_invB[7U];
            m_invBBB[8U] = TNTf3 * m_invB[8U];
            m_M0[0U] = -m_B[0U] / modelParameter.vecTimeconstantsXYN[0U];
            m_M0[1U] = -m_B[1U] / modelParameter.vecTimeconstantsXYN[1U];
            m_M0[2U] = -m_B[2U] / modelParameter.vecTimeconstantsXYN[2U];
            m_M0[3U] = -m_B[3U] / modelParameter.vecTimeconstantsXYN[0U];
            m_M0[4U] = -m_B[4U] / modelParameter.vecTimeconstantsXYN[1U];
            m_M0[5U] = -m_B[5U] / modelParameter.vecTimeconstantsXYN[2U];
            m_M0[6U] = -m_B[6U] / modelParameter.vecTimeconstantsXYN[0U];
            m_M0[7U] = -m_B[7U] / modelParameter.vecTimeconstantsXYN[1U];
            m_M0[8U] = -m_B[8U] / modelParameter.vecTimeconstantsXYN[2U];
            m_f17_2 = 2.0 * m_F[6U];
            m_f18_2 = 2.0 * m_F[7U];
            m_f19_2 = 2.0 * m_F[8U];
            m_f1a_3 = 3.0 * m_F[9U];
            m_f1b_3 = 3.0 * m_F[10U];
            m_f1c_3 = 3.0 * m_F[11U];
            m_f27_2 = 2.0 * m_F[18U];
            m_f28_2 = 2.0 * m_F[19U];
            m_f29_2 = 2.0 * m_F[20U];
            m_f2a_3 = 3.0 * m_F[21U];
            m_f2b_3 = 3.0 * m_F[22U];
            m_f2c_3 = 3.0 * m_F[23U];
            m_f37_2 = 2.0 * m_F[30U];
            m_f38_2 = 2.0 * m_F[31U];
            m_f39_2 = 2.0 * m_F[32U];
            m_f3a_3 = 3.0 * m_F[33U];
            m_f3b_3 = 3.0 * m_F[34U];
            m_f3c_3 = 3.0 * m_F[35U];
            m_f1a_6 = 6.0 * m_F[9U];
            m_f1b_6 = 6.0 * m_F[10U];
            m_f1c_6 = 6.0 * m_F[11U];
            m_f2a_6 = 6.0 * m_F[21U];
            m_f2b_6 = 6.0 * m_F[22U];
            m_f2c_6 = 6.0 * m_F[23U];
            m_f3a_6 = 6.0 * m_F[33U];
            m_f3b_6 = 6.0 * m_F[34U];
            m_f3c_6 = 6.0 * m_F[35U];

            // Set controller parameters
            m_K = controllerParameter.matK;
            m_minSquaredRadiusPosition = controllerParameter.minRadiusPosition * controllerParameter.minRadiusPosition;
            m_invSquaredRadiusX = 1.0 / (controllerParameter.maxRadiusX * controllerParameter.maxRadiusX);
            m_invSquaredRadiusY = 1.0 / (controllerParameter.maxRadiusY * controllerParameter.maxRadiusY);
            m_invSquaredRadiusPsi = 1.0 / (controllerParameter.maxRadiusPsi * controllerParameter.maxRadiusPsi);
            return error_code::NONE;
        }

        /**
         * @brief Explore a path completely to the end by using the path guidance algorithm and a pose controller. The trajectory is simulated until the regionOfAttraction is entered AND the minNumSimulationSteps is exceeded.
         * @param[out] trajectory The full trajectory that has been simulated. It contains {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc} per data point. The initial state and input is not inserted.
         * @param[in] initialStateAndInput The initial body-fixed state {x,y,psi,u,v,r,X,Y,N} and input {Xc,Yc,Nc}.
         * @param[in] path The input path to be used for exploration. Each point indicates {x,y,psi}.
         * @param[in] regionOfAttraction The region of attraction to be used to check if the state has entered the final pose of the path.
         * @param[in] staticObstacles List of static obstacles to be used by the guidance algorithm to limit the exploration radius near obstacles.
         * @param[in] sampletime The sampletime to be used for forward simulation.
         * @param[in] minNumSimulationSteps The minimum number of simulation steps to be computed (must be at least 1 and at most 0x0000FFFF), defaults to 1.
         * @return The number of simulation steps that have been performed to reach the goal. The maximum value is 0x0000FFFF. To prevent an infinite loop, the maximum number of iterations is fixed to a high value.
         * @details The explicit euler method is used for forward simulation.
         */
        [[nodiscard]] std::uint32_t ExplorePathToEnd(std::vector<std::array<double, 12U>>& trajectory, const std::array<double, 12U>& initialStateAndInput, const std::vector<std::array<double, 3U>>& path, mpsv::control::RegionOfAttraction& regionOfAttraction, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, double sampletime, std::uint32_t minNumSimulationSteps = 1U) const noexcept {
            // xu stores the current state x={x,y,psi,u,v,r,X,Y,N} and input u={Xc,Yc,Nc}
            std::array<double, 12U> xu = initialStateAndInput;
            trajectory.clear();

            // Set final pose
            std::array<double, 3U> finalPose = {initialStateAndInput[0U], initialStateAndInput[1U], initialStateAndInput[2U]};
            if (path.size()) {
                finalPose = path.back();
            }

            // Simulate step by step
            double h2{sampletime / 2.0};
            double h6{sampletime / 6.0};
            double c, s, dx, dy;
            std::array<double, 3U> targetPose, deltaPose, uTau, tmp, forcedResponse, allocationModel, compensation, uz;
            std::array<double, 9U> J, S0, S1, z;
            std::array<double, 12U> xuTmp, k1, k2, k3, k4;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            std::uint32_t k{0U};
            std::uint32_t kmin{static_cast<std::uint32_t>(std::max(minNumSimulationSteps, static_cast<std::uint32_t>(1U)))};
            for (; k < 0x0000FFFFU; ++k) {
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check if termination condition is satisfied (minimum number of steps and state inside region of attraction)
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if ((k >= kmin) && regionOfAttraction.IsInside(xu, finalPose)) {
                    break;
                }


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Target pose for pose controller according to waypoint guidance law
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                c = std::cos(xu[2U]);
                s = std::sin(xu[2U]);
                PathGuidance(targetPose, {xu[0U], xu[1U], xu[2U]}, c, s,  path, staticObstacles);


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Feedback linearization: calculate useful model terms and compensation
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // forced response velocity model: f(nu) = F*n(nu) + B*tau, where n(nu) = (u,v,r,v*r,u*r,u*v,u^2,v^2,r^2,u^3,v^3,r^3)^T
                vr = xu[4U] * xu[5U];
                ur = xu[3U] * xu[5U];
                uv = xu[3U] * xu[4U];
                uu = xu[3U] * xu[3U];
                vv = xu[4U] * xu[4U];
                rr = xu[5U] * xu[5U];
                uuu = uu * xu[3U];
                vvv = vv * xu[4U];
                rrr = rr * xu[5U];
                forcedResponse[0U] =  m_F[0U]*xu[3U] +  m_F[1U]*xu[4U] +  m_F[2U]*xu[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*xu[6U] + m_B[1U]*xu[7U] + m_B[2U]*xu[8U];
                forcedResponse[1U] = m_F[12U]*xu[3U] + m_F[13U]*xu[4U] + m_F[14U]*xu[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*xu[6U] + m_B[4U]*xu[7U] + m_B[5U]*xu[8U];
                forcedResponse[2U] = m_F[24U]*xu[3U] + m_F[25U]*xu[4U] + m_F[26U]*xu[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*xu[6U] + m_B[7U]*xu[7U] + m_B[8U]*xu[8U];

                // forced response allocation model: A_tau * tau + B_tau * tau_c
                allocationModel[0U] = (xu[9U] - xu[6U]) * m_invTXYN[0U];
                allocationModel[1U] = (xu[10U] - xu[7U]) * m_invTXYN[1U];
                allocationModel[2U] = (xu[11U] - xu[8U]) * m_invTXYN[2U];

                // Jacobian J = df(nu)/dnu (uvr model w.r.t. uvr)
                J[0U] =  m_F[0U] +  m_F[4U]*xu[5U] +  m_F[5U]*xu[4U] + m_f17_2*xu[3U] + m_f1a_3*uu;
                J[1U] =  m_F[1U] +  m_F[3U]*xu[5U] +  m_F[5U]*xu[3U] + m_f18_2*xu[4U] + m_f1b_3*vv;
                J[2U] =  m_F[2U] +  m_F[3U]*xu[4U] +  m_F[4U]*xu[3U] + m_f19_2*xu[5U] + m_f1c_3*rr;
                J[3U] = m_F[12U] + m_F[16U]*xu[5U] + m_F[17U]*xu[4U] + m_f27_2*xu[3U] + m_f2a_3*uu;
                J[4U] = m_F[13U] + m_F[15U]*xu[5U] + m_F[17U]*xu[3U] + m_f28_2*xu[4U] + m_f2b_3*vv;
                J[5U] = m_F[14U] + m_F[15U]*xu[4U] + m_F[16U]*xu[3U] + m_f29_2*xu[5U] + m_f2c_3*rr;
                J[6U] = m_F[24U] + m_F[28U]*xu[5U] + m_F[29U]*xu[4U] + m_f37_2*xu[3U] + m_f3a_3*uu;
                J[7U] = m_F[25U] + m_F[27U]*xu[5U] + m_F[29U]*xu[3U] + m_f38_2*xu[4U] + m_f3b_3*vv;
                J[8U] = m_F[26U] + m_F[27U]*xu[4U] + m_F[28U]*xu[3U] + m_f39_2*xu[5U] + m_f3c_3*rr;

                // S0 matrix: J^2 + T, where t_m1 = (dj_m1/du, dj_m2/du, dj_m2/du)*forcedResponse, t_m2 = (dj_m1/dv, dj_m2/dv, dj_m2/dv)*forcedResponse, t_m3 = (dj_m1/dr, dj_m2/dr, dj_m2/dr)*forcedResponse for m=1,2,3
                S0[0U] = J[0U]*J[0U] + J[1U]*J[3U] + J[2U]*J[6U] + (m_f17_2 + m_f1a_6*xu[3U])*forcedResponse[0U] +                    m_F[5U]*forcedResponse[1U] +                    m_F[4U]*forcedResponse[2U];
                S0[1U] = J[0U]*J[1U] + J[1U]*J[4U] + J[2U]*J[7U] +                    m_F[5U]*forcedResponse[0U] + (m_f18_2 + m_f1b_6*xu[4U])*forcedResponse[1U] +                    m_F[3U]*forcedResponse[2U];
                S0[2U] = J[0U]*J[2U] + J[1U]*J[5U] + J[2U]*J[8U] +                    m_F[4U]*forcedResponse[0U] +                    m_F[3U]*forcedResponse[1U] + (m_f19_2 + m_f1c_6*xu[5U])*forcedResponse[2U];
                S0[3U] = J[3U]*J[0U] + J[4U]*J[3U] + J[5U]*J[6U] + (m_f27_2 + m_f2a_6*xu[3U])*forcedResponse[0U] +                   m_F[17U]*forcedResponse[1U] +                   m_F[16U]*forcedResponse[2U];
                S0[4U] = J[3U]*J[1U] + J[4U]*J[4U] + J[5U]*J[7U] +                   m_F[17U]*forcedResponse[0U] + (m_f28_2 + m_f2b_6*xu[4U])*forcedResponse[1U] +                   m_F[15U]*forcedResponse[2U];
                S0[5U] = J[3U]*J[2U] + J[4U]*J[5U] + J[5U]*J[8U] +                   m_F[16U]*forcedResponse[0U] +                   m_F[15U]*forcedResponse[1U] + (m_f29_2 + m_f2c_6*xu[5U])*forcedResponse[2U];
                S0[6U] = J[6U]*J[0U] + J[7U]*J[3U] + J[8U]*J[6U] + (m_f37_2 + m_f3a_6*xu[3U])*forcedResponse[0U] +                   m_F[29U]*forcedResponse[1U] +                   m_F[28U]*forcedResponse[2U];
                S0[7U] = J[6U]*J[1U] + J[7U]*J[4U] + J[8U]*J[7U] +                   m_F[29U]*forcedResponse[0U] + (m_f38_2 + m_f3b_6*xu[4U])*forcedResponse[1U] +                   m_F[27U]*forcedResponse[2U];
                S0[8U] = J[6U]*J[2U] + J[7U]*J[5U] + J[8U]*J[8U] +                   m_F[28U]*forcedResponse[0U] +                   m_F[27U]*forcedResponse[1U] + (m_f39_2 + m_f3c_6*xu[5U])*forcedResponse[2U];

                // S1 matrix: J * B
                S1[0U] = J[0U]*m_B[0U] + J[1U]*m_B[3U] + J[2U]*m_B[6U];
                S1[1U] = J[0U]*m_B[1U] + J[1U]*m_B[4U] + J[2U]*m_B[7U];
                S1[2U] = J[0U]*m_B[2U] + J[1U]*m_B[5U] + J[2U]*m_B[8U];
                S1[3U] = J[3U]*m_B[0U] + J[4U]*m_B[3U] + J[5U]*m_B[6U];
                S1[4U] = J[3U]*m_B[1U] + J[4U]*m_B[4U] + J[5U]*m_B[7U];
                S1[5U] = J[3U]*m_B[2U] + J[4U]*m_B[5U] + J[5U]*m_B[8U];
                S1[6U] = J[6U]*m_B[0U] + J[7U]*m_B[3U] + J[8U]*m_B[6U];
                S1[7U] = J[6U]*m_B[1U] + J[7U]*m_B[4U] + J[8U]*m_B[7U];
                S1[8U] = J[6U]*m_B[2U] + J[7U]*m_B[5U] + J[8U]*m_B[8U];

                // flat state vector
                z[0U] = xu[3U];
                z[1U] = xu[4U];
                z[2U] = xu[5U];
                z[3U] = forcedResponse[0U];
                z[4U] = forcedResponse[1U];
                z[5U] = forcedResponse[2U];
                z[6U] = J[0U]*forcedResponse[0U] + J[1U]*forcedResponse[1U] + J[2U]*forcedResponse[2U] + m_B[0U]*allocationModel[0U] + m_B[1U]*allocationModel[1U] + m_B[2U]*allocationModel[2U];
                z[7U] = J[3U]*forcedResponse[0U] + J[4U]*forcedResponse[1U] + J[5U]*forcedResponse[2U] + m_B[3U]*allocationModel[0U] + m_B[4U]*allocationModel[1U] + m_B[5U]*allocationModel[2U];
                z[8U] = J[6U]*forcedResponse[0U] + J[7U]*forcedResponse[1U] + J[8U]*forcedResponse[2U] + m_B[6U]*allocationModel[0U] + m_B[7U]*allocationModel[1U] + m_B[8U]*allocationModel[2U];

                // compensation = S0(uvr) * forcedResponse + S1(uvr) * allocationModel + M0 * (allocationModel + Bc*tau_c)
                tmp[0U] = allocationModel[0U] + xu[9U] * m_invTf123[0U];
                tmp[1U] = allocationModel[1U] + xu[10U] * m_invTf123[1U];
                tmp[2U] = allocationModel[2U] + xu[11U] * m_invTf123[2U];
                compensation[0U] = S0[0U]*forcedResponse[0U] + S0[1U]*forcedResponse[1U] + S0[2U]*forcedResponse[2U] + S1[0U]*allocationModel[0U] + S1[1U]*allocationModel[1U] + S1[2U]*allocationModel[2U] + m_M0[0U]*tmp[0U] + m_M0[1U]*tmp[1U] + m_M0[2U]*tmp[2U];
                compensation[1U] = S0[3U]*forcedResponse[0U] + S0[4U]*forcedResponse[1U] + S0[5U]*forcedResponse[2U] + S1[3U]*allocationModel[0U] + S1[4U]*allocationModel[1U] + S1[5U]*allocationModel[2U] + m_M0[3U]*tmp[0U] + m_M0[4U]*tmp[1U] + m_M0[5U]*tmp[2U];
                compensation[2U] = S0[6U]*forcedResponse[0U] + S0[7U]*forcedResponse[1U] + S0[8U]*forcedResponse[2U] + S1[6U]*allocationModel[0U] + S1[7U]*allocationModel[1U] + S1[8U]*allocationModel[2U] + m_M0[6U]*tmp[0U] + m_M0[7U]*tmp[1U] + m_M0[8U]*tmp[2U];


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Pose controller (flat state controller with respect to target pose as origin)
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                dx = xu[0U] - targetPose[0U];
                dy = xu[1U] - targetPose[1U];
                deltaPose[0U] = c*dx + s*dy;
                deltaPose[1U] =-s*dx + c*dy;
                deltaPose[2U] = mpsv::math::SymmetricalAngle(xu[2U] - targetPose[2U]);

                // Calculate the flat state control command using the control law uz = -K * [deltaPose; z1; z2; z3]
                uz[0U] =  -m_K[0U]*deltaPose[0U] -  m_K[1U]*deltaPose[1U] -  m_K[2U]*deltaPose[2U] -  m_K[3U]*z[0U] -  m_K[4U]*z[1U] -  m_K[5U]*z[2U] -  m_K[6U]*z[3U] -  m_K[7U]*z[4U] -  m_K[8U]*z[5U] -  m_K[9U]*z[6U] - m_K[10U]*z[7U] - m_K[11U]*z[8U];
                uz[1U] = -m_K[12U]*deltaPose[0U] - m_K[13U]*deltaPose[1U] - m_K[14U]*deltaPose[2U] - m_K[15U]*z[0U] - m_K[16U]*z[1U] - m_K[17U]*z[2U] - m_K[18U]*z[3U] - m_K[19U]*z[4U] - m_K[20U]*z[5U] - m_K[21U]*z[6U] - m_K[22U]*z[7U] - m_K[23U]*z[8U];
                uz[2U] = -m_K[24U]*deltaPose[0U] - m_K[25U]*deltaPose[1U] - m_K[26U]*deltaPose[2U] - m_K[27U]*z[0U] - m_K[28U]*z[1U] - m_K[29U]*z[2U] - m_K[30U]*z[3U] - m_K[31U]*z[4U] - m_K[32U]*z[5U] - m_K[33U]*z[6U] - m_K[34U]*z[7U] - m_K[35U]*z[8U];

                // final control law uTau = invBBB * (uz - compensation)
                tmp[0U] = uz[0U] - compensation[0U];
                tmp[1U] = uz[1U] - compensation[1U];
                tmp[2U] = uz[2U] - compensation[2U];
                uTau[0U] = m_invBBB[0U]*tmp[0U] + m_invBBB[1U]*tmp[1U] + m_invBBB[2U]*tmp[2U];
                uTau[1U] = m_invBBB[3U]*tmp[0U] + m_invBBB[4U]*tmp[1U] + m_invBBB[5U]*tmp[2U];
                uTau[2U] = m_invBBB[6U]*tmp[0U] + m_invBBB[7U]*tmp[1U] + m_invBBB[8U]*tmp[2U];

                // saturate control input
                uTau[0U] = std::clamp(uTau[0U], m_lowerLimitXYN[0U], m_upperLimitXYN[0U]);
                uTau[1U] = std::clamp(uTau[1U], m_lowerLimitXYN[1U], m_upperLimitXYN[1U]);
                uTau[2U] = std::clamp(uTau[2U], m_lowerLimitXYN[2U], m_upperLimitXYN[2U]);


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Non-linear Model: numerical integration via RK4
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // RK4: k1
                k1[0U]  = c*xu[3U] - s*xu[4U];
                k1[1U]  = s*xu[3U] + c*xu[4U];
                k1[2U]  = xu[5U];
                k1[3U]  = forcedResponse[0U];
                k1[4U]  = forcedResponse[1U];
                k1[5U]  = forcedResponse[2U];
                k1[6U]  = allocationModel[0U];
                k1[7U]  = allocationModel[1U];
                k1[8U]  = allocationModel[2U];
                k1[9U]  = (uTau[0U] - xu[9U]) * m_invTf123[0U];
                k1[10U] = (uTau[1U] - xu[10U]) * m_invTf123[1U];
                k1[11U] = (uTau[2U] - xu[11U]) * m_invTf123[2U];

                // RK4: k2
                xuTmp[0U]  = xu[0U]  + h2 * k1[0U];
                xuTmp[1U]  = xu[1U]  + h2 * k1[1U];
                xuTmp[2U]  = xu[2U]  + h2 * k1[2U];
                xuTmp[3U]  = xu[3U]  + h2 * k1[3U];
                xuTmp[4U]  = xu[4U]  + h2 * k1[4U];
                xuTmp[5U]  = xu[5U]  + h2 * k1[5U];
                xuTmp[6U]  = xu[6U]  + h2 * k1[6U];
                xuTmp[7U]  = xu[7U]  + h2 * k1[7U];
                xuTmp[8U]  = xu[8U]  + h2 * k1[8U];
                xuTmp[9U]  = xu[9U]  + h2 * k1[9U];
                xuTmp[10U] = xu[10U] + h2 * k1[10U];
                xuTmp[11U] = xu[11U] + h2 * k1[11U];
                c = std::cos(xuTmp[2U]);
                s = std::sin(xuTmp[2U]);
                uv = xuTmp[3U] * xuTmp[4U];
                ur = xuTmp[3U] * xuTmp[5U];
                vr = xuTmp[4U] * xuTmp[5U];
                uu = xuTmp[3U] * xuTmp[3U];
                vv = xuTmp[4U] * xuTmp[4U];
                rr = xuTmp[5U] * xuTmp[5U];
                uuu = uu * xuTmp[3U];
                vvv = vv * xuTmp[4U];
                rrr = rr * xuTmp[5U];
                k2[0U]  = c*xuTmp[3U] - s*xuTmp[4U];
                k2[1U]  = s*xuTmp[3U] + c*xuTmp[4U];
                k2[2U]  = xuTmp[5U];
                k2[3U]  =  m_F[0U]*xuTmp[3U] +  m_F[1U]*xuTmp[4U] +  m_F[2U]*xuTmp[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*xuTmp[6U] + m_B[1U]*xuTmp[7U] + m_B[2U]*xuTmp[8U];
                k2[4U]  = m_F[12U]*xuTmp[3U] + m_F[13U]*xuTmp[4U] + m_F[14U]*xuTmp[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*xuTmp[6U] + m_B[4U]*xuTmp[7U] + m_B[5U]*xuTmp[8U];
                k2[5U]  = m_F[24U]*xuTmp[3U] + m_F[25U]*xuTmp[4U] + m_F[26U]*xuTmp[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*xuTmp[6U] + m_B[7U]*xuTmp[7U] + m_B[8U]*xuTmp[8U];
                k2[6U]  = (xuTmp[9U]  - xuTmp[6U]) * m_invTXYN[0U];
                k2[7U]  = (xuTmp[10U] - xuTmp[7U]) * m_invTXYN[1U];
                k2[8U]  = (xuTmp[11U] - xuTmp[8U]) * m_invTXYN[2U];
                k2[9U]  = (uTau[0U] - xuTmp[9U]) * m_invTf123[0U];
                k2[10U] = (uTau[1U] - xuTmp[10U]) * m_invTf123[1U];
                k2[11U] = (uTau[2U] - xuTmp[11U]) * m_invTf123[2U];

                // RK4: k3
                xuTmp[0U]  = xu[0U]  + h2 * k2[0U];
                xuTmp[1U]  = xu[1U]  + h2 * k2[1U];
                xuTmp[2U]  = xu[2U]  + h2 * k2[2U];
                xuTmp[3U]  = xu[3U]  + h2 * k2[3U];
                xuTmp[4U]  = xu[4U]  + h2 * k2[4U];
                xuTmp[5U]  = xu[5U]  + h2 * k2[5U];
                xuTmp[6U]  = xu[6U]  + h2 * k2[6U];
                xuTmp[7U]  = xu[7U]  + h2 * k2[7U];
                xuTmp[8U]  = xu[8U]  + h2 * k2[8U];
                xuTmp[9U]  = xu[9U]  + h2 * k2[9U];
                xuTmp[10U] = xu[10U] + h2 * k2[10U];
                xuTmp[11U] = xu[11U] + h2 * k2[11U];
                c = std::cos(xuTmp[2U]);
                s = std::sin(xuTmp[2U]);
                uv = xuTmp[3U] * xuTmp[4U];
                ur = xuTmp[3U] * xuTmp[5U];
                vr = xuTmp[4U] * xuTmp[5U];
                uu = xuTmp[3U] * xuTmp[3U];
                vv = xuTmp[4U] * xuTmp[4U];
                rr = xuTmp[5U] * xuTmp[5U];
                uuu = uu * xuTmp[3U];
                vvv = vv * xuTmp[4U];
                rrr = rr * xuTmp[5U];
                k3[0U]  = c*xuTmp[3U] - s*xuTmp[4U];
                k3[1U]  = s*xuTmp[3U] + c*xuTmp[4U];
                k3[2U]  = xuTmp[5U];
                k3[3U]  =  m_F[0U]*xuTmp[3U] +  m_F[1U]*xuTmp[4U] +  m_F[2U]*xuTmp[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*xuTmp[6U] + m_B[1U]*xuTmp[7U] + m_B[2U]*xuTmp[8U];
                k3[4U]  = m_F[12U]*xuTmp[3U] + m_F[13U]*xuTmp[4U] + m_F[14U]*xuTmp[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*xuTmp[6U] + m_B[4U]*xuTmp[7U] + m_B[5U]*xuTmp[8U];
                k3[5U]  = m_F[24U]*xuTmp[3U] + m_F[25U]*xuTmp[4U] + m_F[26U]*xuTmp[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*xuTmp[6U] + m_B[7U]*xuTmp[7U] + m_B[8U]*xuTmp[8U];
                k3[6U]  = (xuTmp[9U]  - xuTmp[6U]) * m_invTXYN[0U];
                k3[7U]  = (xuTmp[10U] - xuTmp[7U]) * m_invTXYN[1U];
                k3[8U]  = (xuTmp[11U] - xuTmp[8U]) * m_invTXYN[2U];
                k3[9U]  = (uTau[0U] - xuTmp[9U]) * m_invTf123[0U];
                k3[10U] = (uTau[1U] - xuTmp[10U]) * m_invTf123[1U];
                k3[11U] = (uTau[2U] - xuTmp[11U]) * m_invTf123[2U];

                // RK4: k4
                xuTmp[0U]  = xu[0U]  + sampletime * k3[0U];
                xuTmp[1U]  = xu[1U]  + sampletime * k3[1U];
                xuTmp[2U]  = xu[2U]  + sampletime * k3[2U];
                xuTmp[3U]  = xu[3U]  + sampletime * k3[3U];
                xuTmp[4U]  = xu[4U]  + sampletime * k3[4U];
                xuTmp[5U]  = xu[5U]  + sampletime * k3[5U];
                xuTmp[6U]  = xu[6U]  + sampletime * k3[6U];
                xuTmp[7U]  = xu[7U]  + sampletime * k3[7U];
                xuTmp[8U]  = xu[8U]  + sampletime * k3[8U];
                xuTmp[9U]  = xu[9U]  + sampletime * k3[9U];
                xuTmp[10U] = xu[10U] + sampletime * k3[10U];
                xuTmp[11U] = xu[11U] + sampletime * k3[11U];
                c = std::cos(xuTmp[2U]);
                s = std::sin(xuTmp[2U]);
                uv = xuTmp[3U] * xuTmp[4U];
                ur = xuTmp[3U] * xuTmp[5U];
                vr = xuTmp[4U] * xuTmp[5U];
                uu = xuTmp[3U] * xuTmp[3U];
                vv = xuTmp[4U] * xuTmp[4U];
                rr = xuTmp[5U] * xuTmp[5U];
                uuu = uu * xuTmp[3U];
                vvv = vv * xuTmp[4U];
                rrr = rr * xuTmp[5U];
                k4[0U]  = c*xuTmp[3U] - s*xuTmp[4U];
                k4[1U]  = s*xuTmp[3U] + c*xuTmp[4U];
                k4[2U]  = xuTmp[5U];
                k4[3U]  =  m_F[0U]*xuTmp[3U] +  m_F[1U]*xuTmp[4U] +  m_F[2U]*xuTmp[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*xuTmp[6U] + m_B[1U]*xuTmp[7U] + m_B[2U]*xuTmp[8U];
                k4[4U]  = m_F[12U]*xuTmp[3U] + m_F[13U]*xuTmp[4U] + m_F[14U]*xuTmp[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*xuTmp[6U] + m_B[4U]*xuTmp[7U] + m_B[5U]*xuTmp[8U];
                k4[5U]  = m_F[24U]*xuTmp[3U] + m_F[25U]*xuTmp[4U] + m_F[26U]*xuTmp[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*xuTmp[6U] + m_B[7U]*xuTmp[7U] + m_B[8U]*xuTmp[8U];
                k4[6U]  = (xuTmp[9U]  - xuTmp[6U]) * m_invTXYN[0U];
                k4[7U]  = (xuTmp[10U] - xuTmp[7U]) * m_invTXYN[1U];
                k4[8U]  = (xuTmp[11U] - xuTmp[8U]) * m_invTXYN[2U];
                k4[9U]  = (uTau[0U] - xuTmp[9U]) * m_invTf123[0U];
                k4[10U] = (uTau[1U] - xuTmp[10U]) * m_invTf123[1U];
                k4[11U] = (uTau[2U] - xuTmp[11U]) * m_invTf123[2U];

                // Predicted state according to RK4
                xu[0U]  += h6 * (k1[0U]  + 2.0*(k2[0U]  + k3[0U])  + k4[0U]);
                xu[1U]  += h6 * (k1[1U]  + 2.0*(k2[1U]  + k3[1U])  + k4[1U]);
                xu[2U]  += h6 * (k1[2U]  + 2.0*(k2[2U]  + k3[2U])  + k4[2U]);
                xu[3U]  += h6 * (k1[3U]  + 2.0*(k2[3U]  + k3[3U])  + k4[3U]);
                xu[4U]  += h6 * (k1[4U]  + 2.0*(k2[4U]  + k3[4U])  + k4[4U]);
                xu[5U]  += h6 * (k1[5U]  + 2.0*(k2[5U]  + k3[5U])  + k4[5U]);
                xu[6U]  += h6 * (k1[6U]  + 2.0*(k2[6U]  + k3[6U])  + k4[6U]);
                xu[7U]  += h6 * (k1[7U]  + 2.0*(k2[7U]  + k3[7U])  + k4[7U]);
                xu[8U]  += h6 * (k1[8U]  + 2.0*(k2[8U]  + k3[8U])  + k4[8U]);
                xu[9U]  += h6 * (k1[9U]  + 2.0*(k2[9U]  + k3[9U])  + k4[9U]);
                xu[10U] += h6 * (k1[10U] + 2.0*(k2[10U] + k3[10U]) + k4[10U]);
                xu[11U] += h6 * (k1[11U] + 2.0*(k2[11U] + k3[11U]) + k4[11U]);

                // xu contains (xk,uk), copy them to the trajectory
                xu[2U] = mpsv::math::SymmetricalAngle(xu[2U]);
                trajectory.push_back(xu);
            }

            // Return the number of performed simulation steps
            return k;
        }

        /**
         * @brief Predict the motion of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] sampletime The sampletime to be used for forward simulation.
         * @param[in] numSimulationSteps The number of simulation steps to be computed.
         * @return The predicted state given as {x,y,psi,u,v,r,X,Y,N}.
         */
        [[nodiscard]] std::array<double, 9U> PredictMotion(const std::array<double, 12U>& initialStateAndInput, double sampletime, std::uint32_t numSimulationSteps) const noexcept {
            // Start from the initial state
            std::array<double, 9U> predictedState;
            std::copy(initialStateAndInput.begin(), initialStateAndInput.begin() + 9, predictedState.begin());

            // Simulate step by step
            double h2{sampletime / 2.0};
            double h6{sampletime / 6.0};
            double c, s;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            std::array<double, 9U> x, k1, k2, k3, k4;
            for (std::uint32_t k{0U}; k != numSimulationSteps; ++k) {
                // RK4: k1
                x = predictedState;
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k1[0U] = (c*x[3U] - s*x[4U]);
                k1[1U] = (s*x[3U] + c*x[4U]);
                k1[2U] = (x[5U]);
                k1[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k1[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k1[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k1[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k1[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k1[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k2
                x[0U] = predictedState[0U] + h2 * k1[0U];
                x[1U] = predictedState[1U] + h2 * k1[1U];
                x[2U] = predictedState[2U] + h2 * k1[2U];
                x[3U] = predictedState[3U] + h2 * k1[3U];
                x[4U] = predictedState[4U] + h2 * k1[4U];
                x[5U] = predictedState[5U] + h2 * k1[5U];
                x[6U] = predictedState[6U] + h2 * k1[6U];
                x[7U] = predictedState[7U] + h2 * k1[7U];
                x[8U] = predictedState[8U] + h2 * k1[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k2[0U] = (c*x[3U] - s*x[4U]);
                k2[1U] = (s*x[3U] + c*x[4U]);
                k2[2U] = (x[5U]);
                k2[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k2[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k2[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k2[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k2[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k2[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k3
                x[0U] = predictedState[0U] + h2 * k2[0U];
                x[1U] = predictedState[1U] + h2 * k2[1U];
                x[2U] = predictedState[2U] + h2 * k2[2U];
                x[3U] = predictedState[3U] + h2 * k2[3U];
                x[4U] = predictedState[4U] + h2 * k2[4U];
                x[5U] = predictedState[5U] + h2 * k2[5U];
                x[6U] = predictedState[6U] + h2 * k2[6U];
                x[7U] = predictedState[7U] + h2 * k2[7U];
                x[8U] = predictedState[8U] + h2 * k2[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k3[0U] = (c*x[3U] - s*x[4U]);
                k3[1U] = (s*x[3U] + c*x[4U]);
                k3[2U] = (x[5U]);
                k3[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k3[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k3[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k3[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k3[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k3[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k4
                x[0U] = predictedState[0U] + sampletime * k3[0U];
                x[1U] = predictedState[1U] + sampletime * k3[1U];
                x[2U] = predictedState[2U] + sampletime * k3[2U];
                x[3U] = predictedState[3U] + sampletime * k3[3U];
                x[4U] = predictedState[4U] + sampletime * k3[4U];
                x[5U] = predictedState[5U] + sampletime * k3[5U];
                x[6U] = predictedState[6U] + sampletime * k3[6U];
                x[7U] = predictedState[7U] + sampletime * k3[7U];
                x[8U] = predictedState[8U] + sampletime * k3[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k4[0U] = (c*x[3U] - s*x[4U]);
                k4[1U] = (s*x[3U] + c*x[4U]);
                k4[2U] = (x[5U]);
                k4[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k4[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k4[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k4[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k4[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k4[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // Predicted state according to RK4
                predictedState[0U] += h6 * (k1[0U] + 2.0*(k2[0U] + k3[0U]) + k4[0U]);
                predictedState[1U] += h6 * (k1[1U] + 2.0*(k2[1U] + k3[1U]) + k4[1U]);
                predictedState[2U] += h6 * (k1[2U] + 2.0*(k2[2U] + k3[2U]) + k4[2U]);
                predictedState[3U] += h6 * (k1[3U] + 2.0*(k2[3U] + k3[3U]) + k4[3U]);
                predictedState[4U] += h6 * (k1[4U] + 2.0*(k2[4U] + k3[4U]) + k4[4U]);
                predictedState[5U] += h6 * (k1[5U] + 2.0*(k2[5U] + k3[5U]) + k4[5U]);
                predictedState[6U] += h6 * (k1[6U] + 2.0*(k2[6U] + k3[6U]) + k4[6U]);
                predictedState[7U] += h6 * (k1[7U] + 2.0*(k2[7U] + k3[7U]) + k4[7U]);
                predictedState[8U] += h6 * (k1[8U] + 2.0*(k2[8U] + k3[8U]) + k4[8U]);
            }
            predictedState[2U] = mpsv::math::SymmetricalAngle(predictedState[2U]);
            return predictedState;
        }

        /**
         * @brief Predict the motion trajectory of the vehicle for a given prediction horizon assuming the control input to be constant all the time.
         * @param[out] trajectory The prediction trajectory where each entry contains the state and the input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state/input is not inserted.
         * @param[in] initialStateAndInput The initial state and input vector given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
         * @param[in] sampletime The sampletime to be used for forward simulation.
         * @param[in] numSimulationSteps The number (> 0) of simulation steps to be computed. Internally, at least one simulation step is performed.
         * @param[in] maxNumSimulationSteps The maximum number of simulation steps to be computed.
         * @return True if success, false otherwise. If the maximum number of simulation steps is exceeded, false is returned.
         */
        [[nodiscard]] bool PredictMotionTrajectory(std::vector<std::array<double, 12U>>& trajectory, const std::array<double, 12U>& initialStateAndInput, double sampletime, std::uint32_t numSimulationSteps, const std::uint32_t maxNumSimulationSteps) const noexcept {
            // Start from the initial state
            std::array<double, 9U> predictedState;
            std::copy(initialStateAndInput.begin(), initialStateAndInput.begin() + 9, predictedState.begin());

            // Simulate step by step
            double h2{sampletime / 2.0};
            double h6{sampletime / 6.0};
            double c, s;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            std::array<double, 9U> x, k1, k2, k3, k4;
            trajectory.clear();
            if (numSimulationSteps >= maxNumSimulationSteps) {
                return false;
            }
            numSimulationSteps = std::max(numSimulationSteps, std::uint32_t{1U});
            trajectory.reserve(numSimulationSteps);
            for (std::uint32_t k{0U}; k != numSimulationSteps; ++k) {
                // RK4: k1
                x = predictedState;
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k1[0U] = (c*x[3U] - s*x[4U]);
                k1[1U] = (s*x[3U] + c*x[4U]);
                k1[2U] = (x[5U]);
                k1[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k1[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k1[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k1[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k1[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k1[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k2
                x[0U] = predictedState[0U] + h2 * k1[0U];
                x[1U] = predictedState[1U] + h2 * k1[1U];
                x[2U] = predictedState[2U] + h2 * k1[2U];
                x[3U] = predictedState[3U] + h2 * k1[3U];
                x[4U] = predictedState[4U] + h2 * k1[4U];
                x[5U] = predictedState[5U] + h2 * k1[5U];
                x[6U] = predictedState[6U] + h2 * k1[6U];
                x[7U] = predictedState[7U] + h2 * k1[7U];
                x[8U] = predictedState[8U] + h2 * k1[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k2[0U] = (c*x[3U] - s*x[4U]);
                k2[1U] = (s*x[3U] + c*x[4U]);
                k2[2U] = (x[5U]);
                k2[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k2[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k2[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k2[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k2[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k2[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k3
                x[0U] = predictedState[0U] + h2 * k2[0U];
                x[1U] = predictedState[1U] + h2 * k2[1U];
                x[2U] = predictedState[2U] + h2 * k2[2U];
                x[3U] = predictedState[3U] + h2 * k2[3U];
                x[4U] = predictedState[4U] + h2 * k2[4U];
                x[5U] = predictedState[5U] + h2 * k2[5U];
                x[6U] = predictedState[6U] + h2 * k2[6U];
                x[7U] = predictedState[7U] + h2 * k2[7U];
                x[8U] = predictedState[8U] + h2 * k2[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k3[0U] = (c*x[3U] - s*x[4U]);
                k3[1U] = (s*x[3U] + c*x[4U]);
                k3[2U] = (x[5U]);
                k3[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k3[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k3[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k3[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k3[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k3[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // RK4: k4
                x[0U] = predictedState[0U] + sampletime * k3[0U];
                x[1U] = predictedState[1U] + sampletime * k3[1U];
                x[2U] = predictedState[2U] + sampletime * k3[2U];
                x[3U] = predictedState[3U] + sampletime * k3[3U];
                x[4U] = predictedState[4U] + sampletime * k3[4U];
                x[5U] = predictedState[5U] + sampletime * k3[5U];
                x[6U] = predictedState[6U] + sampletime * k3[6U];
                x[7U] = predictedState[7U] + sampletime * k3[7U];
                x[8U] = predictedState[8U] + sampletime * k3[8U];
                c = std::cos(x[2U]);
                s = std::sin(x[2U]);
                uv = x[3U] * x[4U];
                ur = x[3U] * x[5U];
                vr = x[4U] * x[5U];
                uu = x[3U] * x[3U];
                vv = x[4U] * x[4U];
                rr = x[5U] * x[5U];
                uuu = uu * x[3U];
                vvv = vv * x[4U];
                rrr = rr * x[5U];
                k4[0U] = (c*x[3U] - s*x[4U]);
                k4[1U] = (s*x[3U] + c*x[4U]);
                k4[2U] = (x[5U]);
                k4[3U] = ( m_F[0U]*x[3U] +  m_F[1U]*x[4U] +  m_F[2U]*x[5U] +  m_F[3U]*vr +  m_F[4U]*ur +  m_F[5U]*uv +  m_F[6U]*uu +  m_F[7U]*vv +  m_F[8U]*rr +  m_F[9U]*uuu + m_F[10U]*vvv + m_F[11U]*rrr + m_B[0U]*x[6U] + m_B[1U]*x[7U] + m_B[2U]*x[8U]);
                k4[4U] = (m_F[12U]*x[3U] + m_F[13U]*x[4U] + m_F[14U]*x[5U] + m_F[15U]*vr + m_F[16U]*ur + m_F[17U]*uv + m_F[18U]*uu + m_F[19U]*vv + m_F[20U]*rr + m_F[21U]*uuu + m_F[22U]*vvv + m_F[23U]*rrr + m_B[3U]*x[6U] + m_B[4U]*x[7U] + m_B[5U]*x[8U]);
                k4[5U] = (m_F[24U]*x[3U] + m_F[25U]*x[4U] + m_F[26U]*x[5U] + m_F[27U]*vr + m_F[28U]*ur + m_F[29U]*uv + m_F[30U]*uu + m_F[31U]*vv + m_F[32U]*rr + m_F[33U]*uuu + m_F[34U]*vvv + m_F[35U]*rrr + m_B[6U]*x[6U] + m_B[7U]*x[7U] + m_B[8U]*x[8U]);
                k4[6U] = (initialStateAndInput[9U]  - x[6U]) * m_invTXYN[0U];
                k4[7U] = (initialStateAndInput[10U] - x[7U]) * m_invTXYN[1U];
                k4[8U] = (initialStateAndInput[11U] - x[8U]) * m_invTXYN[2U];

                // Predicted state according to RK4
                predictedState[0U] += h6 * (k1[0U] + 2.0*(k2[0U] + k3[0U]) + k4[0U]);
                predictedState[1U] += h6 * (k1[1U] + 2.0*(k2[1U] + k3[1U]) + k4[1U]);
                predictedState[2U] += h6 * (k1[2U] + 2.0*(k2[2U] + k3[2U]) + k4[2U]);
                predictedState[3U] += h6 * (k1[3U] + 2.0*(k2[3U] + k3[3U]) + k4[3U]);
                predictedState[4U] += h6 * (k1[4U] + 2.0*(k2[4U] + k3[4U]) + k4[4U]);
                predictedState[5U] += h6 * (k1[5U] + 2.0*(k2[5U] + k3[5U]) + k4[5U]);
                predictedState[6U] += h6 * (k1[6U] + 2.0*(k2[6U] + k3[6U]) + k4[6U]);
                predictedState[7U] += h6 * (k1[7U] + 2.0*(k2[7U] + k3[7U]) + k4[7U]);
                predictedState[8U] += h6 * (k1[8U] + 2.0*(k2[8U] + k3[8U]) + k4[8U]);

                // Add new trajectory entry
                predictedState[2U] = mpsv::math::SymmetricalAngle(predictedState[2U]);
                trajectory.push_back({predictedState[0U], predictedState[1U], predictedState[2U], predictedState[3U], predictedState[4U], predictedState[5U], predictedState[6U], predictedState[7U], predictedState[8U], initialStateAndInput[9U], initialStateAndInput[10U], initialStateAndInput[11U]});
            }
            return true;
        }

    protected:
        /* model parameters and precalculated values set by @ref SetModel */
        std::array<double, 36U> m_F{};              // 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 9U> m_B{};               // 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 9U> m_invB{};            // Inverse of input matrix B.
        std::array<double, 3U> m_invTXYN{};         // {1/TX, 1/TY, 1/TN}.
        std::array<double, 3U> m_invTf123{};        // {1/Tf1, 1/Tf2, 1/Tf3}.
        std::array<double, 9U> m_invBBB{};          // Matrix inverse of (B * B_tau * B_c) (row-major order).
        std::array<double, 9U> m_M0{};              // Row-major order of M0 = B * A_tau.
        std::array<double, 3U> m_lowerLimitXYN{};   // Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 3U> m_upperLimitXYN{};   // Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.

        /* precalculated coefficients for first derivatives set by @ref SetModel */
        double m_f17_2{};                           // 2 * f_17.
        double m_f18_2{};                           // 2 * f_18.
        double m_f19_2{};                           // 2 * f_19.
        double m_f1a_3{};                           // 3 * f_1a.
        double m_f1b_3{};                           // 3 * f_1b.
        double m_f1c_3{};                           // 3 * f_1c.

        double m_f27_2{};                           // 2 * f_27.
        double m_f28_2{};                           // 2 * f_28.
        double m_f29_2{};                           // 2 * f_29.
        double m_f2a_3{};                           // 3 * f_2a.
        double m_f2b_3{};                           // 3 * f_2b.
        double m_f2c_3{};                           // 3 * f_2c.

        double m_f37_2{};                           // 2 * f_37.
        double m_f38_2{};                           // 2 * f_38.
        double m_f39_2{};                           // 2 * f_39.
        double m_f3a_3{};                           // 3 * f_3a.
        double m_f3b_3{};                           // 3 * f_3b.
        double m_f3c_3{};                           // 3 * f_3c.

        /* precalculated coefficients for second derivatives set by @ref SetModel */
        double m_f1a_6{};                           // 6 * f_1a.
        double m_f1b_6{};                           // 6 * f_1b.
        double m_f1c_6{};                           // 6 * f_1c.

        double m_f2a_6{};                           // 6 * f_2a.
        double m_f2b_6{};                           // 6 * f_2b.
        double m_f2c_6{};                           // 6 * f_2c.

        double m_f3a_6{};                           // 6 * f_3a.
        double m_f3b_6{};                           // 6 * f_3b.
        double m_f3c_6{};                           // 6 * f_3c.

        /* controller parameters and precalculated values set by @ref SetController */
        double m_minSquaredRadiusPosition{};        // r^2 with r being the minimum radius for position for the waypoint guidance law.
        double m_invSquaredRadiusX{};               // 1 / r^2 with r being the radius for the longitudinal distance for the waypoint guidance law.
        double m_invSquaredRadiusY{};               // 1 / r^2 with r being the radius for the lateral distance for the waypoint guidance law.
        double m_invSquaredRadiusPsi{};             // 1 / r^2 with r being the radius for the angular distrance for the waypoint guidance law.
        std::array<double, 36U> m_K{};              // 3-by-12 control gain matrix (row-major order) for pose control.

        /**
         * @brief The path guidance law calculates a target pose based on a given path and the current pose of the vehicle.
         * @param[out] targetPose Output target pose (x,y,psi) that is used by a pose controller.
         * @param[in] pose Input pose that denotes the current vehicle pose (x,y,psi).
         * @param[in] cosPsi Cosine of the heading angle psi from the pose.
         * @param[in] sinPsi Sine of the heading angle psi from the pose.
         * @param[in] path Input path desribed by a list of poses. Each point indicates (x,y,psi).
         * @param[in] staticObstacles List of static obstacles to be used by the guidance algorithm to limit the exploration radius near obstacles.
         * @details The target pose corresponds to the intersection point of the path with an ellipsoid, which is located at the pose of the vehicle.
         * The major and minor axes of that ellipsoid point in the longitudinal and lateral direction of the vehicles body frame. The dimension
         * of the ellpsoid is set by three radii - a longitudinal radius (X), a lateral radius (Y) and a radius for the heading angle (Psi).
         * This function requires the inverse of the squareroot of the corresponding radii, that is 1/(R*R). In addition, the radii for position are
         * limited by the nearby obstacles.
         * 
         * - If the ellipsoid intersects the path at several points, then that point being closest to the end of the path along that path is used.
         * 
         * - If there's no intersection point of the path with that ellipsoid, then the closest point of the ellipsoid to the path is used as
         *   target pose. This function thus ensures, that the distance between the given pose and the resulting target pose is not larger than
         *   the specified radii of the ellipsoid.
         * 
         * - If the final pose of the path is within the ellipsoid, then this final pose is used as target pose.
         * 
         * - If the specified path is empty, then the current vehicle pose is used as target pose.
         * 
         * - If the path consists of only one pose, then the line from the vehicles pose to that single pose is used as path to find the intersection
         *   with the ellipsoid.
         */
        void PathGuidance(std::array<double, 3U>& targetPose, std::array<double, 3U> pose, double cosPsi, double sinPsi, const std::vector<std::array<double, 3U>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) const noexcept {
            // To find the intersection point of an rotated ellipsoid with a path in SE(2), the path is transformed into a corresponding
            // euclidean space R^3, whose origin is located at the vehicles pose. The space transformation is done in such a way, that
            // the intersection simplifies to finding the intersection of a line in 3D space with a 3D unit sphere centered at the
            // vehicles pose. For a given vehicle pose [x; y; psi] any pose of the path [x_i; y_i; psi_i] can be transformed into the
            // euclidean space by
            // 
            //    ( p_ix )   (1/rx  0    0    )   ( cos(psi)  sin(psi)  0)   (     x_i - x   )
            //    ( p_iy ) = ( 0   1/ry  0    ) * (-sin(psi)  cos(psi)  0) * (     y_i - y   )
            //    (p_ipsi)   ( 0    0   1/rpsi)   (    0         0      1)   (SA(psi_i - psi))  ,
            // 
            // where rx,ry,rpsi denote the radii for longitudinal, lateral and angular direction, respectively, and SA() denotes the
            // symmetrical angle function. Assume, the path is given by five points [q0, q1, q2, q3, q4] and the vehicle pose is given by
            // eta = [x;y;psi].
            // 
            //    q0                 q1                 q2                 q3                 q4
            //     o ---------------- o ---------------- o ---------------- o ---------------- o
            // 
            // 
            // 
            //                                o eta
            // 
            // 
            // All poses of the path are transformed into the euclidean space and eta denotes the origin of that space. The guidance law
            // consists of two main steps.
            // 
            // 
            // STEP 1: FIND CLOSEST INTERPOLATED POSE
            // In the first step the closest interapolated pose along the path is searched for.
            // 
            //                             closest interpolated pose (pI)
            //                                |
            //     p0                p1       V         p2                 p3                 p4
            //      o - - - - - - - - o - - - x -------- o ---------------- o ---------------- o
            //                                :
            //                                :
            //                                :
            //                                o origin [0;0;0]
            // 
            // Now the actual path of interest consists of the origin, the closest interpolated pose and p2, p3, p4. This path
            // [0, pI, p2, p3, p4] always intersects with a unit sphere around the origin.
            // 
            // 
            // STEP 2: FIND FIRST INTERSECTION
            // In the second step, it is iterated through all line segments, starting from the origin. The first line segment that
            // intersects with the unit sphere, gives the commanded pose in the transformed euclidean space. The intersection point is
            // defined by the equation
            // 
            //  pc = pa + lambda * (pb - pa)  ,
            // 
            // where pa and pb denote two consecutive points, e.g. starting and ending point of a specific line segment. If a lambda is
            // found, then the corresponding intersection pose is calculated by the same equation but in SE(2).
            // 
            // eta_c = SA(eta_a + lambda * SA(eta_b - eta_a))  ,
            // 
            // where SA() ensures, that the angular dimension is handled correctly and remains in [-pi, +pi).
            // 
            //                                    intersection point (target point pc)
            //                                                  |
            //       p0              p1      pI         p2      V         p3                 p4
            //        o   -   -   -   o   -   o -------- o -----x---------- o ---------------- o
            //                                :                  ' 
            //                                :                   ' 
            //                                :                    :
            //                                o <~~~~~~~~~~~~~~~~> :
            //                                        r = 1        :
            //                                                    ,
            //                                                   ,
            // 
            // If the end of the path is within the unit sphere, e.g. |p4| < 1, then pc is equal to p4.

            // Some variables and constants
            constexpr double eps{100.0 * std::numeric_limits<double>::epsilon()};
            double a, bHalf, c, sr, lambda, dx, dy, dz, L, denominator, squaredDistance, lowestSquaredDistance;
            double tmp1, tmp2, tmp3;
            std::array<double, 3U> pa, pb;
            const std::size_t numWaypoints{path.size()};

            // Fallback value if path is empty
            if (!numWaypoints) {
                targetPose = pose;
                return;
            }

            // Clamp position radii to maximum value (e.g. distance to obstacles)
            double minSquaredDistance{std::numeric_limits<double>::infinity()};
            for (auto&& obstacle : staticObstacles) {
                minSquaredDistance = std::min(minSquaredDistance, obstacle.MinimumSquaredDistanceToEdges(pose[0U], pose[1U]));
            }
            double limitedInvRR{0.0};
            if (std::isfinite(minSquaredDistance)) {
                limitedInvRR = 1.0 / std::max(minSquaredDistance, m_minSquaredRadiusPosition);
            }
            double invRX{std::max(m_invSquaredRadiusX, limitedInvRR)};
            double invRY{std::max(m_invSquaredRadiusY, limitedInvRR)};

            // STEP 1: Get the closest interpolated pose on the path to the given pose
            std::array<std::size_t, 2U> idxSegment = {0U, 0U}; // stores the indices of the start and end point from a line segment
            double interpolation{0.0}; // stores the lambda value (interpolation ratio) for that line segment
            if (numWaypoints > 1U) {
                // Go through all line segments and search for the lowest squared distance
                lowestSquaredDistance = std::numeric_limits<double>::infinity();

                // Initial starting point pa for first segment (pa without 1/r scale)
                dx = path[0U][0U] - pose[0U];
                dy = path[0U][1U] - pose[1U];
                pa[0U] = cosPsi * dx + sinPsi * dy;
                pa[1U] = -sinPsi * dx + cosPsi * dy;
                pa[2U] = mpsv::math::SymmetricalAngle(path[0U][2U] - pose[2U]);
                for (std::size_t n{1U}; n < path.size(); ++n) {
                    // For each line segment: calculate the interpolation factor lambda that minimizes the distance from pose to the current line segment
                    // Starting point pa is already given, calculate ending point pb for this line segment (also without 1/r scale)
                    dx = path[n][0U] - pose[0U];
                    dy = path[n][1U] - pose[1U];
                    pb[0U] = cosPsi * dx + sinPsi * dy;
                    pb[1U] = -sinPsi * dx + cosPsi * dy;
                    pb[2U] = mpsv::math::SymmetricalAngle(path[n][2U] - pose[2U]);

                    // Calculate lambda of p = pa + lambda*(pb - pa), where p is closest to the origin (here we have to take into account the 1/r scale)
                    // lambda = (pa' * (pa - pb) / ((pa - pb)' * (pa - pb)))
                    dx = pa[0U] - pb[0U];
                    dy = pa[1U] - pb[1U];
                    dz = pa[2U] - pb[2U];
                    tmp1 = invRX*dx;
                    tmp2 = invRY*dy;
                    tmp3 = m_invSquaredRadiusPsi*dz;
                    denominator = tmp1*dx + tmp2*dy + tmp3*dz;
                    lambda = 0.0;
                    if (std::fabs(denominator) > eps) {
                        lambda = (tmp1*pa[0U] + tmp2*pa[1U] + tmp3*pa[2U]) / denominator;
                    }

                    // Clamp lambda to line segment [0, 1]
                    lambda = std::clamp(lambda, 0.0, 1.0);

                    // Calculate squared distance from pI = [sx; sy; sz] from origin in transformed R3 space and check if distance is reduced
                    pa[0U] -= lambda * dx;
                    pa[1U] -= lambda * dy;
                    pa[2U] -= lambda * dz;
                    squaredDistance = invRX*pa[0U]*pa[0U] + invRY*pa[1U]*pa[1U] + m_invSquaredRadiusPsi*pa[2U]*pa[2U];
                    if (squaredDistance < lowestSquaredDistance) {
                        lowestSquaredDistance = squaredDistance;
                        idxSegment[0U] = n - 1U;
                        idxSegment[1U] = n;
                        interpolation = lambda;
                    }

                    // Set the ending point of this line segment as starting point for the next line segment
                    pa = pb;
                }
            }
            std::array<double, 3U> interpolatedPose = mpsv::math::PoseDifference(path[idxSegment[1U]], path[idxSegment[0U]]);
            interpolatedPose[0U] = path[idxSegment[0U]][0U] + interpolation * interpolatedPose[0U];
            interpolatedPose[1U] = path[idxSegment[0U]][1U] + interpolation * interpolatedPose[1U];
            interpolatedPose[2U] = mpsv::math::SymmetricalAngle(path[idxSegment[0U]][2U] + interpolation * interpolatedPose[2U]);

            // STEP 2: Find the target pose, that is, the intersection of the ellipsoid with the path.
            // The path to be considered starts from the given pose and continues with the closest interpolated pose followed by all remaining poses
            // from the input path. The order is: pose --> interpolatedPose --> path[idxSegment[1]] --> ... --> path[end]. Because this path starts
            // at the pose, there's always an intersection of the ellipsoid with the line segment if the length of the linesegment is non-zero.

            // Line segment 1: pose --> interpolatedPose
            // We set pa to the interpolated pose (without 1/r scale) because this is the starting point for the next line segment
            dx = interpolatedPose[0U] - pose[0U];
            dy = interpolatedPose[1U] - pose[1U];
            pa[0U] = cosPsi * dx + sinPsi * dy;
            pa[1U] = -sinPsi * dx + cosPsi * dy;
            pa[2U] = mpsv::math::SymmetricalAngle(interpolatedPose[2U] - pose[2U]);
            L = invRX*pa[0U]*pa[0U] + invRY*pa[1U]*pa[1U] + m_invSquaredRadiusPsi*pa[2U]*pa[2U]; // take into account 1/r scale
            if (L > 1.0) {
                lambda = 1.0 / std::sqrt(L);
                targetPose[0U] = pose[0U] + lambda * dx;
                targetPose[1U] = pose[1U] + lambda * dy;
                targetPose[2U] = mpsv::math::SymmetricalAngle(pose[2U] + lambda * pa[2U]);
                return;
            }
            targetPose = interpolatedPose;

            // Line segment 2: interpolatedPose --> path[idxSegment[1]]
            // pa is already known from line segment 1, calculate end point pb (also without 1/r scale)
            dx = path[idxSegment[1U]][0U] - pose[0U];
            dy = path[idxSegment[1U]][1U] - pose[1U];
            pb[0U] = cosPsi * dx + sinPsi * dy;
            pb[1U] = -sinPsi * dx + cosPsi * dy;
            pb[2U] = mpsv::math::SymmetricalAngle(path[idxSegment[1U]][2U] - pose[2U]);
            dx = pb[0U] - pa[0U];
            dy = pb[1U] - pa[1U];
            dz = pb[2U] - pa[2U];
            tmp1 = invRX*dx;
            tmp2 = invRY*dy;
            tmp3 = m_invSquaredRadiusPsi*dz;
            a = tmp1*dx + tmp2*dy + tmp3*dz;
            bHalf = (tmp1*pa[0U] + tmp2*pa[1U] + tmp3*pa[2U]);
            c = invRX*pa[0U]*pa[0U] + invRY*pa[1U]*pa[1U] + m_invSquaredRadiusPsi*pa[2U]*pa[2U] - 1.0;
            if (a > eps) {
                sr = std::sqrt(bHalf * bHalf - a * c);
                lambda = std::max((-bHalf + sr) / a, (-bHalf - sr) / a);
                if (lambda <= 1.0) {
                    targetPose[0U] = interpolatedPose[0U] + lambda * (path[idxSegment[1U]][0U] - interpolatedPose[0U]);
                    targetPose[1U] = interpolatedPose[1U] + lambda * (path[idxSegment[1U]][1U] - interpolatedPose[1U]);
                    targetPose[2U] = mpsv::math::SymmetricalAngle(interpolatedPose[2U] + lambda * mpsv::math::SymmetricalAngle(path[idxSegment[1U]][2U] - interpolatedPose[2U]));
                    return;
                }
            }
            targetPose = path[idxSegment[1U]];

            // Line segments 3,...: path[idxSegment[1]] --> ... --> path[end]
            for (std::size_t i{idxSegment[1U] + 1U}; i < path.size(); ++i) {
                // Start point pa is previous end point pb
                pa = pb;

                // Calculate the endpoint of this line segment (again without 1/r scale)
                dx = path[i][0U] - pose[0U];
                dy = path[i][1U] - pose[1U];
                pb[0U] = cosPsi * dx + sinPsi * dy;
                pb[1U] = -sinPsi * dx + cosPsi * dy;
                pb[2U] = mpsv::math::SymmetricalAngle(path[i][2U] - pose[2U]);
                dx = pb[0U] - pa[0U];
                dy = pb[1U] - pa[1U];
                dz = pb[2U] - pa[2U];
                tmp1 = invRX*dx;
                tmp2 = invRY*dy;
                tmp3 = m_invSquaredRadiusPsi*dz;
                a = tmp1*dx + tmp2*dy + tmp3*dz;
                bHalf = (tmp1*pa[0U] + tmp2*pa[1U] + tmp3*pa[2U]);
                c = invRX*pa[0U]*pa[0U] + invRY*pa[1U]*pa[1U] + m_invSquaredRadiusPsi*pa[2U]*pa[2U] - 1.0;
                if (a > eps) {
                    sr = std::sqrt(bHalf * bHalf - a * c);
                    lambda = std::max((-bHalf + sr) / a, (-bHalf - sr) / a);
                    if (lambda <= 1.0) {
                        targetPose[0U] = path[i - 1U][0U] + lambda * (path[i][0U] - path[i - 1U][0U]);
                        targetPose[1U] = path[i - 1U][1U] + lambda * (path[i][1U] - path[i - 1U][1U]);
                        targetPose[2U] = mpsv::math::SymmetricalAngle(path[i - 1U][2U] + lambda * mpsv::math::SymmetricalAngle(path[i][2U] - path[i - 1U][2U]));
                        return;
                    }
                }
                targetPose = path[i];
            }
            targetPose[2U] = mpsv::math::SymmetricalAngle(targetPose[2U]);
        }
};


} /* namespace: mpsv::control */
