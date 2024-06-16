#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>


namespace mpsv {


namespace control {


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
         * @brief Construct a new vehicle simulator object.
         * @details Calls the @ref ClearModel and @ref ClearController member function. All values are set to zero.
         */
        VehicleSimulator() noexcept { ClearModel(); ClearController(); }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // INITIALIZATION
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /**
         * @brief Set parameters for the dynamic model.
         * @param[in] matF 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
         * @param[in] matB 3-by-3 non-singular input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
         * @param[in] vecTimeconstantsXYN Timeconstants {TX, TY, TN} for input force dynamics.
         * @param[in] vecTimeconstantsInput Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
         * @param[in] lowerLimitXYN Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
         * @param[in] upperLimitXYN Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
         * @return True if all input values and precalculated values are finite, false otherwise.
         */
        bool SetModel(std::array<double,36> matF, std::array<double,9> matB, std::array<double,3> vecTimeconstantsXYN, std::array<double,3> vecTimeconstantsInput, std::array<double,3> lowerLimitXYN, std::array<double,3> upperLimitXYN) noexcept {
            // Set internal parameters
            F = matF;
            B = matB;
            auto [success, invMatB] = mpsv::math::MatrixInverse3x3(B);
            if(!success){
                ClearModel();
                return false;
            }
            invB.swap(invMatB);
            this->lowerLimitXYN = lowerLimitXYN;
            this->upperLimitXYN = upperLimitXYN;

            // Precalculate parameters
            invTXYN[0] = 1.0 / vecTimeconstantsXYN[0];
            invTXYN[1] = 1.0 / vecTimeconstantsXYN[1];
            invTXYN[2] = 1.0 / vecTimeconstantsXYN[2];
            invTf123[0] = 1.0 / vecTimeconstantsInput[0];
            invTf123[1] = 1.0 / vecTimeconstantsInput[1];
            invTf123[2] = 1.0 / vecTimeconstantsInput[2];
            double TXTf1 = vecTimeconstantsXYN[0] * vecTimeconstantsInput[0];
            double TYTf2 = vecTimeconstantsXYN[1] * vecTimeconstantsInput[1];
            double TNTf3 = vecTimeconstantsXYN[2] * vecTimeconstantsInput[2];
            invBBB[0] = TXTf1 * invB[0];
            invBBB[1] = TXTf1 * invB[1];
            invBBB[2] = TXTf1 * invB[2];
            invBBB[3] = TYTf2 * invB[3];
            invBBB[4] = TYTf2 * invB[4];
            invBBB[5] = TYTf2 * invB[5];
            invBBB[6] = TNTf3 * invB[6];
            invBBB[7] = TNTf3 * invB[7];
            invBBB[8] = TNTf3 * invB[8];
            M0[0] = -B[0] / vecTimeconstantsXYN[0];
            M0[1] = -B[1] / vecTimeconstantsXYN[1];
            M0[2] = -B[2] / vecTimeconstantsXYN[2];
            M0[3] = -B[3] / vecTimeconstantsXYN[0];
            M0[4] = -B[4] / vecTimeconstantsXYN[1];
            M0[5] = -B[5] / vecTimeconstantsXYN[2];
            M0[6] = -B[6] / vecTimeconstantsXYN[0];
            M0[7] = -B[7] / vecTimeconstantsXYN[1];
            M0[8] = -B[8] / vecTimeconstantsXYN[2];
            f17_2 = 2.0 * F[6];
            f18_2 = 2.0 * F[7];
            f19_2 = 2.0 * F[8];
            f1a_3 = 3.0 * F[9];
            f1b_3 = 3.0 * F[10];
            f1c_3 = 3.0 * F[11];
            f27_2 = 2.0 * F[18];
            f28_2 = 2.0 * F[19];
            f29_2 = 2.0 * F[20];
            f2a_3 = 3.0 * F[21];
            f2b_3 = 3.0 * F[22];
            f2c_3 = 3.0 * F[23];
            f37_2 = 2.0 * F[30];
            f38_2 = 2.0 * F[31];
            f39_2 = 2.0 * F[32];
            f3a_3 = 3.0 * F[33];
            f3b_3 = 3.0 * F[34];
            f3c_3 = 3.0 * F[35];
            f1a_6 = 6.0 * F[9];
            f1b_6 = 6.0 * F[10];
            f1c_6 = 6.0 * F[11];
            f2a_6 = 6.0 * F[21];
            f2b_6 = 6.0 * F[22];
            f2c_6 = 6.0 * F[23];
            f3a_6 = 6.0 * F[33];
            f3b_6 = 6.0 * F[34];
            f3c_6 = 6.0 * F[35];

            // Check for finite model values
            bool validModel = true;
            validModel &= std::isfinite(this->lowerLimitXYN[0]) && std::isfinite(this->lowerLimitXYN[1]) && std::isfinite(this->lowerLimitXYN[2]);
            validModel &= std::isfinite(this->upperLimitXYN[0]) && std::isfinite(this->upperLimitXYN[1]) && std::isfinite(this->upperLimitXYN[2]);
            validModel = std::isfinite(matF[0]) && std::isfinite(matF[1]) && std::isfinite(matF[2]) && std::isfinite(matF[3]) && std::isfinite(matF[4]) && std::isfinite(matF[5]) && std::isfinite(matF[6]) && std::isfinite(matF[7]) && std::isfinite(matF[8]) && std::isfinite(matF[9]) && std::isfinite(matF[10]) && std::isfinite(matF[11]);
            validModel &= std::isfinite(matF[12]) && std::isfinite(matF[13]) && std::isfinite(matF[14]) && std::isfinite(matF[15]) && std::isfinite(matF[16]) && std::isfinite(matF[17]) && std::isfinite(matF[18]) && std::isfinite(matF[19]) && std::isfinite(matF[20]) && std::isfinite(matF[21]) && std::isfinite(matF[22]) && std::isfinite(matF[23]);
            validModel &= std::isfinite(matF[24]) && std::isfinite(matF[25]) && std::isfinite(matF[26]) && std::isfinite(matF[27]) && std::isfinite(matF[28]) && std::isfinite(matF[29]) && std::isfinite(matF[30]) && std::isfinite(matF[31]) && std::isfinite(matF[32]) && std::isfinite(matF[33]) && std::isfinite(matF[34]) && std::isfinite(matF[35]);
            validModel &= std::isfinite(B[0]) && std::isfinite(B[1]) && std::isfinite(B[2]) && std::isfinite(B[3]) && std::isfinite(B[4]) && std::isfinite(B[5]) && std::isfinite(B[6]) && std::isfinite(B[7]) && std::isfinite(B[8]);
            validModel &= std::isfinite(invB[0]) && std::isfinite(invB[1]) && std::isfinite(invB[2]) && std::isfinite(invB[3]) && std::isfinite(invB[4]) && std::isfinite(invB[5]) && std::isfinite(invB[6]) && std::isfinite(invB[7]) && std::isfinite(invB[8]);
            validModel &= std::isfinite(invTXYN[0]) && std::isfinite(invTXYN[1]) && std::isfinite(invTXYN[2]);
            validModel &= std::isfinite(invTf123[0]) && std::isfinite(invTf123[1]) && std::isfinite(invTf123[2]);
            validModel &= std::isfinite(invBBB[0]) && std::isfinite(invBBB[1]) && std::isfinite(invBBB[2]) && std::isfinite(invBBB[3]) && std::isfinite(invBBB[4]) && std::isfinite(invBBB[5]) && std::isfinite(invBBB[6]) && std::isfinite(invBBB[7]) && std::isfinite(invBBB[8]);
            validModel &= std::isfinite(M0[0]) && std::isfinite(M0[1]) && std::isfinite(M0[2]) && std::isfinite(M0[3]) && std::isfinite(M0[4]) && std::isfinite(M0[5]) && std::isfinite(M0[6]) && std::isfinite(M0[7]) && std::isfinite(M0[8]);
            if(!validModel){
                ClearModel();
                return false;
            }
            return true;
        }

        /**
         * @brief Set parameters for the control system.
         * @param[in] vecTimeconstantsFlatStates Timeconstants for flat states {Tu, Tv, Tr, Tu_dot, Tv_dot, Tr_dot, Tu_dotdot, Tv_dotdot, Tr_dotdot}.
         * @param[in] matK 3-by-12 control gain matrix (row-major order) for pose control (state controller using underlying velocity controller based on feedback-linearization).
         * @param[in] maxRadiusX Maximum look-ahead distance for longitudinal distance during pose control.
         * @param[in] maxRadiusY Maximum look-ahead distance for lateral distance during pose control.
         * @param[in] maxRadiusPsi Maximum look-ahead distance for angular distance during pose control.
         * @param[in] minRadiusPosition Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
         * @return True if all input values and precalculated values are finite, false otherwise.
         */
        bool SetController(std::array<double,9> vecTimeconstantsFlatStates, std::array<double,36> matK, double maxRadiusX, double maxRadiusY, double maxRadiusPsi, double minRadiusPosition) noexcept {
            // Set internal parameters
            K = matK;
            minSquaredRadiusPosition = minRadiusPosition * minRadiusPosition;
            invSquaredRadiusX = 1.0 / (maxRadiusX * maxRadiusX);
            invSquaredRadiusY = 1.0 / (maxRadiusY * maxRadiusY);
            invSquaredRadiusPsi = 1.0 / (maxRadiusPsi * maxRadiusPsi);

            // Precalculate parameters
            Kz[0] = 1.0 / (vecTimeconstantsFlatStates[0] * vecTimeconstantsFlatStates[3] * vecTimeconstantsFlatStates[6]);
            Kz[1] = 1.0 / (vecTimeconstantsFlatStates[1] * vecTimeconstantsFlatStates[4] * vecTimeconstantsFlatStates[7]);
            Kz[2] = 1.0 / (vecTimeconstantsFlatStates[2] * vecTimeconstantsFlatStates[5] * vecTimeconstantsFlatStates[8]);
            Kz[3] = (vecTimeconstantsFlatStates[0] + vecTimeconstantsFlatStates[3] + vecTimeconstantsFlatStates[6])*Kz[0];
            Kz[4] = (vecTimeconstantsFlatStates[1] + vecTimeconstantsFlatStates[4] + vecTimeconstantsFlatStates[7])*Kz[1];
            Kz[5] = (vecTimeconstantsFlatStates[2] + vecTimeconstantsFlatStates[5] + vecTimeconstantsFlatStates[8])*Kz[2];
            Kz[6] = (vecTimeconstantsFlatStates[0]*vecTimeconstantsFlatStates[3] + vecTimeconstantsFlatStates[0]*vecTimeconstantsFlatStates[6] + vecTimeconstantsFlatStates[3]*vecTimeconstantsFlatStates[6])*Kz[0];
            Kz[7] = (vecTimeconstantsFlatStates[1]*vecTimeconstantsFlatStates[4] + vecTimeconstantsFlatStates[1]*vecTimeconstantsFlatStates[7] + vecTimeconstantsFlatStates[4]*vecTimeconstantsFlatStates[7])*Kz[1];
            Kz[8] = (vecTimeconstantsFlatStates[2]*vecTimeconstantsFlatStates[5] + vecTimeconstantsFlatStates[2]*vecTimeconstantsFlatStates[8] + vecTimeconstantsFlatStates[5]*vecTimeconstantsFlatStates[8])*Kz[2];

            // Check for finite controller values
            bool validController = true;
            validController &= std::isfinite(K[0]) && std::isfinite(K[1]) && std::isfinite(K[2]) && std::isfinite(K[3]) && std::isfinite(K[4]) && std::isfinite(K[5]) && std::isfinite(K[6]) && std::isfinite(K[7]) && std::isfinite(K[8]);
            validController &= std::isfinite(K[9]) && std::isfinite(K[10]) && std::isfinite(K[11]) && std::isfinite(K[12]) && std::isfinite(K[13]) && std::isfinite(K[14]) && std::isfinite(K[15]) && std::isfinite(K[16]) && std::isfinite(K[17]);
            validController &= std::isfinite(K[18]) && std::isfinite(K[19]) && std::isfinite(K[20]) && std::isfinite(K[21]) && std::isfinite(K[22]) && std::isfinite(K[23]) && std::isfinite(K[24]) && std::isfinite(K[25]) && std::isfinite(K[26]);
            validController &= std::isfinite(K[27]) && std::isfinite(K[28]) && std::isfinite(K[29]) && std::isfinite(K[30]) && std::isfinite(K[31]) && std::isfinite(K[32]) && std::isfinite(K[33]) && std::isfinite(K[34]) && std::isfinite(K[35]);
            validController &= std::isfinite(Kz[0]) && std::isfinite(Kz[1]) && std::isfinite(Kz[2]) && std::isfinite(Kz[3]) && std::isfinite(Kz[4]) && std::isfinite(Kz[5]) && std::isfinite(Kz[6]) && std::isfinite(Kz[7]) && std::isfinite(Kz[8]);
            validController &= std::isfinite(minSquaredRadiusPosition);
            validController &= std::isfinite(invSquaredRadiusX);
            validController &= std::isfinite(invSquaredRadiusY);
            validController &= std::isfinite(invSquaredRadiusPsi);
            if(!validController){
                ClearController();
                return false;
            }
            return true;
        }

        /**
         * @brief Clear model parameters. All values are set to zero.
         */
        void ClearModel(void) noexcept {
            F.fill(0.0);
            B.fill(0.0);
            invB.fill(0.0);
            invTXYN.fill(0.0);
            invTf123.fill(0.0);
            invBBB.fill(0.0);
            M0.fill(0.0);
            lowerLimitXYN.fill(0.0);
            upperLimitXYN.fill(0.0);
            f17_2 = 0.0;
            f18_2 = 0.0;
            f19_2 = 0.0;
            f1a_3 = 0.0;
            f1b_3 = 0.0;
            f1c_3 = 0.0;
            f27_2 = 0.0;
            f28_2 = 0.0;
            f29_2 = 0.0;
            f2a_3 = 0.0;
            f2b_3 = 0.0;
            f2c_3 = 0.0;
            f37_2 = 0.0;
            f38_2 = 0.0;
            f39_2 = 0.0;
            f3a_3 = 0.0;
            f3b_3 = 0.0;
            f3c_3 = 0.0;
            f1a_6 = 0.0;
            f1b_6 = 0.0;
            f1c_6 = 0.0;
            f2a_6 = 0.0;
            f2b_6 = 0.0;
            f2c_6 = 0.0;
            f3a_6 = 0.0;
            f3b_6 = 0.0;
            f3c_6 = 0.0;
        }

        /**
         * @brief Clear controller parameters. All values are set to zero.
         */
        void ClearController(void) noexcept {
            K.fill(0.0);
            Kz.fill(0.0);
            invSquaredRadiusX = 0.0;
            invSquaredRadiusY = 0.0;
            invSquaredRadiusPsi = 0.0;
            minSquaredRadiusPosition = 0.0;
        }


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // SIMULATION
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
        uint32_t ExplorePathToEnd(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, const std::vector<std::array<double,3>>& path, mpsv::control::RegionOfAttraction& regionOfAttraction, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles, double sampletime, uint32_t minNumSimulationSteps = 1) noexcept {
            // xu stores the current state x={x,y,psi,u,v,r,X,Y,N} and input u={Xc,Yc,Nc}
            std::array<double,12> xu = initialStateAndInput;
            trajectory.clear();

            // Set final pose
            std::array<double,3> finalPose = {initialStateAndInput[0], initialStateAndInput[1], initialStateAndInput[2]};
            if(path.size()){
                finalPose = path.back();
            }

            // Simulate step by step
            double h2 = sampletime / 2.0;
            double h6 = sampletime / 6.0;
            double c, s, dx, dy;
            std::array<double,3> targetPose, forcedResponse, allocationModel, deltaPose, uvrCommand, tmp, compensation, uTau;
            std::array<double,9> J, S0, S1, z;
            std::array<double,12> xuTmp, k1, k2, k3, k4;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            uint32_t k = 0;
            uint32_t kmin = static_cast<uint32_t>(std::max(minNumSimulationSteps, static_cast<uint32_t>(1)));
            for(; k < 0x0000FFFF; ++k){
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Check if termination condition is satisfied (minimum number of steps and state inside region of attraction)
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if((k >= kmin) && regionOfAttraction.IsInside(xu, finalPose)){
                    break;
                }


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Target pose for pose controller according to waypoint guidance law
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                c = std::cos(xu[2]);
                s = std::sin(xu[2]);
                PathGuidance(targetPose, {xu[0], xu[1], xu[2]}, c, s,  path, staticObstacles);


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Calculation of useful model terms and matrices
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // forced response velocity model: f(nu) = F*n(nu) + B*tau, where n(nu) = (u,v,r,v*r,u*r,u*v,u^2,v^2,r^2,u^3,v^3,r^3)^T
                vr = xu[4] * xu[5];
                ur = xu[3] * xu[5];
                uv = xu[3] * xu[4];
                uu = xu[3] * xu[3];
                vv = xu[2] * xu[4];
                rr = xu[5] * xu[5];
                uuu = uu * xu[3];
                vvv = vv * xu[4];
                rrr = rr * xu[5];
                forcedResponse[0] =  F[0]*xu[3] +  F[1]*xu[4] +  F[2]*xu[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*xu[6] + B[1]*xu[7] + B[2]*xu[8];
                forcedResponse[1] = F[12]*xu[3] + F[13]*xu[4] + F[14]*xu[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*xu[6] + B[4]*xu[7] + B[5]*xu[8];
                forcedResponse[2] = F[24]*xu[3] + F[25]*xu[4] + F[26]*xu[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*xu[6] + B[7]*xu[7] + B[8]*xu[8];

                // Forced response allocation model: A_tau * tau + B_tau * tau_c
                allocationModel[0] = (xu[9] - xu[6]) * invTXYN[0];
                allocationModel[1] = (xu[10] - xu[7]) * invTXYN[1];
                allocationModel[2] = (xu[11] - xu[8]) * invTXYN[2];

                // Jacobian J = df(nu)/dnu (uvr model w.r.t. uvr)
                J[0] =  F[0] +  F[4]*xu[5] +  F[5]*xu[4] + f17_2*xu[3] + f1a_3*uu;
                J[1] =  F[1] +  F[3]*xu[5] +  F[5]*xu[3] + f18_2*xu[4] + f1b_3*vv;
                J[2] =  F[2] +  F[3]*xu[4] +  F[4]*xu[3] + f19_2*xu[5] + f1c_3*rr;
                J[3] = F[12] + F[16]*xu[5] + F[17]*xu[4] + f27_2*xu[3] + f2a_3*uu;
                J[4] = F[13] + F[15]*xu[5] + F[17]*xu[3] + f28_2*xu[4] + f2b_3*vv;
                J[5] = F[14] + F[15]*xu[4] + F[16]*xu[3] + f29_2*xu[5] + f2c_3*rr;
                J[6] = F[24] + F[28]*xu[5] + F[29]*xu[4] + f37_2*xu[3] + f3a_3*uu;
                J[7] = F[25] + F[27]*xu[5] + F[29]*xu[3] + f38_2*xu[4] + f3b_3*vv;
                J[8] = F[26] + F[27]*xu[4] + F[28]*xu[3] + f39_2*xu[5] + f3c_3*rr;

                // S0 matrix: J^2 + T, where t_m1 = (dj_m1/du, dj_m2/du, dj_m2/du)*forcedResponse, t_m2 = (dj_m1/dv, dj_m2/dv, dj_m2/dv)*forcedResponse, t_m3 = (dj_m1/dr, dj_m2/dr, dj_m2/dr)*forcedResponse for m=1,2,3
                S0[0] = J[0]*J[0] + J[1]*J[3] + J[2]*J[6] + (f17_2 + f1a_6*xu[3])*forcedResponse[0] +                  F[5]*forcedResponse[1] +                  F[4]*forcedResponse[2];
                S0[1] = J[0]*J[1] + J[1]*J[4] + J[2]*J[7] +                  F[5]*forcedResponse[0] + (f18_2 + f1b_6*xu[4])*forcedResponse[1] +                  F[3]*forcedResponse[2];
                S0[2] = J[0]*J[2] + J[1]*J[5] + J[2]*J[8] +                  F[4]*forcedResponse[0] +                  F[3]*forcedResponse[1] + (f19_2 + f1c_6*xu[5])*forcedResponse[2];
                S0[3] = J[3]*J[0] + J[4]*J[3] + J[5]*J[6] + (f27_2 + f2a_6*xu[3])*forcedResponse[0] +                 F[17]*forcedResponse[1] +                 F[16]*forcedResponse[2];
                S0[4] = J[3]*J[1] + J[4]*J[4] + J[5]*J[7] +                 F[17]*forcedResponse[0] + (f28_2 + f2b_6*xu[4])*forcedResponse[1] +                 F[15]*forcedResponse[2];
                S0[5] = J[3]*J[2] + J[4]*J[5] + J[5]*J[8] +                 F[16]*forcedResponse[0] +                 F[15]*forcedResponse[1] + (f29_2 + f2c_6*xu[5])*forcedResponse[2];
                S0[6] = J[6]*J[0] + J[7]*J[3] + J[8]*J[6] + (f37_2 + f3a_6*xu[3])*forcedResponse[0] +                 F[29]*forcedResponse[1] +                 F[28]*forcedResponse[2];
                S0[7] = J[6]*J[1] + J[7]*J[4] + J[8]*J[7] +                 F[29]*forcedResponse[0] + (f38_2 + f3b_6*xu[4])*forcedResponse[1] +                 F[27]*forcedResponse[2];
                S0[8] = J[6]*J[2] + J[7]*J[5] + J[8]*J[8] +                 F[28]*forcedResponse[0] +                 F[27]*forcedResponse[1] + (f39_2 + f3c_6*xu[5])*forcedResponse[2];

                // S1 matrix: J * B
                S1[0] = J[0]*B[0] + J[1]*B[3] + J[2]*B[6];
                S1[1] = J[0]*B[1] + J[1]*B[4] + J[2]*B[7];
                S1[2] = J[0]*B[2] + J[1]*B[5] + J[2]*B[8];
                S1[3] = J[3]*B[0] + J[4]*B[3] + J[5]*B[6];
                S1[4] = J[3]*B[1] + J[4]*B[4] + J[5]*B[7];
                S1[5] = J[3]*B[2] + J[4]*B[5] + J[5]*B[8];
                S1[6] = J[6]*B[0] + J[7]*B[3] + J[8]*B[6];
                S1[7] = J[6]*B[1] + J[7]*B[4] + J[8]*B[7];
                S1[8] = J[6]*B[2] + J[7]*B[5] + J[8]*B[8];

                // Flat state vector
                z[0] = xu[3];
                z[1] = xu[4];
                z[2] = xu[5];
                z[3] = forcedResponse[0];
                z[4] = forcedResponse[1];
                z[5] = forcedResponse[2];
                z[6] = J[0]*forcedResponse[0] + J[1]*forcedResponse[1] + J[2]*forcedResponse[2] + B[0]*allocationModel[0] + B[1]*allocationModel[1] + B[2]*allocationModel[2];
                z[7] = J[3]*forcedResponse[0] + J[4]*forcedResponse[1] + J[5]*forcedResponse[2] + B[3]*allocationModel[0] + B[4]*allocationModel[1] + B[5]*allocationModel[2];
                z[8] = J[6]*forcedResponse[0] + J[7]*forcedResponse[1] + J[8]*forcedResponse[2] + B[6]*allocationModel[0] + B[7]*allocationModel[1] + B[8]*allocationModel[2];


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Pose controller (state controller with respect to target pose as origin)
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                dx = xu[0] - targetPose[0];
                dy = xu[1] - targetPose[1];
                deltaPose[0] = c*dx + s*dy;
                deltaPose[1] =-s*dx + c*dy;
                deltaPose[2] = mpsv::math::SymmetricalAngle(xu[2] - targetPose[2]);

                // Calculate the velocity command using the control law uvrCommand = -K * [deltaPose; z]
                uvrCommand[0] = -K[0]*deltaPose[0]  - K[1]*deltaPose[1]  - K[2]*deltaPose[2]  - K[3]*z[0]  - K[4]*z[1]  - K[5]*z[2]  - K[6]*z[3]  - K[7]*z[4]  - K[8]*z[5]  - K[9]*z[6]  - K[10]*z[7] - K[11]*z[8];
                uvrCommand[1] = -K[12]*deltaPose[0] - K[13]*deltaPose[1] - K[14]*deltaPose[2] - K[15]*z[0] - K[16]*z[1] - K[17]*z[2] - K[18]*z[3] - K[19]*z[4] - K[20]*z[5] - K[21]*z[6] - K[22]*z[7] - K[23]*z[8];
                uvrCommand[2] = -K[24]*deltaPose[0] - K[25]*deltaPose[1] - K[26]*deltaPose[2] - K[27]*z[0] - K[28]*z[1] - K[29]*z[2] - K[30]*z[3] - K[31]*z[4] - K[32]*z[5] - K[33]*z[6] - K[34]*z[7] - K[35]*z[8];


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Velocity controller using feedback-linearization
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // compensation = S0(uvr) * forcedResponse + S1(uvr) * allocationModel + M0 * (allocationModel + Bc*tau_c)
                tmp[0] = allocationModel[0] + xu[9] * invTf123[0];
                tmp[1] = allocationModel[1] + xu[10] * invTf123[1];
                tmp[2] = allocationModel[2] + xu[11] * invTf123[2];
                compensation[0] = S0[0]*forcedResponse[0] + S0[1]*forcedResponse[1] + S0[2]*forcedResponse[2] + S1[0]*allocationModel[0] + S1[1]*allocationModel[1] + S1[2]*allocationModel[2] + M0[0]*tmp[0] + M0[1]*tmp[1] + M0[2]*tmp[2];
                compensation[1] = S0[3]*forcedResponse[0] + S0[4]*forcedResponse[1] + S0[5]*forcedResponse[2] + S1[3]*allocationModel[0] + S1[4]*allocationModel[1] + S1[5]*allocationModel[2] + M0[3]*tmp[0] + M0[4]*tmp[1] + M0[5]*tmp[2];
                compensation[2] = S0[6]*forcedResponse[0] + S0[7]*forcedResponse[1] + S0[8]*forcedResponse[2] + S1[6]*allocationModel[0] + S1[7]*allocationModel[1] + S1[8]*allocationModel[2] + M0[6]*tmp[0] + M0[7]*tmp[1] + M0[8]*tmp[2];

                // Final control law uTau = invBBB * (-compensation - K*z + V*uvrCommand)
                tmp[0] = Kz[0]*(uvrCommand[0] - z[0]) - Kz[3]*z[3] - Kz[6]*z[6] - compensation[0];
                tmp[1] = Kz[1]*(uvrCommand[1] - z[1]) - Kz[4]*z[4] - Kz[7]*z[7] - compensation[1];
                tmp[2] = Kz[2]*(uvrCommand[2] - z[2]) - Kz[5]*z[5] - Kz[8]*z[8] - compensation[2];
                uTau[0] = invBBB[0]*tmp[0] + invBBB[1]*tmp[1] + invBBB[2]*tmp[2];
                uTau[1] = invBBB[3]*tmp[0] + invBBB[4]*tmp[1] + invBBB[5]*tmp[2];
                uTau[2] = invBBB[6]*tmp[0] + invBBB[7]*tmp[1] + invBBB[8]*tmp[2];

                // Saturate control input
                uTau[0] = std::clamp(uTau[0], lowerLimitXYN[0], upperLimitXYN[0]);
                uTau[1] = std::clamp(uTau[1], lowerLimitXYN[1], upperLimitXYN[1]);
                uTau[2] = std::clamp(uTau[2], lowerLimitXYN[2], upperLimitXYN[2]);


                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // Non-linear Model: numerical integration via RK4
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // RK4: k1
                k1[0]  = c*xu[3] - s*xu[4];
                k1[1]  = s*xu[3] + c*xu[4];
                k1[2]  = xu[5];
                k1[3]  = forcedResponse[0];
                k1[4]  = forcedResponse[1];
                k1[5]  = forcedResponse[2];
                k1[6]  = allocationModel[0];
                k1[7]  = allocationModel[1];
                k1[8]  = allocationModel[2];
                k1[9]  = (uTau[0] - xu[9]) * invTf123[0];
                k1[10] = (uTau[1] - xu[10]) * invTf123[1];
                k1[11] = (uTau[2] - xu[11]) * invTf123[2];

                // RK4: k2
                xuTmp[0]  = xu[0]  + h2 * k1[0];
                xuTmp[1]  = xu[1]  + h2 * k1[1];
                xuTmp[2]  = xu[2]  + h2 * k1[2];
                xuTmp[3]  = xu[3]  + h2 * k1[3];
                xuTmp[4]  = xu[4]  + h2 * k1[4];
                xuTmp[5]  = xu[5]  + h2 * k1[5];
                xuTmp[6]  = xu[6]  + h2 * k1[6];
                xuTmp[7]  = xu[7]  + h2 * k1[7];
                xuTmp[8]  = xu[8]  + h2 * k1[8];
                xuTmp[9]  = xu[9]  + h2 * k1[9];
                xuTmp[10] = xu[10] + h2 * k1[10];
                xuTmp[11] = xu[11] + h2 * k1[11];
                c = std::cos(xuTmp[2]);
                s = std::sin(xuTmp[2]);
                uv = xuTmp[3] * xuTmp[4];
                ur = xuTmp[3] * xuTmp[5];
                vr = xuTmp[4] * xuTmp[5];
                uu = xuTmp[3] * xuTmp[3];
                vv = xuTmp[4] * xuTmp[4];
                rr = xuTmp[5] * xuTmp[5];
                uuu = uu * xuTmp[3];
                vvv = vv * xuTmp[4];
                rrr = rr * xuTmp[5];
                k2[0]  = c*xuTmp[3] - s*xuTmp[4];
                k2[1]  = s*xuTmp[3] + c*xuTmp[4];
                k2[2]  = xuTmp[5];
                k2[3]  =  F[0]*xuTmp[3] +  F[1]*xuTmp[4] +  F[2]*xuTmp[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*xuTmp[6] + B[1]*xuTmp[7] + B[2]*xuTmp[8];
                k2[4]  = F[12]*xuTmp[3] + F[13]*xuTmp[4] + F[14]*xuTmp[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*xuTmp[6] + B[4]*xuTmp[7] + B[5]*xuTmp[8];
                k2[5]  = F[24]*xuTmp[3] + F[25]*xuTmp[4] + F[26]*xuTmp[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*xuTmp[6] + B[7]*xuTmp[7] + B[8]*xuTmp[8];
                k2[6]  = (xuTmp[9]  - xuTmp[6]) * invTXYN[0];
                k2[7]  = (xuTmp[10] - xuTmp[7]) * invTXYN[1];
                k2[8]  = (xuTmp[11] - xuTmp[8]) * invTXYN[2];
                k2[9]  = (uTau[0] - xuTmp[9]) * invTf123[0];
                k2[10] = (uTau[1] - xuTmp[10]) * invTf123[1];
                k2[11] = (uTau[2] - xuTmp[11]) * invTf123[2];

                // RK4: k3
                xuTmp[0]  = xu[0]  + h2 * k2[0];
                xuTmp[1]  = xu[1]  + h2 * k2[1];
                xuTmp[2]  = xu[2]  + h2 * k2[2];
                xuTmp[3]  = xu[3]  + h2 * k2[3];
                xuTmp[4]  = xu[4]  + h2 * k2[4];
                xuTmp[5]  = xu[5]  + h2 * k2[5];
                xuTmp[6]  = xu[6]  + h2 * k2[6];
                xuTmp[7]  = xu[7]  + h2 * k2[7];
                xuTmp[8]  = xu[8]  + h2 * k2[8];
                xuTmp[9]  = xu[9]  + h2 * k2[9];
                xuTmp[10] = xu[10] + h2 * k2[10];
                xuTmp[11] = xu[11] + h2 * k2[11];
                c = std::cos(xuTmp[2]);
                s = std::sin(xuTmp[2]);
                uv = xuTmp[3] * xuTmp[4];
                ur = xuTmp[3] * xuTmp[5];
                vr = xuTmp[4] * xuTmp[5];
                uu = xuTmp[3] * xuTmp[3];
                vv = xuTmp[4] * xuTmp[4];
                rr = xuTmp[5] * xuTmp[5];
                uuu = uu * xuTmp[3];
                vvv = vv * xuTmp[4];
                rrr = rr * xuTmp[5];
                k3[0]  = c*xuTmp[3] - s*xuTmp[4];
                k3[1]  = s*xuTmp[3] + c*xuTmp[4];
                k3[2]  = xuTmp[5];
                k3[3]  =  F[0]*xuTmp[3] +  F[1]*xuTmp[4] +  F[2]*xuTmp[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*xuTmp[6] + B[1]*xuTmp[7] + B[2]*xuTmp[8];
                k3[4]  = F[12]*xuTmp[3] + F[13]*xuTmp[4] + F[14]*xuTmp[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*xuTmp[6] + B[4]*xuTmp[7] + B[5]*xuTmp[8];
                k3[5]  = F[24]*xuTmp[3] + F[25]*xuTmp[4] + F[26]*xuTmp[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*xuTmp[6] + B[7]*xuTmp[7] + B[8]*xuTmp[8];
                k3[6]  = (xuTmp[9]  - xuTmp[6]) * invTXYN[0];
                k3[7]  = (xuTmp[10] - xuTmp[7]) * invTXYN[1];
                k3[8]  = (xuTmp[11] - xuTmp[8]) * invTXYN[2];
                k3[9]  = (uTau[0] - xuTmp[9]) * invTf123[0];
                k3[10] = (uTau[1] - xuTmp[10]) * invTf123[1];
                k3[11] = (uTau[2] - xuTmp[11]) * invTf123[2];

                // RK4: k4
                xuTmp[0]  = xu[0]  + sampletime * k3[0];
                xuTmp[1]  = xu[1]  + sampletime * k3[1];
                xuTmp[2]  = xu[2]  + sampletime * k3[2];
                xuTmp[3]  = xu[3]  + sampletime * k3[3];
                xuTmp[4]  = xu[4]  + sampletime * k3[4];
                xuTmp[5]  = xu[5]  + sampletime * k3[5];
                xuTmp[6]  = xu[6]  + sampletime * k3[6];
                xuTmp[7]  = xu[7]  + sampletime * k3[7];
                xuTmp[8]  = xu[8]  + sampletime * k3[8];
                xuTmp[9]  = xu[9]  + sampletime * k3[9];
                xuTmp[10] = xu[10] + sampletime * k3[10];
                xuTmp[11] = xu[11] + sampletime * k3[11];
                c = std::cos(xuTmp[2]);
                s = std::sin(xuTmp[2]);
                uv = xuTmp[3] * xuTmp[4];
                ur = xuTmp[3] * xuTmp[5];
                vr = xuTmp[4] * xuTmp[5];
                uu = xuTmp[3] * xuTmp[3];
                vv = xuTmp[4] * xuTmp[4];
                rr = xuTmp[5] * xuTmp[5];
                uuu = uu * xuTmp[3];
                vvv = vv * xuTmp[4];
                rrr = rr * xuTmp[5];
                k4[0]  = c*xuTmp[3] - s*xuTmp[4];
                k4[1]  = s*xuTmp[3] + c*xuTmp[4];
                k4[2]  = xuTmp[5];
                k4[3]  =  F[0]*xuTmp[3] +  F[1]*xuTmp[4] +  F[2]*xuTmp[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*xuTmp[6] + B[1]*xuTmp[7] + B[2]*xuTmp[8];
                k4[4]  = F[12]*xuTmp[3] + F[13]*xuTmp[4] + F[14]*xuTmp[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*xuTmp[6] + B[4]*xuTmp[7] + B[5]*xuTmp[8];
                k4[5]  = F[24]*xuTmp[3] + F[25]*xuTmp[4] + F[26]*xuTmp[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*xuTmp[6] + B[7]*xuTmp[7] + B[8]*xuTmp[8];
                k4[6]  = (xuTmp[9]  - xuTmp[6]) * invTXYN[0];
                k4[7]  = (xuTmp[10] - xuTmp[7]) * invTXYN[1];
                k4[8]  = (xuTmp[11] - xuTmp[8]) * invTXYN[2];
                k4[9]  = (uTau[0] - xuTmp[9]) * invTf123[0];
                k4[10] = (uTau[1] - xuTmp[10]) * invTf123[1];
                k4[11] = (uTau[2] - xuTmp[11]) * invTf123[2];

                // Predicted state according to RK4
                xu[0]  += h6 * (k1[0]  + 2.0*(k2[0]  + k3[0])  + k4[0]);
                xu[1]  += h6 * (k1[1]  + 2.0*(k2[1]  + k3[1])  + k4[1]);
                xu[2]  += h6 * (k1[2]  + 2.0*(k2[2]  + k3[2])  + k4[2]);
                xu[3]  += h6 * (k1[3]  + 2.0*(k2[3]  + k3[3])  + k4[3]);
                xu[4]  += h6 * (k1[4]  + 2.0*(k2[4]  + k3[4])  + k4[4]);
                xu[5]  += h6 * (k1[5]  + 2.0*(k2[5]  + k3[5])  + k4[5]);
                xu[6]  += h6 * (k1[6]  + 2.0*(k2[6]  + k3[6])  + k4[6]);
                xu[7]  += h6 * (k1[7]  + 2.0*(k2[7]  + k3[7])  + k4[7]);
                xu[8]  += h6 * (k1[8]  + 2.0*(k2[8]  + k3[8])  + k4[8]);
                xu[9]  += h6 * (k1[9]  + 2.0*(k2[9]  + k3[9])  + k4[9]);
                xu[10] += h6 * (k1[10] + 2.0*(k2[10] + k3[10]) + k4[10]);
                xu[11] += h6 * (k1[11] + 2.0*(k2[11] + k3[11]) + k4[11]);

                // xu contains (xk,uk), copy them to the trajectory
                xu[2] = mpsv::math::SymmetricalAngle(xu[2]);
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
        std::array<double,9> PredictMotion(const std::array<double,12>& initialStateAndInput, double sampletime, uint32_t numSimulationSteps) noexcept {
            // Start from the initial state
            std::array<double,9> predictedState;
            std::copy(initialStateAndInput.begin(), initialStateAndInput.begin() + 9, predictedState.begin());

            // Simulate step by step
            double h2 = sampletime / 2.0;
            double h6 = sampletime / 6.0;
            double c, s;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            std::array<double,9> x, k1, k2, k3, k4;
            for(uint32_t k = 0; k != numSimulationSteps; ++k){
                // RK4: k1
                x = predictedState;
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k1[0] = (c*x[3] - s*x[4]);
                k1[1] = (s*x[3] + c*x[4]);
                k1[2] = (x[5]);
                k1[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k1[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k1[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k1[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k1[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k1[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k2
                x[0] = predictedState[0] + h2 * k1[0];
                x[1] = predictedState[1] + h2 * k1[1];
                x[2] = predictedState[2] + h2 * k1[2];
                x[3] = predictedState[3] + h2 * k1[3];
                x[4] = predictedState[4] + h2 * k1[4];
                x[5] = predictedState[5] + h2 * k1[5];
                x[6] = predictedState[6] + h2 * k1[6];
                x[7] = predictedState[7] + h2 * k1[7];
                x[8] = predictedState[8] + h2 * k1[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k2[0] = (c*x[3] - s*x[4]);
                k2[1] = (s*x[3] + c*x[4]);
                k2[2] = (x[5]);
                k2[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k2[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k2[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k2[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k2[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k2[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k3
                x[0] = predictedState[0] + h2 * k2[0];
                x[1] = predictedState[1] + h2 * k2[1];
                x[2] = predictedState[2] + h2 * k2[2];
                x[3] = predictedState[3] + h2 * k2[3];
                x[4] = predictedState[4] + h2 * k2[4];
                x[5] = predictedState[5] + h2 * k2[5];
                x[6] = predictedState[6] + h2 * k2[6];
                x[7] = predictedState[7] + h2 * k2[7];
                x[8] = predictedState[8] + h2 * k2[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k3[0] = (c*x[3] - s*x[4]);
                k3[1] = (s*x[3] + c*x[4]);
                k3[2] = (x[5]);
                k3[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k3[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k3[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k3[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k3[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k3[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k4
                x[0] = predictedState[0] + sampletime * k3[0];
                x[1] = predictedState[1] + sampletime * k3[1];
                x[2] = predictedState[2] + sampletime * k3[2];
                x[3] = predictedState[3] + sampletime * k3[3];
                x[4] = predictedState[4] + sampletime * k3[4];
                x[5] = predictedState[5] + sampletime * k3[5];
                x[6] = predictedState[6] + sampletime * k3[6];
                x[7] = predictedState[7] + sampletime * k3[7];
                x[8] = predictedState[8] + sampletime * k3[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k4[0] = (c*x[3] - s*x[4]);
                k4[1] = (s*x[3] + c*x[4]);
                k4[2] = (x[5]);
                k4[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k4[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k4[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k4[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k4[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k4[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // Predicted state according to RK4
                predictedState[0] += h6 * (k1[0] + 2.0*(k2[0] + k3[0]) + k4[0]);
                predictedState[1] += h6 * (k1[1] + 2.0*(k2[1] + k3[1]) + k4[1]);
                predictedState[2] += h6 * (k1[2] + 2.0*(k2[2] + k3[2]) + k4[2]);
                predictedState[3] += h6 * (k1[3] + 2.0*(k2[3] + k3[3]) + k4[3]);
                predictedState[4] += h6 * (k1[4] + 2.0*(k2[4] + k3[4]) + k4[4]);
                predictedState[5] += h6 * (k1[5] + 2.0*(k2[5] + k3[5]) + k4[5]);
                predictedState[6] += h6 * (k1[6] + 2.0*(k2[6] + k3[6]) + k4[6]);
                predictedState[7] += h6 * (k1[7] + 2.0*(k2[7] + k3[7]) + k4[7]);
                predictedState[8] += h6 * (k1[8] + 2.0*(k2[8] + k3[8]) + k4[8]);
            }
            predictedState[2] = mpsv::math::SymmetricalAngle(predictedState[2]);
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
        bool PredictMotionTrajectory(std::vector<std::array<double,12>>& trajectory, const std::array<double,12>& initialStateAndInput, double sampletime, uint32_t numSimulationSteps, const uint32_t maxNumSimulationSteps) noexcept {
            // Start from the initial state
            std::array<double,9> predictedState;
            std::copy(initialStateAndInput.begin(), initialStateAndInput.begin() + 9, predictedState.begin());

            // Simulate step by step
            double h2 = sampletime / 2.0;
            double h6 = sampletime / 6.0;
            double c, s;
            double uv, ur, vr, uu, vv, rr, uuu, vvv, rrr;
            std::array<double,9> x, k1, k2, k3, k4;
            trajectory.clear();
            if(numSimulationSteps >= maxNumSimulationSteps){
                return false;
            }
            numSimulationSteps = std::max(numSimulationSteps, static_cast<uint32_t>(1));
            trajectory.reserve(numSimulationSteps);
            for(uint32_t k = 0; k != numSimulationSteps; ++k){
                // RK4: k1
                x = predictedState;
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k1[0] = (c*x[3] - s*x[4]);
                k1[1] = (s*x[3] + c*x[4]);
                k1[2] = (x[5]);
                k1[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k1[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k1[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k1[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k1[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k1[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k2
                x[0] = predictedState[0] + h2 * k1[0];
                x[1] = predictedState[1] + h2 * k1[1];
                x[2] = predictedState[2] + h2 * k1[2];
                x[3] = predictedState[3] + h2 * k1[3];
                x[4] = predictedState[4] + h2 * k1[4];
                x[5] = predictedState[5] + h2 * k1[5];
                x[6] = predictedState[6] + h2 * k1[6];
                x[7] = predictedState[7] + h2 * k1[7];
                x[8] = predictedState[8] + h2 * k1[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k2[0] = (c*x[3] - s*x[4]);
                k2[1] = (s*x[3] + c*x[4]);
                k2[2] = (x[5]);
                k2[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k2[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k2[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k2[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k2[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k2[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k3
                x[0] = predictedState[0] + h2 * k2[0];
                x[1] = predictedState[1] + h2 * k2[1];
                x[2] = predictedState[2] + h2 * k2[2];
                x[3] = predictedState[3] + h2 * k2[3];
                x[4] = predictedState[4] + h2 * k2[4];
                x[5] = predictedState[5] + h2 * k2[5];
                x[6] = predictedState[6] + h2 * k2[6];
                x[7] = predictedState[7] + h2 * k2[7];
                x[8] = predictedState[8] + h2 * k2[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k3[0] = (c*x[3] - s*x[4]);
                k3[1] = (s*x[3] + c*x[4]);
                k3[2] = (x[5]);
                k3[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k3[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k3[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k3[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k3[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k3[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // RK4: k4
                x[0] = predictedState[0] + sampletime * k3[0];
                x[1] = predictedState[1] + sampletime * k3[1];
                x[2] = predictedState[2] + sampletime * k3[2];
                x[3] = predictedState[3] + sampletime * k3[3];
                x[4] = predictedState[4] + sampletime * k3[4];
                x[5] = predictedState[5] + sampletime * k3[5];
                x[6] = predictedState[6] + sampletime * k3[6];
                x[7] = predictedState[7] + sampletime * k3[7];
                x[8] = predictedState[8] + sampletime * k3[8];
                c = std::cos(x[2]);
                s = std::sin(x[2]);
                uv = x[3] * x[4];
                ur = x[3] * x[5];
                vr = x[4] * x[5];
                uu = x[3] * x[3];
                vv = x[4] * x[4];
                rr = x[5] * x[5];
                uuu = uu * x[3];
                vvv = vv * x[4];
                rrr = rr * x[5];
                k4[0] = (c*x[3] - s*x[4]);
                k4[1] = (s*x[3] + c*x[4]);
                k4[2] = (x[5]);
                k4[3] = ( F[0]*x[3] +  F[1]*x[4] +  F[2]*x[5] +  F[3]*vr +  F[4]*ur +  F[5]*uv +  F[6]*uu +  F[7]*vv +  F[8]*rr +  F[9]*uuu + F[10]*vvv + F[11]*rrr + B[0]*x[6] + B[1]*x[7] + B[2]*x[8]);
                k4[4] = (F[12]*x[3] + F[13]*x[4] + F[14]*x[5] + F[15]*vr + F[16]*ur + F[17]*uv + F[18]*uu + F[19]*vv + F[20]*rr + F[21]*uuu + F[22]*vvv + F[23]*rrr + B[3]*x[6] + B[4]*x[7] + B[5]*x[8]);
                k4[5] = (F[24]*x[3] + F[25]*x[4] + F[26]*x[5] + F[27]*vr + F[28]*ur + F[29]*uv + F[30]*uu + F[31]*vv + F[32]*rr + F[33]*uuu + F[34]*vvv + F[35]*rrr + B[6]*x[6] + B[7]*x[7] + B[8]*x[8]);
                k4[6] = (initialStateAndInput[9]  - x[6]) * invTXYN[0];
                k4[7] = (initialStateAndInput[10] - x[7]) * invTXYN[1];
                k4[8] = (initialStateAndInput[11] - x[8]) * invTXYN[2];

                // Predicted state according to RK4
                predictedState[0] += h6 * (k1[0] + 2.0*(k2[0] + k3[0]) + k4[0]);
                predictedState[1] += h6 * (k1[1] + 2.0*(k2[1] + k3[1]) + k4[1]);
                predictedState[2] += h6 * (k1[2] + 2.0*(k2[2] + k3[2]) + k4[2]);
                predictedState[3] += h6 * (k1[3] + 2.0*(k2[3] + k3[3]) + k4[3]);
                predictedState[4] += h6 * (k1[4] + 2.0*(k2[4] + k3[4]) + k4[4]);
                predictedState[5] += h6 * (k1[5] + 2.0*(k2[5] + k3[5]) + k4[5]);
                predictedState[6] += h6 * (k1[6] + 2.0*(k2[6] + k3[6]) + k4[6]);
                predictedState[7] += h6 * (k1[7] + 2.0*(k2[7] + k3[7]) + k4[7]);
                predictedState[8] += h6 * (k1[8] + 2.0*(k2[8] + k3[8]) + k4[8]);

                // Add new trajectory entry
                predictedState[2] = mpsv::math::SymmetricalAngle(predictedState[2]);
                trajectory.push_back({predictedState[0], predictedState[1], predictedState[2], predictedState[3], predictedState[4], predictedState[5], predictedState[6], predictedState[7], predictedState[8], initialStateAndInput[9], initialStateAndInput[10], initialStateAndInput[11]});
            }
            return true;
        }

        /**
         * @brief Get the closest interpolated pose to a path based on the rotated ellipsoid metric of the guidance law.
         * @param[in] pose Input pose that denotes the current vehicle pose (x,y,psi).
         * @param[in] path Input path desribed by a list of poses. Each point indicates (x,y,psi).
         * @param[in] staticObstacles List of static obstacles to be used to limit the exploration radius near obstacles.
         * @return A tuple containing the following values.
         * [0]: Closest interpolated pose. If the input path is empty, then the given input pose is returned.
         * [1]: Number of poses to be deleted from the beginning of the given path to obtain a subpath that would start after the closest interpolated pose.
         */
        /*template <class T> std::tuple<std::array<double,3>, size_t> GetClosestInterpolatedPoseToPath(T pose, const std::vector<std::array<double,3>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            // To find the closest interpolation pose to a path based on an rotated ellipsoid metric, the path is transformed into a corresponding
            // euclidean space R^3, whose origin is located at the vehicles pose. The space transformation is done in such a way, that
            // all calculations simplifiy to the closest point calculation of a line with in 3D space. For a given vehicle pose [x; y; psi] any pose
            // of the path [x_i; y_i; psi_i] can be transformed into the euclidean space by
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
            // All poses of the path are transformed into the euclidean space and eta denotes the origin of that space. Then the closest
            // interapolated pose along the path is searched for.
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
            // In this example, the index to p2 indicates the starting index of the subpath that is kept. The number of poses to be deleted
            // would be 2. If the closest interpolated pose would be equal to the final pose p4, then the index would point to a virtual p5,
            // indicating, that none of the poses from the path are used for the subpath. The number of poses to be deleted would be 5.

            // Some variables and constants
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            double lambda, dx, dy, dz, denominator, squaredDistance, lowestSquaredDistance;
            double tmp1, tmp2, tmp3;
            std::array<double,3> pa, pb;
            const size_t numWaypoints = path.size();
            double cosPsi = std::cos(pose[2]);
            double sinPsi = std::sin(pose[2]);

            // Return values
            std::tuple<std::array<double,3>,size_t> result;
            std::array<double,3>& resultClosestInterpolatedPose = std::get<0>(result);
            size_t resultStartingIndexSubpath = std::get<1>(result);

            // Fallback value if path is empty
            resultClosestInterpolatedPose[0] = pose[0];
            resultClosestInterpolatedPose[1] = pose[1];
            resultClosestInterpolatedPose[2] = pose[2];
            resultStartingIndexSubpath = 0;
            if(!numWaypoints){
                return result;
            }

            // Clamp position radii to maximum value (e.g. distance to obstacles)
            double minSquaredDistance = std::numeric_limits<double>::infinity();
            for(auto&& obstacle : staticObstacles){
                minSquaredDistance = std::min(minSquaredDistance, obstacle.MinimumSquaredDistanceToEdges(pose[0], pose[1]));
            }
            double limitedInvRR = 0.0;
            if(std::isfinite(minSquaredDistance)){
                limitedInvRR = 1.0 / std::max(minSquaredDistance, minSquaredRadiusPosition);
            }
            double invRX = std::max(invSquaredRadiusX, limitedInvRR);
            double invRY = std::max(invSquaredRadiusY, limitedInvRR);

            // Get the closest interpolated pose on the path to the given pose
            if(numWaypoints > 1){
                std::array<size_t,2> idxSegment = {0, 0}; // stores the indices of the start and end point from a line segment
                double interpolation = 0.0; // stores the lambda value (interpolation ratio) for that line segment

                // Go through all line segments and search for the lowest squared distance
                lowestSquaredDistance = std::numeric_limits<double>::infinity();

                // Initial starting point pa for first segment (pa without 1/r scale)
                dx = path[0][0] - pose[0];
                dy = path[0][1] - pose[1];
                pa[0] = cosPsi * dx + sinPsi * dy;
                pa[1] = -sinPsi * dx + cosPsi * dy;
                pa[2] = mpsv::math::SymmetricalAngle(path[0][2] - pose[2]);
                for(size_t n = 1; n < path.size(); ++n){
                    // For each line segment: calculate the interpolation factor lambda that minimizes the distance from pose to the current line segment
                    // Starting point pa is already given, calculate ending point pb for this line segment (also without 1/r scale)
                    dx = path[n][0] - pose[0];
                    dy = path[n][1] - pose[1];
                    pb[0] = cosPsi * dx + sinPsi * dy;
                    pb[1] = -sinPsi * dx + cosPsi * dy;
                    pb[2] = mpsv::math::SymmetricalAngle(path[n][2] - pose[2]);

                    // Calculate lambda of p = pa + lambda*(pb - pa), where p is closest to the origin (here we have to take into account the 1/r scale)
                    // lambda = (pa' * (pa - pb) / ((pa - pb)' * (pa - pb)))
                    dx = pa[0] - pb[0];
                    dy = pa[1] - pb[1];
                    dz = pa[2] - pb[2];
                    tmp1 = invRX*dx;
                    tmp2 = invRY*dy;
                    tmp3 = invSquaredRadiusPsi*dz;
                    denominator = tmp1*dx + tmp2*dy + tmp3*dz;
                    lambda = 0.0;
                    if(std::fabs(denominator) > eps){
                        lambda = (tmp1*pa[0] + tmp2*pa[1] + tmp3*pa[2]) / denominator;
                    }

                    // Clamp lambda to line segment [0, 1]
                    lambda = std::clamp(lambda, 0.0, 1.0);

                    // Calculate squared distance from pI = [sx; sy; sz] from origin in transformed R3 space and check if distance is reduced
                    pa[0] -= lambda * dx;
                    pa[1] -= lambda * dy;
                    pa[2] -= lambda * dz;
                    squaredDistance = invRX*pa[0]*pa[0] + invRY*pa[1]*pa[1] + invSquaredRadiusPsi*pa[2]*pa[2];
                    if(squaredDistance < lowestSquaredDistance){
                        lowestSquaredDistance = squaredDistance;
                        idxSegment[0] = n - 1;
                        idxSegment[1] = n;
                        interpolation = lambda;
                    }

                    // Set the ending point of this line segment as starting point for the next line segment
                    pa = pb;
                }
                resultClosestInterpolatedPose = mpsv::math::PoseDifference(path[idxSegment[1]], path[idxSegment[0]]);
                resultClosestInterpolatedPose[0] = path[idxSegment[0]][0] + interpolation * resultClosestInterpolatedPose[0];
                resultClosestInterpolatedPose[1] = path[idxSegment[0]][1] + interpolation * resultClosestInterpolatedPose[1];
                resultClosestInterpolatedPose[2] = mpsv::math::SymmetricalAngle(path[idxSegment[0]][2] + interpolation * resultClosestInterpolatedPose[2]);
                resultStartingIndexSubpath = idxSegment[1];
                if((interpolation + eps) >= 1.0){
                    ++resultStartingIndexSubpath;
                }
            }
            else{
                resultClosestInterpolatedPose = path[0];
                resultClosestInterpolatedPose[2] = mpsv::math::SymmetricalAngle(resultClosestInterpolatedPose[2]);
                resultStartingIndexSubpath = 1;
            }
            return result;
        }*/


    protected:
        /* model parameters and precalculated values set by @ref SetModel */
        std::array<double,36> F;              // 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double,9> B;               // 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double,9> invB;            // Inverse of input matrix B.
        std::array<double,3> invTXYN;         // {1/TX, 1/TY, 1/TN}.
        std::array<double,3> invTf123;        // {1/Tf1, 1/Tf2, 1/Tf3}.
        std::array<double,9> invBBB;          // Matrix inverse of (B * B_tau * B_c) (row-major order).
        std::array<double,9> M0;              // Row-major order of M0 = B * A_tau.
        std::array<double,3> lowerLimitXYN;   // Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
        std::array<double,3> upperLimitXYN;   // Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.

        /* precalculated coefficients for first derivatives set by @ref SetModel */
        double f17_2;                         // 2 * f_17.
        double f18_2;                         // 2 * f_18.
        double f19_2;                         // 2 * f_19.
        double f1a_3;                         // 3 * f_1a.
        double f1b_3;                         // 3 * f_1b.
        double f1c_3;                         // 3 * f_1c.

        double f27_2;                         // 2 * f_27.
        double f28_2;                         // 2 * f_28.
        double f29_2;                         // 2 * f_29.
        double f2a_3;                         // 3 * f_2a.
        double f2b_3;                         // 3 * f_2b.
        double f2c_3;                         // 3 * f_2c.

        double f37_2;                         // 2 * f_37.
        double f38_2;                         // 2 * f_38.
        double f39_2;                         // 2 * f_39.
        double f3a_3;                         // 3 * f_3a.
        double f3b_3;                         // 3 * f_3b.
        double f3c_3;                         // 3 * f_3c.

        /* precalculated coefficients for second derivatives set by @ref SetModel */
        double f1a_6;                         // 6 * f_1a.
        double f1b_6;                         // 6 * f_1b.
        double f1c_6;                         // 6 * f_1c.

        double f2a_6;                         // 6 * f_2a.
        double f2b_6;                         // 6 * f_2b.
        double f2c_6;                         // 6 * f_2c.

        double f3a_6;                         // 6 * f_3a.
        double f3b_6;                         // 6 * f_3b.
        double f3c_6;                         // 6 * f_3c.

        /* controller parameters and precalculated values set by @ref SetController */
        double minSquaredRadiusPosition;      // r^2 with r being the minimum radius for position for the waypoint guidance law.
        double invSquaredRadiusX;             // 1 / r^2 with r being the radius for the longitudinal distance for the waypoint guidance law.
        double invSquaredRadiusY;             // 1 / r^2 with r being the radius for the lateral distance for the waypoint guidance law.
        double invSquaredRadiusPsi;           // 1 / r^2 with r being the radius for the angular distrance for the waypoint guidance law.
        std::array<double,36> K;              // 3-by-12 control gain matrix (row-major order) for pose control (state controller using underlying velocity controller based on feedback-linearization).
        std::array<double,9> Kz;              // Diagonal elements for gain matrix of flatness-based velocity control.

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
        void PathGuidance(std::array<double,3>& targetPose, std::array<double,3> pose, double cosPsi, double sinPsi, const std::vector<std::array<double,3>>& path, const std::vector<mpsv::geometry::StaticObstacle>& staticObstacles) noexcept {
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
            constexpr double eps = 100.0 * std::numeric_limits<double>::epsilon();
            double a, bHalf, c, sr, lambda, dx, dy, dz, L, denominator, squaredDistance, lowestSquaredDistance;
            double tmp1, tmp2, tmp3;
            std::array<double,3> pa, pb;
            const size_t numWaypoints = path.size();

            // Fallback value if path is empty
            if(!numWaypoints){
                targetPose = pose;
                return;
            }

            // Clamp position radii to maximum value (e.g. distance to obstacles)
            double minSquaredDistance = std::numeric_limits<double>::infinity();
            for(auto&& obstacle : staticObstacles){
                minSquaredDistance = std::min(minSquaredDistance, obstacle.MinimumSquaredDistanceToEdges(pose[0], pose[1]));
            }
            double limitedInvRR = 0.0;
            if(std::isfinite(minSquaredDistance)){
                limitedInvRR = 1.0 / std::max(minSquaredDistance, minSquaredRadiusPosition);
            }
            double invRX = std::max(invSquaredRadiusX, limitedInvRR);
            double invRY = std::max(invSquaredRadiusY, limitedInvRR);

            // STEP 1: Get the closest interpolated pose on the path to the given pose
            std::array<size_t,2> idxSegment = {0, 0}; // stores the indices of the start and end point from a line segment
            double interpolation = 0.0; // stores the lambda value (interpolation ratio) for that line segment
            if(numWaypoints > 1){
                // Go through all line segments and search for the lowest squared distance
                lowestSquaredDistance = std::numeric_limits<double>::infinity();

                // Initial starting point pa for first segment (pa without 1/r scale)
                dx = path[0][0] - pose[0];
                dy = path[0][1] - pose[1];
                pa[0] = cosPsi * dx + sinPsi * dy;
                pa[1] = -sinPsi * dx + cosPsi * dy;
                pa[2] = mpsv::math::SymmetricalAngle(path[0][2] - pose[2]);
                for(size_t n = 1; n < path.size(); ++n){
                    // For each line segment: calculate the interpolation factor lambda that minimizes the distance from pose to the current line segment
                    // Starting point pa is already given, calculate ending point pb for this line segment (also without 1/r scale)
                    dx = path[n][0] - pose[0];
                    dy = path[n][1] - pose[1];
                    pb[0] = cosPsi * dx + sinPsi * dy;
                    pb[1] = -sinPsi * dx + cosPsi * dy;
                    pb[2] = mpsv::math::SymmetricalAngle(path[n][2] - pose[2]);

                    // Calculate lambda of p = pa + lambda*(pb - pa), where p is closest to the origin (here we have to take into account the 1/r scale)
                    // lambda = (pa' * (pa - pb) / ((pa - pb)' * (pa - pb)))
                    dx = pa[0] - pb[0];
                    dy = pa[1] - pb[1];
                    dz = pa[2] - pb[2];
                    tmp1 = invRX*dx;
                    tmp2 = invRY*dy;
                    tmp3 = invSquaredRadiusPsi*dz;
                    denominator = tmp1*dx + tmp2*dy + tmp3*dz;
                    lambda = 0.0;
                    if(std::fabs(denominator) > eps){
                        lambda = (tmp1*pa[0] + tmp2*pa[1] + tmp3*pa[2]) / denominator;
                    }

                    // Clamp lambda to line segment [0, 1]
                    lambda = std::clamp(lambda, 0.0, 1.0);

                    // Calculate squared distance from pI = [sx; sy; sz] from origin in transformed R3 space and check if distance is reduced
                    pa[0] -= lambda * dx;
                    pa[1] -= lambda * dy;
                    pa[2] -= lambda * dz;
                    squaredDistance = invRX*pa[0]*pa[0] + invRY*pa[1]*pa[1] + invSquaredRadiusPsi*pa[2]*pa[2];
                    if(squaredDistance < lowestSquaredDistance){
                        lowestSquaredDistance = squaredDistance;
                        idxSegment[0] = n - 1;
                        idxSegment[1] = n;
                        interpolation = lambda;
                    }

                    // Set the ending point of this line segment as starting point for the next line segment
                    pa = pb;
                }
            }
            std::array<double,3> interpolatedPose = mpsv::math::PoseDifference(path[idxSegment[1]], path[idxSegment[0]]);
            interpolatedPose[0] = path[idxSegment[0]][0] + interpolation * interpolatedPose[0];
            interpolatedPose[1] = path[idxSegment[0]][1] + interpolation * interpolatedPose[1];
            interpolatedPose[2] = mpsv::math::SymmetricalAngle(path[idxSegment[0]][2] + interpolation * interpolatedPose[2]);

            // STEP 2: Find the target pose, that is, the intersection of the ellipsoid with the path.
            // The path to be considered starts from the given pose and continues with the closest interpolated pose followed by all remaining poses
            // from the input path. The order is: pose --> interpolatedPose --> path[idxSegment[1]] --> ... --> path[end]. Because this path starts
            // at the pose, there's always an intersection of the ellipsoid with the line segment if the length of the linesegment is non-zero.

            // Line segment 1: pose --> interpolatedPose
            // We set pa to the interpolated pose (without 1/r scale) because this is the starting point for the next line segment
            dx = interpolatedPose[0] - pose[0];
            dy = interpolatedPose[1] - pose[1];
            pa[0] = cosPsi * dx + sinPsi * dy;
            pa[1] = -sinPsi * dx + cosPsi * dy;
            pa[2] = mpsv::math::SymmetricalAngle(interpolatedPose[2] - pose[2]);
            L = invRX*pa[0]*pa[0] + invRY*pa[1]*pa[1] + invSquaredRadiusPsi*pa[2]*pa[2]; // take into account 1/r scale
            if(L > 1.0){
                lambda = 1.0 / std::sqrt(L);
                targetPose[0] = pose[0] + lambda * dx;
                targetPose[1] = pose[1] + lambda * dy;
                targetPose[2] = mpsv::math::SymmetricalAngle(pose[2] + lambda * pa[2]);
                return;
            }
            targetPose = interpolatedPose;

            // Line segment 2: interpolatedPose --> path[idxSegment[1]]
            // pa is already known from line segment 1, calculate end point pb (also without 1/r scale)
            dx = path[idxSegment[1]][0] - pose[0];
            dy = path[idxSegment[1]][1] - pose[1];
            pb[0] = cosPsi * dx + sinPsi * dy;
            pb[1] = -sinPsi * dx + cosPsi * dy;
            pb[2] = mpsv::math::SymmetricalAngle(path[idxSegment[1]][2] - pose[2]);
            dx = pb[0] - pa[0];
            dy = pb[1] - pa[1];
            dz = pb[2] - pa[2];
            tmp1 = invRX*dx;
            tmp2 = invRY*dy;
            tmp3 = invSquaredRadiusPsi*dz;
            a = tmp1*dx + tmp2*dy + tmp3*dz;
            bHalf = (tmp1*pa[0] + tmp2*pa[1] + tmp3*pa[2]);
            c = invRX*pa[0]*pa[0] + invRY*pa[1]*pa[1] + invSquaredRadiusPsi*pa[2]*pa[2] - 1.0;
            if(a > eps){
                sr = std::sqrt(bHalf * bHalf - a * c);
                lambda = std::max((-bHalf + sr) / a, (-bHalf - sr) / a);
                if(lambda <= 1.0){
                    targetPose[0] = interpolatedPose[0] + lambda * (path[idxSegment[1]][0] - interpolatedPose[0]);
                    targetPose[1] = interpolatedPose[1] + lambda * (path[idxSegment[1]][1] - interpolatedPose[1]);
                    targetPose[2] = mpsv::math::SymmetricalAngle(interpolatedPose[2] + lambda * mpsv::math::SymmetricalAngle(path[idxSegment[1]][2] - interpolatedPose[2]));
                    return;
                }
            }
            targetPose = path[idxSegment[1]];

            // Line segments 3,...: path[idxSegment[1]] --> ... --> path[end]
            for(size_t i = idxSegment[1] + 1; i < path.size(); ++i){
                // Start point pa is previous end point pb
                pa = pb;

                // Calculate the endpoint of this line segment (again without 1/r scale)
                dx = path[i][0] - pose[0];
                dy = path[i][1] - pose[1];
                pb[0] = cosPsi * dx + sinPsi * dy;
                pb[1] = -sinPsi * dx + cosPsi * dy;
                pb[2] = mpsv::math::SymmetricalAngle(path[i][2] - pose[2]);
                dx = pb[0] - pa[0];
                dy = pb[1] - pa[1];
                dz = pb[2] - pa[2];
                tmp1 = invRX*dx;
                tmp2 = invRY*dy;
                tmp3 = invSquaredRadiusPsi*dz;
                a = tmp1*dx + tmp2*dy + tmp3*dz;
                bHalf = (tmp1*pa[0] + tmp2*pa[1] + tmp3*pa[2]);
                c = invRX*pa[0]*pa[0] + invRY*pa[1]*pa[1] + invSquaredRadiusPsi*pa[2]*pa[2] - 1.0;
                if(a > eps){
                    sr = std::sqrt(bHalf * bHalf - a * c);
                    lambda = std::max((-bHalf + sr) / a, (-bHalf - sr) / a);
                    if(lambda <= 1.0){
                        targetPose[0] = path[i - 1][0] + lambda * (path[i][0] - path[i - 1][0]);
                        targetPose[1] = path[i - 1][1] + lambda * (path[i][1] - path[i - 1][1]);
                        targetPose[2] = mpsv::math::SymmetricalAngle(path[i - 1][2] + lambda * mpsv::math::SymmetricalAngle(path[i][2] - path[i - 1][2]));
                        return;
                    }
                }
                targetPose = path[i];
            }
            targetPose[2] = mpsv::math::SymmetricalAngle(targetPose[2]);
        }
};


} /* namespace: control */


} /* namespace: mpsv */

