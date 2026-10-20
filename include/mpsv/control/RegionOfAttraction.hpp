#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv {


namespace control {


/**
 * @brief This class represents a simple region of attraction based on a multi-dimensional box including constraints
 * for position, angle, velocity and angular velocity.
 */
class RegionOfAttraction {
    public:
        /**
         * @brief Create a new region of attraction object and set default values (zeros).
         */
        RegionOfAttraction() noexcept { Clear(); }

        /**
         * @brief Clear the region of attraction and set all box constraints to default values (zero).
         */
        void Clear(void) noexcept {
            m_rangePose.fill(0.0);
            m_rangeUVR.fill(0.0);
            m_rangeXYN.fill(0.0);
        }

        /**
         * @brief Set parameters for the region of attraction.
         * @param[in] parameter The parameter to be set.
         * @return mpsv::error_code::NONE if all parameters are valid and have been applied, a non-zero error code otherwise.
         */
        error_code SetParameter(const mpsv::planner::ParameterRegionOfAttraction& parameter) noexcept {
            error_code e = parameter.IsValid();
            if(error_code::NONE == e){
                m_rangePose = parameter.rangePose;
                m_rangeUVR = parameter.rangeUVR;
                m_rangeXYN = parameter.rangeXYN;
            }
            return e;
        }

        /**
         * @brief Check whether a state is in the region of attraction around a given pose or not. The state is inside the region of attraction if the pose error
         * between state and pose and the absolute velocity of the state is below a defined threshold.
         * @tparam T Template parameter should be of type std::array<double,N> with N >= 9.
         * @tparam U Template parameter should be of type std::array<double,N> with N >= 3.
         * @param[in] state The state to be checked, given as {x,y,psi,u,v,r,...}. Only the pose and the velocities are to be considered from that array.
         * @param[in] pose The pose defining the center for the region of attraction, given as {x,y,psi,...}. Only the pose is be considered from that array.
         * @return True if the state is inside the region of attraction, false otherwise.
         */
        template <class T, class U> bool IsInside(const T& state, const U& pose) noexcept {
            static_assert(mpsv::is_vec9<T>::value,"Argument {state} must be of type std::array<double,N> with N >= 9!");
            static_assert(mpsv::is_vec3<U>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            double dx = std::fabs(pose[0] - state[0]);
            double dy = std::fabs(pose[1] - state[1]);
            double dpsi = std::fabs(mpsv::math::SymmetricalAngle(pose[2] - state[2]));
            double dvn = std::fabs(state[3]);
            double dve = std::fabs(state[4]);
            double dr = std::fabs(state[5]);
            double dX = std::fabs(state[6]);
            double dY = std::fabs(state[7]);
            double dN = std::fabs(state[8]);
            return ((dx < m_rangePose[0]) && (dy < m_rangePose[1]) && (dpsi < m_rangePose[2]) && (dvn < m_rangeUVR[0]) && (dve < m_rangeUVR[1]) && (dr < m_rangeUVR[2]) && (dX < m_rangeXYN[0]) && (dY < m_rangeXYN[1]) && (dN < m_rangeXYN[2]));
        }

    protected:
        std::array<double,3> m_rangePose;   // The pose range {dx,dy,dpsi}. A given pose must be in this range {[x-dx, x+dx], [y-dy, y+dy], [psi-dpsi, psi+dpsi]} to be in the region of attraction.
        std::array<double,3> m_rangeUVR;    // The velocity range {du,dv,dr}. A given velocity must be in this range {[-du,du],[-dv,dv],[-dr,dr]} to be in the region of attraction.
        std::array<double,3> m_rangeXYN;    // The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.
};


} /* namespace: control */


} /* namespace: mpsv */
