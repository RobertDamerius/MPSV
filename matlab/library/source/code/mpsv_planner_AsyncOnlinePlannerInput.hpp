#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_planner_OnlinePlannerInput.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the input to the asynchronous online planning algorithm (online sequential planner).
 */
class AsyncOnlinePlannerInput: public mpsv::planner::OnlinePlannerInput {
    public:
        bool enable;            // True if the asynchronous online planner should be enabled, false if the planner should go into standby mode.
        bool reset;             // True if the asynchronous online planner should be reset. A reset is ensured to be performed before the next internal solve operation of the planner.

        /**
         * @brief Construct a new input object for the asynchronous online planner and set default values.
         */
        AsyncOnlinePlannerInput() noexcept { Clear(); }

        /**
         * @brief Clear the input and set default values.
         */
        void Clear(void) noexcept {
            mpsv::planner::OnlinePlannerInput::Clear();
            enable = false;
            reset = false;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::AsyncOnlinePlannerInput& dst) noexcept {
            mpsv::planner::OnlinePlannerInput::MoveTo(dst);
            dst.enable = enable;
            dst.reset = reset;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

