#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the parameter set for the asynchronous online planning algorithm (online sequential planner).
 */
class AsyncOnlinePlannerParameterSet: public mpsv::planner::OnlinePlannerParameterSet {
    public:
        double timestamp;      // The user-defined timestamp that indicates the parameters to be used. This parameter set is only applied, if this value changes. The default value is quiet_NaN.
        double timeoutInput;   // Timeout in seconds for the input data.

        /**
         * @brief Construct a new input object for the asynchronous online planner parameters and set default values.
         */
        AsyncOnlinePlannerParameterSet() noexcept { Clear(); }

        /**
         * @brief Clear the parameters and set default values.
         */
        void Clear(void) noexcept {
            timestamp = std::numeric_limits<double>::quiet_NaN();
            timeoutInput = 0.0;
            mpsv::planner::OnlinePlannerParameterSet::Clear();
        }

        /**
         * @brief Check whether this parameter set is valid or not.
         * @return True if parameter set is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool valid = mpsv::planner::OnlinePlannerParameterSet::IsValid();
            valid &= std::isfinite(timestamp);
            valid &= std::isfinite(timeoutInput);
            return valid;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::AsyncOnlinePlannerParameterSet& dst) noexcept {
            mpsv::planner::OnlinePlannerParameterSet::MoveTo(dst);
            dst.timestamp = timestamp;
            dst.timeoutInput = timeoutInput;
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

