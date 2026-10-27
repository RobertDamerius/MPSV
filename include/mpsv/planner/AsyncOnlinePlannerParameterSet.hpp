#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the parameter set for the asynchronous online planning algorithm (online sequential planner).
 */
class AsyncOnlinePlannerParameterSet: public mpsv::planner::OnlinePlannerParameterSet {
    public:
        double timestamp{std::numeric_limits<double>::quiet_NaN()};   // The user-defined timestamp that indicates the parameters to be used. This parameter set is only applied, if this value changes. The default value is quiet_NaN.
        double timeoutInput{};                                        // Timeout in seconds for the input data.

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
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         * @details The vertex order of polygon data may be adjusted if possible.
         */
        [[nodiscard]] error_code IsValid(void) const noexcept {
            if(!std::isfinite(timestamp))
                return error_code::PARAM_ASYNCPLANNER_TIMESTAMP;
            if(!std::isfinite(timeoutInput))
                return error_code::PARAM_ASYNCPLANNER_TIMEOUT_INPUT;
            return mpsv::planner::OnlinePlannerParameterSet::IsValid();
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
