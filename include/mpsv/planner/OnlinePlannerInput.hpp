#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv::planner {


/**
 * @brief This class represents the input to the online planning algorithm (online sequential planner).
 */
class OnlinePlannerInput {
    public:
        double timestamp{};                                            // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint corresponding to @ref initialStateAndInput.
        std::array<double, 3U> originLLA{};                            // Geographical origin to which this input belongs to, given as {lat,lon,alt}.
        std::array<double, 12U> initialStateAndInput{};                // Initial state and input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
        std::array<double, 3U> finalPose{};                            // The final pose given as {x,y,psi}.
        std::array<double, 3U> samplingBoxCenterPose{};                // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double, 2U> samplingBoxDimension{};                 // Dimension of the sampling box along major and minor axes of the box.
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   // List of static obstacles, where each vertex is given as {x,y}.

        /**
         * @brief Clear the input and set default values.
         */
        void Clear() noexcept {
            timestamp = 0.0;
            originLLA.fill(0.0);
            initialStateAndInput.fill(0.0);
            finalPose.fill(0.0);
            samplingBoxCenterPose.fill(0.0);
            samplingBoxDimension.fill(0.0);
            staticObstacles.clear();
        }

        /**
         * @brief Check whether the attributes are valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!std::isfinite(timestamp)) {
                return error_code::INPUT_TIMESTAMP;
            }
            if (!(std::isfinite(originLLA[0U]) &&
                  std::isfinite(originLLA[1U]) &&
                  std::isfinite(originLLA[2U])))
                return error_code::INPUT_ORIGIN_LLA;
            if (!(std::isfinite(initialStateAndInput[0U]) &&
                  std::isfinite(initialStateAndInput[1U]) &&
                  std::isfinite(initialStateAndInput[2U]) &&
                  std::isfinite(initialStateAndInput[3U]) &&
                  std::isfinite(initialStateAndInput[4U]) &&
                  std::isfinite(initialStateAndInput[5U]) &&
                  std::isfinite(initialStateAndInput[6U]) &&
                  std::isfinite(initialStateAndInput[7U]) &&
                  std::isfinite(initialStateAndInput[8U]) &&
                  std::isfinite(initialStateAndInput[9U]) &&
                  std::isfinite(initialStateAndInput[10U]) &&
                  std::isfinite(initialStateAndInput[11U])))
                return error_code::INPUT_INITIAL_STATE_AND_INPUT;
            if (!(std::isfinite(finalPose[0U]) &&
                  std::isfinite(finalPose[1U]) &&
                  std::isfinite(finalPose[2U])))
                return error_code::INPUT_FINAL_POSE;
            if (!(std::isfinite(samplingBoxCenterPose[0U]) &&
                  std::isfinite(samplingBoxCenterPose[1U]) &&
                  std::isfinite(samplingBoxCenterPose[2U])))
                return error_code::INPUT_SAMPLING_BOX_CENTER_POSE;
            if (!(std::isfinite(samplingBoxDimension[0U]) &&
                  std::isfinite(samplingBoxDimension[1U])))
                return error_code::INPUT_SAMPLING_BOX_DIMENSION;
            mpsv::geometry::OrientedBox box;
            box.Create(samplingBoxCenterPose, samplingBoxDimension);
            if (!box.IsInside(initialStateAndInput)) {
                return error_code::INPUT_INITIAL_STATE_NOT_INSIDE_SAMPLING_BOX;
            }
            if (!box.IsInside(finalPose)) {
                return error_code::INPUT_FINAL_POSE_NOT_INSIDE_SAMPLING_BOX;
            }
            for (auto&& obstacle : staticObstacles) {
                if (!obstacle.IsValid()) {
                    return error_code::INPUT_STATIC_OBSTACLES;
                }
            }
            return error_code::NONE;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::OnlinePlannerInput& dst) noexcept {
            dst.timestamp = timestamp;
            dst.originLLA = originLLA;
            dst.initialStateAndInput = initialStateAndInput;
            dst.finalPose = finalPose;
            dst.samplingBoxCenterPose = samplingBoxCenterPose;
            dst.samplingBoxDimension = samplingBoxDimension;
            dst.staticObstacles.swap(staticObstacles);
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "timestamp", {1U}, &timestamp, sizeof(timestamp));
            file.WriteField("double", preString + "originLLA", {3U}, &originLLA[0U], sizeof(originLLA));
            file.WriteField("double", preString + "initialStateAndInput", {12U}, &initialStateAndInput[0U], sizeof(initialStateAndInput));
            file.WriteField("double", preString + "finalPose", {3U}, &finalPose[0U], sizeof(finalPose));
            file.WriteField("double", preString + "samplingBoxCenterPose", {3U}, &samplingBoxCenterPose[0U], sizeof(samplingBoxCenterPose));
            file.WriteField("double", preString + "samplingBoxDimension", {2U}, &samplingBoxDimension[0U], sizeof(samplingBoxDimension));
            file.WriteConvexPolygonsField(preString + "staticObstacles", staticObstacles);
        }
};


} /* namespace: mpsv::planner */
