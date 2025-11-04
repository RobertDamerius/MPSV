#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the input to the path planning algorithm.
 */
class PathPlannerInput {
    public:
        std::array<double,3> initialPose;                              // Initial pose given as {x,y,psi}. The initial pose MUST be inside the sampling box defined by @ref samplingBoxCenterPose and @ref samplingBoxDimension.
        std::array<double,3> finalPose;                                // Final pose given as {x,y,psi}. The final pose MUST be inside the sampling box defined by @ref samplingBoxCenterPose and @ref samplingBoxDimension.
        std::array<double,2> originOldToNew;                           // Translation between two consecutive problems. If the origin of the previous problem is different to the origin of the new problem, then the tree must be transformed during a warm start. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
        std::array<double,3> samplingBoxCenterPose;                    // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double,2> samplingBoxDimension;                     // Dimension of the sampling box along major and minor axes of the box.
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   // List of static obstacles, where each vertex is given as {x,y}.

        /**
         * @brief Construct a new path planner input object and set default values.
         */
        PathPlannerInput() noexcept {
            initialPose.fill(0.0);
            finalPose.fill(0.0);
            originOldToNew.fill(0.0);
            samplingBoxCenterPose.fill(0.0);
            samplingBoxDimension.fill(0.0);
        }

        /**
         * @brief Check whether the attributes are valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         * @details The vertex order of polygon data may be adjusted if possible.
         */
        error_code IsValid(void) noexcept {
            if(!(std::isfinite(initialPose[0]) &&
                 std::isfinite(initialPose[1]) &&
                 std::isfinite(initialPose[2])))
                return error_code::INPUT_INITIAL_POSE;
            if(!(std::isfinite(finalPose[0]) &&
                 std::isfinite(finalPose[1]) &&
                 std::isfinite(finalPose[2])))
                return error_code::INPUT_FINAL_POSE;
            if(!(std::isfinite(originOldToNew[0]) &&
                 std::isfinite(originOldToNew[1])))
                return error_code::INPUT_ORIGIN_OLD_TO_NEW;
            if(!(std::isfinite(samplingBoxCenterPose[0]) &&
                 std::isfinite(samplingBoxCenterPose[1]) &&
                 std::isfinite(samplingBoxCenterPose[2])))
                return error_code::INPUT_SAMPLING_BOX_CENTER_POSE;
            if(!(std::isfinite(samplingBoxDimension[0]) &&
                 std::isfinite(samplingBoxDimension[1])))
                return error_code::INPUT_SAMPLING_BOX_DIMENSION;
            mpsv::geometry::OrientedBox box;
            box.Create(samplingBoxCenterPose, samplingBoxDimension);
            if(!box.IsInside(initialPose))
                return error_code::INPUT_INITIAL_POSE_NOT_INSIDE_SAMPLING_BOX;
            if(!box.IsInside(finalPose))
                return error_code::INPUT_FINAL_POSE_NOT_INSIDE_SAMPLING_BOX;
            for(auto&& obstacle : staticObstacles){
                if(!(obstacle.IsFinite() &&
                     obstacle.EnsureCorrectVertexOrder()))
                    return error_code::INPUT_STATIC_OBSTACLES;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "initialPose", {3}, &initialPose[0], sizeof(initialPose));
            file.WriteField("double", preString + "finalPose", {3}, &finalPose[0], sizeof(finalPose));
            file.WriteField("double", preString + "originOldToNew", {2}, &originOldToNew[0], sizeof(originOldToNew));
            file.WriteField("double", preString + "samplingBoxCenterPose", {3}, &samplingBoxCenterPose[0], sizeof(samplingBoxCenterPose));
            file.WriteField("double", preString + "samplingBoxDimension", {2}, &samplingBoxDimension[0], sizeof(samplingBoxDimension));
            file.WriteConvexPolygonsField(preString + "staticObstacles", staticObstacles);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

