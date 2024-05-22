#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_geometry_StaticObstacle.hpp>
#include <mpsv_geometry_OrientedBox.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the input to the online planning algorithm (online sequential planner).
 */
class OnlinePlannerInput {
    public:
        double timestamp;                                              // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint corresponding to @ref initialStateAndInput.
        std::array<double,3> originLLA;                                // Geographical origin to which this input belongs to, given as {lat,lon,alt}.
        std::array<double,12> initialStateAndInput;                    // Initial state and input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
        std::array<double,3> finalPose;                                // The final pose given as {x,y,psi}.
        std::array<double,3> samplingBoxCenterPose;                    // Center pose of the sampling box given as {x,y,psi}. The angle indicates the orientation of the box.
        std::array<double,2> samplingBoxDimension;                     // Dimension of the sampling box along major and minor axes of the box.
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   // List of static obstacles, where each vertex is given as {x,y}.

        /**
         * @brief Construct a new online planner input object and set default values.
         */
        OnlinePlannerInput() noexcept { Clear(); }

        /**
         * @brief Clear the input and set default values.
         */
        void Clear(void) noexcept {
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
         * @return True if all attributes are valid, false otherwise.
         * @details Attributes are valid if all values are finite and static obstacles are indeed convex.
         */
        bool IsValid(void) noexcept {
            bool valid = std::isfinite(timestamp);
            valid &= std::isfinite(originLLA[0]) && std::isfinite(originLLA[1]) && std::isfinite(originLLA[2]);
            valid &= std::isfinite(initialStateAndInput[0]) && std::isfinite(initialStateAndInput[1]) && std::isfinite(initialStateAndInput[2]) && std::isfinite(initialStateAndInput[3]) && std::isfinite(initialStateAndInput[4]) && std::isfinite(initialStateAndInput[5]) && std::isfinite(initialStateAndInput[6]) && std::isfinite(initialStateAndInput[7]) && std::isfinite(initialStateAndInput[8]) && std::isfinite(initialStateAndInput[9]) && std::isfinite(initialStateAndInput[10]) && std::isfinite(initialStateAndInput[11]);
            valid &= std::isfinite(finalPose[0]) && std::isfinite(finalPose[1]) && std::isfinite(finalPose[2]);
            valid &= std::isfinite(samplingBoxCenterPose[0]) && std::isfinite(samplingBoxCenterPose[1]) && std::isfinite(samplingBoxCenterPose[2]);
            valid &= std::isfinite(samplingBoxDimension[0]) && std::isfinite(samplingBoxDimension[1]);
            mpsv::geometry::OrientedBox box;
            box.Create(samplingBoxCenterPose, samplingBoxDimension);
            valid &= box.IsInside(initialStateAndInput);
            valid &= box.IsInside(finalPose);
            for(auto&& obstacle : staticObstacles){
                valid &= obstacle.IsConvex() && obstacle.IsFinite();
            }
            return valid;
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
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "timestamp", {1}, &timestamp, sizeof(timestamp));
            file.WriteField("double", preString + "originLLA", {3}, &originLLA[0], sizeof(originLLA));
            file.WriteField("double", preString + "initialStateAndInput", {12}, &initialStateAndInput[0], sizeof(initialStateAndInput));
            file.WriteField("double", preString + "finalPose", {3}, &finalPose[0], sizeof(finalPose));
            file.WriteField("double", preString + "samplingBoxCenterPose", {3}, &samplingBoxCenterPose[0], sizeof(samplingBoxCenterPose));
            file.WriteField("double", preString + "samplingBoxDimension", {2}, &samplingBoxDimension[0], sizeof(samplingBoxDimension));
            file.WriteConvexPolygonsField(preString + "staticObstacles", staticObstacles);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

