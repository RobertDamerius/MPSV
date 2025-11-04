#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief This class represents the input to the motion planning algorithm.
 */
class MotionPlannerInput {
    public:
        std::array<double,12> initialStateAndInput;                    // Initial state and input given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}.
        std::vector<std::array<double,3>> initialPath;                 // The initial path starting from the initial state. The final pose is indicated by the last element. Each element is given as {x,y,psi}.
        std::array<double,2> originOldToNew;                           // Translation between two consecutive problems. If the origin of the previous problem is different to the origin of the new problem, then the old solution must be transformed to be used as initial sample set in the new problem. This vector specifies the position of the new origin with respect to the old origin (vector from old origin to new origin).
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   // List of static obstacles, where each vertex is given as {x,y}.

        /**
         * @brief Construct a new motion planner input object and set default values.
         */
        MotionPlannerInput() noexcept {
            initialStateAndInput.fill(0.0);
            originOldToNew.fill(0.0);
        }

        /**
         * @brief Check whether the attributes are valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         * @details The vertex order of polygon data may be adjusted if possible.
         */
        error_code IsValid(void) noexcept {
            if(!(std::isfinite(initialStateAndInput[0]) &&
                 std::isfinite(initialStateAndInput[1]) &&
                 std::isfinite(initialStateAndInput[2]) &&
                 std::isfinite(initialStateAndInput[3]) &&
                 std::isfinite(initialStateAndInput[4]) &&
                 std::isfinite(initialStateAndInput[5]) &&
                 std::isfinite(initialStateAndInput[6]) &&
                 std::isfinite(initialStateAndInput[7]) &&
                 std::isfinite(initialStateAndInput[8]) &&
                 std::isfinite(initialStateAndInput[9]) &&
                 std::isfinite(initialStateAndInput[10]) &&
                 std::isfinite(initialStateAndInput[11])))
                return error_code::INPUT_INITIAL_STATE_AND_INPUT;
            if(!(std::isfinite(originOldToNew[0]) &&
                 std::isfinite(originOldToNew[1])))
                return error_code::INPUT_ORIGIN_OLD_TO_NEW;
            for(auto&& pose : initialPath){
                if(!(std::isfinite(pose[0]) &&
                     std::isfinite(pose[1]) &&
                     std::isfinite(pose[2])))
                    return error_code::INPUT_INITIAL_PATH;
            }
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
            file.WriteField("double", preString + "initialStateAndInput", {12}, &initialStateAndInput[0], sizeof(initialStateAndInput));
            file.WriteVectorField(preString + "initialPath", initialPath);
            file.WriteField("double", preString + "originOldToNew", {2}, &originOldToNew[0], sizeof(originOldToNew));
            file.WriteConvexPolygonsField(preString + "staticObstacles", staticObstacles);
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

