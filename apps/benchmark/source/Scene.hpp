#pragma once


#include <mpsv/mpsv.hpp>


class Scene {
    public:
        std::array<double,12> initialStateAndInput;                    ///< Vector containing the initial state {x,y,psi,u,v,r,X,Y,N} and the initial input {Xc,Yc,Nc}.
        std::array<double,3> finalPose;                                ///< Final pose given as {x,y,psi}.
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   ///< List of static obstacles.

        /**
         * @brief Construct a new scene.
         */
        Scene() noexcept {
            initialStateAndInput.fill(0.0);
            finalPose.fill(0.0);
        }

        /**
         * @brief Check whether all attributes are valid.
         * @return True if all attributes are valid, false otherwise.
         */
        bool IsValid(void) noexcept {
            bool valid = std::isfinite(initialStateAndInput[0]) && std::isfinite(initialStateAndInput[1]) && std::isfinite(initialStateAndInput[2]) && std::isfinite(initialStateAndInput[3]) && std::isfinite(initialStateAndInput[4]) && std::isfinite(initialStateAndInput[5]) && std::isfinite(initialStateAndInput[6]) && std::isfinite(initialStateAndInput[7]) && std::isfinite(initialStateAndInput[8]) && std::isfinite(initialStateAndInput[9]) && std::isfinite(initialStateAndInput[10]) && std::isfinite(initialStateAndInput[11]);
            valid &= std::isfinite(finalPose[0]) && std::isfinite(finalPose[1]) && std::isfinite(finalPose[2]);
            for(auto&& obstacle : staticObstacles){
                valid &= obstacle.IsConvex() && obstacle.IsFinite();
            }
            return valid;
        }
};

