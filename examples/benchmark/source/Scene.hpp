#pragma once


#include <mpsv/mpsv.hpp>


class Scene {
    public:
        std::array<double, 12U> initialStateAndInput;                    // Vector containing the initial state {x,y,psi,u,v,r,X,Y,N} and the initial input {Xc,Yc,Nc}.
        std::array<double, 3U> finalPose;                                // Final pose given as {x,y,psi}.
        std::vector<mpsv::geometry::StaticObstacle> staticObstacles;   // List of static obstacles.

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
        bool IsValid() noexcept {
            bool valid{std::isfinite(initialStateAndInput[0U]) && std::isfinite(initialStateAndInput[1U]) && std::isfinite(initialStateAndInput[2U]) && std::isfinite(initialStateAndInput[3U]) && std::isfinite(initialStateAndInput[4U]) && std::isfinite(initialStateAndInput[5U]) && std::isfinite(initialStateAndInput[6U]) && std::isfinite(initialStateAndInput[7U]) && std::isfinite(initialStateAndInput[8U]) && std::isfinite(initialStateAndInput[9U]) && std::isfinite(initialStateAndInput[10U]) && std::isfinite(initialStateAndInput[11U])};
            valid &= std::isfinite(finalPose[0U]) && std::isfinite(finalPose[1U]) && std::isfinite(finalPose[2U]);
            for (auto&& obstacle : staticObstacles) {
                valid &= obstacle.IsValid();
            }
            return valid;
        }
};
