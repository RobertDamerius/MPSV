#pragma once


#include <mpsv_core_MPSVCommon.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief This class represents an axis-aligned bounding box.
 */
class AABB {
    public:
        double lowerBound[2];   // The lower bound of the axis-aligned bounding box [x, y].
        double upperBound[2];   // The upper bound of the axis-aligned bounding box [x, y].

        /**
         * @brief Create a 2D axis-aligned bounding box.
         */
        AABB() noexcept : AABB(0.0,0.0,0.0,0.0){}

        /**
         * @brief Create a 2D axis-aligned bounding box.
         * @param[in] xLower Lower value for x dimension.
         * @param[in] yLower Lower value for y dimension.
         * @param[in] xUpper Upper value for x dimension.
         * @param[in] yUpper Upper value for y dimension.
         */
        AABB(double xLower, double yLower, double xUpper, double yUpper) noexcept {
            lowerBound[0] = std::min(xLower,xUpper);
            upperBound[0] = std::max(xLower,xUpper);
            lowerBound[1] = std::min(yLower,yUpper);
            upperBound[1] = std::max(yLower,yUpper);
        }

        /**
         * @brief Reset the AABB to a point.
         * @param[in] point x and y coordinate of a 2D point.
         */
        void Reset(std::array<double,2> point) noexcept {
            lowerBound[0] = upperBound[0] = point[0];
            lowerBound[1] = upperBound[1] = point[1];
        }

        /**
         * @brief Extend the whole AABB by an absolute value. The AABB is extended by this value in positive
         * and negative direction. Thus, the total dimension of the AABB grows by 2*e in each dimension.
         * @param[in] e The AABB extension value. MAKE SURE THAT THIS VALUE IS POSITIVE OR AT LEAST ZERO!
         */
        void Extend(double e) noexcept {
            lowerBound[0] -= e;
            lowerBound[1] -= e;
            upperBound[0] += e;
            upperBound[1] += e;
        }

        /**
         * @brief Check if two AABBs overlap.
         * @param[in] aabb Another axis-aligned bounding box for which to check intersection.
         * @return True if the AABB does overlap with this one, false otherwise.
         */
        bool Overlap(const AABB& aabb) const noexcept { return !((aabb.lowerBound[0] > upperBound[0]) || (lowerBound[0] > aabb.upperBound[0]) || (aabb.lowerBound[1] > upperBound[1]) || (lowerBound[1] > aabb.upperBound[1])); };

        /**
         * @brief Swap this AABB with a given AABB.
         * @param[inout] aabb The AABB to be used for swapping.
         */
        void Swap(AABB& aabb) noexcept {
            std::swap(lowerBound[0], aabb.lowerBound[0]);
            std::swap(lowerBound[1], aabb.lowerBound[1]);
            std::swap(upperBound[0], aabb.upperBound[0]);
            std::swap(upperBound[1], aabb.upperBound[1]);
        }

        /**
         * @brief Operator+= adds the right-hand sided AABB to this AABB (union).
         */
        AABB& operator+=(const AABB& rhs) noexcept {
            lowerBound[0] = std::min(lowerBound[0], rhs.lowerBound[0]);
            lowerBound[1] = std::min(lowerBound[1], rhs.lowerBound[1]);
            upperBound[0] = std::max(upperBound[0], rhs.upperBound[0]);
            upperBound[1] = std::max(upperBound[1], rhs.upperBound[1]);
            return *this;
        }

        /**
         * @brief Operator+= adds the right-hand sided point (x,y) to this AABB (union).
         */
        AABB& operator+=(const std::array<double,2>& rhs) noexcept {
            lowerBound[0] = std::min(lowerBound[0], rhs[0]);
            lowerBound[1] = std::min(lowerBound[1], rhs[1]);
            upperBound[0] = std::max(upperBound[0], rhs[0]);
            upperBound[1] = std::max(upperBound[1], rhs[1]);
            return *this;
        }

        /**
         * @brief Operator+ for adding two AABBs.
         */
        friend AABB operator+(AABB lhs, const AABB& rhs) noexcept { lhs += rhs; return lhs; }
};


} /* namespace: geometry */


} /* namespace: mpsv */

