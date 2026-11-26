#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv::geometry {


/**
 * @brief This class represents an axis-aligned bounding box.
 */
class AABB {
    public:
        /**
         * @brief Create a 2D axis-aligned bounding box.
         */
        AABB() noexcept : AABB(0.0, 0.0, 0.0, 0.0) {}

        /**
         * @brief Create a 2D axis-aligned bounding box.
         * @param[in] xLower Lower value for x dimension.
         * @param[in] yLower Lower value for y dimension.
         * @param[in] xUpper Upper value for x dimension.
         * @param[in] yUpper Upper value for y dimension.
         */
        AABB(double xLower, double yLower, double xUpper, double yUpper) noexcept {
            m_lowerBound[0U] = std::min(xLower, xUpper);
            m_upperBound[0U] = std::max(xLower, xUpper);
            m_lowerBound[1U] = std::min(yLower, yUpper);
            m_upperBound[1U] = std::max(yLower, yUpper);
        }

        /**
         * @brief Reset the AABB to a point.
         * @param[in] point x and y coordinate of a 2D point.
         */
        void Reset(std::array<double, 2U> point) noexcept {
            m_lowerBound[0U] = m_upperBound[0U] = point[0U];
            m_lowerBound[1U] = m_upperBound[1U] = point[1U];
        }

        /**
         * @brief Extend the whole AABB by an absolute value. The AABB is extended by this value in positive
         * and negative direction. Thus, the total dimension of the AABB grows by 2*e in each dimension.
         * @param[in] e The absolute extension value.
         */
        void Extend(double e) noexcept {
            e = std::fabs(e);
            m_lowerBound[0U] -= e;
            m_lowerBound[1U] -= e;
            m_upperBound[0U] += e;
            m_upperBound[1U] += e;
        }

        /**
         * @brief Check if two AABBs overlap.
         * @param[in] aabb Another axis-aligned bounding box for which to check intersection.
         * @return True if the AABB does overlap with this one, false otherwise.
         */
        [[nodiscard]] bool Overlap(const AABB& aabb) const noexcept {
            return !((aabb.m_lowerBound[0U] > m_upperBound[0U]) || (m_lowerBound[0U] > aabb.m_upperBound[0U]) || (aabb.m_lowerBound[1U] > m_upperBound[1U]) || (m_lowerBound[1U] > aabb.m_upperBound[1U]));
        };

        /**
         * @brief Swap this AABB with a given AABB.
         * @param[inout] aabb The AABB to be used for swapping.
         */
        void Swap(AABB& aabb) noexcept {
            std::swap(m_lowerBound, aabb.m_lowerBound);
            std::swap(m_upperBound, aabb.m_upperBound);
        }

        /**
         * @brief Check if the AABB is finite.
         * @return True if the AABB is finite, false otherwise.
         */
        [[nodiscard]] bool IsFinite() const noexcept {
            return std::isfinite(m_lowerBound[0U]) && std::isfinite(m_lowerBound[1U]) && std::isfinite(m_upperBound[0U]) && std::isfinite(m_upperBound[1U]);
        }

        /**
         * @brief Operator+= adds the right-hand sided AABB to this AABB (union).
         */
        AABB& operator+=(const AABB& rhs) noexcept {
            m_lowerBound[0U] = std::min(m_lowerBound[0U], rhs.m_lowerBound[0U]);
            m_lowerBound[1U] = std::min(m_lowerBound[1U], rhs.m_lowerBound[1U]);
            m_upperBound[0U] = std::max(m_upperBound[0U], rhs.m_upperBound[0U]);
            m_upperBound[1U] = std::max(m_upperBound[1U], rhs.m_upperBound[1U]);
            return *this;
        }

        /**
         * @brief Operator+= adds the right-hand sided point (x,y) to this AABB (union).
         */
        AABB& operator+=(const std::array<double, 2U>& rhs) noexcept {
            m_lowerBound[0U] = std::min(m_lowerBound[0U], rhs[0U]);
            m_lowerBound[1U] = std::min(m_lowerBound[1U], rhs[1U]);
            m_upperBound[0U] = std::max(m_upperBound[0U], rhs[0U]);
            m_upperBound[1U] = std::max(m_upperBound[1U], rhs[1U]);
            return *this;
        }

        /**
         * @brief Operator+ for adding two AABBs.
         */
        friend AABB operator+(AABB lhs, const AABB& rhs) noexcept {
            lhs += rhs;
            return lhs;
        }

    private:
        std::array<double, 2U> m_lowerBound;   // The lower bound of the axis-aligned bounding box [x, y].
        std::array<double, 2U> m_upperBound;   // The upper bound of the axis-aligned bounding box [x, y].
};


} /* namespace: mpsv::geometry */
