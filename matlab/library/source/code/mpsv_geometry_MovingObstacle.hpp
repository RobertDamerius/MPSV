#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_geometry_ConvexPolygon.hpp>
#include <mpsv_geometry_AABB.hpp>
#include <mpsv_geometry_DouglasPeucker.hpp>
#include <mpsv_core_Time.hpp>
#include <mpsv_math_Additional.hpp>


namespace mpsv {


namespace geometry {


/**
 * @brief This class represents a time interval obstacle. It's a set of convex polygons for a specific time interval.
 */
class TimeIntervalObstacle {
    public:
        double tStart;                                               // Starting timepoint of the time interval.
        double tEnd;                                                 // Final timepoint of the time interval.
        std::vector<mpsv::geometry::ConvexPolygon> convexPolygons;   // Set of convex polygons representing this obstacle.
        mpsv::geometry::AABB aabb;                                   // 2D axis-aligned bounding box of this obstacle.
};


/**
 * @brief This class represents a moving obstacle that is based on a convex polygon.
 */
class MovingObstacle : protected mpsv::geometry::ConvexPolygon {
    public:
        std::vector<mpsv::geometry::TimeIntervalObstacle> timeIntervalObstacles;   // List of time interval obstacles.

        /**
         * @brief Default constructor for a moving obstacle.
         */
        MovingObstacle() noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(), timestampUTC(0.0), sampletime(1.0) {}

        /**
         * @brief Construct a moving obstacle based on given vertices.
         * @param[in] vertices Container of vertices.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        explicit MovingObstacle(std::vector<std::array<double, 2>> vertices) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices), timestampUTC(0.0), sampletime(1.0) {}

        /**
         * @brief Construct a moving obstacle based on given vertices and a given trajectory.
         * @param[in] vertices Container of vertices.
         * @param[in] timestampUTC UTC timestamp (seconds of the day) indicating the starting timepoint of the trajectory.
         * @param[in] sampletime Sampletime in seconds indicating the time interval between two data points of the trajectory.
         * @param[in] trajectory Trajectory data given as a set of poses. Each element is given as {x,y,psi}.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         */
        MovingObstacle(std::vector<std::array<double, 2>> vertices, double timestampUTC, double sampletime, std::vector<std::array<double,3>> trajectory) noexcept : mpsv::geometry::ConvexPolygon::ConvexPolygon(vertices), timestampUTC(timestampUTC), sampletime(sampletime) { this->trajectory.swap(trajectory); }

        /**
         * @brief Create the moving obstacle convex polygon based on vertices and trajectory data.
         * @param[in] vertices Container of vertices.
         * @param[in] timestampUTC UTC timestamp (seconds of the day) indicating the starting timepoint of the trajectory.
         * @param[in] sampletime Sampletime in seconds indicating the time interval between two data points of the trajectory.
         * @param[in] trajectory Trajectory data given as a set of poses. Each element is given as {x,y,psi}.
         * @note IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT! Call @ref EnsureCorrectVertexOrder to ensure the correct vertex order and check for convexity.
         * This member function only swap containers and rebuild edges.
         */
        void Create(std::vector<std::array<double, 2>>& vertices, double timestampUTC, double sampletime, std::vector<std::array<double,3>> trajectory) noexcept {
            mpsv::geometry::ConvexPolygon::Create(vertices);
            this->timestampUTC = timestampUTC;
            this->sampletime = sampletime;
            this->trajectory.swap(trajectory);
        }

        /**
         * @brief Build the @ref timeIntervalObstacles container.
         * @param[in] originTimestampUTC Absolute starting timepoint for the time interval obstacles given as UTC timestamp (seconds of the day).
         * @param[in] timeInterval The time interval in seconds.
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. This value must be positive.
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. This value must be positive.
         */
        void BuildTimeIntervalObstacles(double originTimestampUTC, double timeInterval, double thresholdPosition, double thresholdAngle) noexcept {
            timeIntervalObstacles.clear();
            if(trajectory.empty()){
                return;
            }

            // Start and end time of trajectory with respect to the origin
            double tStart = mpsv::core::DifferenceTimestampUTC(timestampUTC, originTimestampUTC);
            double tEnd = tStart + sampletime * static_cast<double>(trajectory.size() - 1);

            // Convert to time window indices and clamp to origin
            int32_t iStart = static_cast<int32_t>(std::floor(tStart / timeInterval));
            int32_t iEnd = static_cast<int32_t>(std::floor(tEnd / timeInterval)) + 1;
            if(iEnd < 0){
                return;
            }
            iStart = std::max(0, iStart);

            // For all time windows, generate time interval obstacles
            std::vector<std::array<double,3>> poses;
            std::vector<mpsv::geometry::ConvexPolygon> convexPolygons;
            for(int32_t i = iStart; i != iEnd; ++i){
                // Get start and endpoint of the current interval
                double startTimeOfInterval = static_cast<double>(i) * timeInterval;
                double endTimeOfInterval = startTimeOfInterval + timeInterval;

                // Transform start and end timepoints to the origin of the trajectory
                double startTimeTrajectory = startTimeOfInterval - tStart;
                double endTimeTrajectory = endTimeOfInterval - tStart;

                // Find all poses between startTimeTrajectory and endTimeTrajectory (start and end pose might be interpolated)
                GetAllPosesForTimeInterval(poses, startTimeTrajectory, endTimeTrajectory);

                // Generate set of convex polygons
                GenerateConvexPolygonsAlongLine(convexPolygons, poses, thresholdPosition, thresholdAngle);

                // Add new time interval obstacle
                if(!convexPolygons.empty()){
                    mpsv::geometry::AABB aabb = convexPolygons[0].GetAABB();
                    for(size_t p = 0; p < convexPolygons.size(); ++p){
                        aabb += convexPolygons[p].GetAABB();
                    }
                    timeIntervalObstacles.push_back(mpsv::geometry::TimeIntervalObstacle());
                    timeIntervalObstacles.back().tStart = startTimeOfInterval;
                    timeIntervalObstacles.back().tEnd = endTimeOfInterval;
                    timeIntervalObstacles.back().convexPolygons.swap(convexPolygons);
                    timeIntervalObstacles.back().aabb = aabb;
                }
            }
        }

        /* Make some member functions public again */
        using mpsv::geometry::ConvexPolygon::EnsureCorrectVertexOrder;
        using mpsv::geometry::ConvexPolygon::IsConvex;

    protected:
        double timestampUTC;                            // UTC timestamp (seconds of the day) of the @ref trajectory data.
        double sampletime;                              // Sampletime of the @ref trajectory data in seconds.
        std::vector<std::array<double,3>> trajectory;   // Trajectory data, where each element is indicated as a pose, given as {x,y,psi}.

        /**
         * @brief Get all poses for a specific time interval.
         * @param[out] poses Resulting list of poses. The first and last pose may be interpolated.
         * @param[in] startTimeTrajectory Starting timepoint of the interval relative to the start of the trajectory.
         * @param[in] endTimeTrajectory Final timepoint of the interval relative to the start of the trajectory.
         */
        void GetAllPosesForTimeInterval(std::vector<std::array<double,3>>& poses, double startTimeTrajectory, double endTimeTrajectory) noexcept {
            poses.clear();

            // Convert timepoints to trajectory indices
            int32_t maxNumPoses = static_cast<int32_t>(trajectory.size());
            double fStart = startTimeTrajectory / sampletime;           // starting index with fractional part
            double fEnd = endTimeTrajectory / sampletime;               // final index with fractional part
            int32_t iStart = static_cast<int32_t>(std::floor(fStart));  // starting index floored to lower integer
            int32_t iEnd = static_cast<int32_t>(std::floor(fEnd)) + 1;  // final index ceiled to upper integer
            if((fEnd < 0.0) || (fStart >= static_cast<double>(maxNumPoses - 1))){
                return;
            }

            // Get all poses from start to end (including start and excluding end)
            int32_t i0 = std::max(0, iStart);
            int32_t iN = std::min(iEnd, maxNumPoses);
            for(int32_t i = i0; i < iN; ++i){
                poses.push_back(trajectory[i]);
            }

            // Interpolate first pose
            if((i0 == iStart) && ((i0 + 1) < maxNumPoses)){
                std::array<double,3> dp = mpsv::math::PoseDifference(trajectory[i0 + 1], trajectory[i0]);
                poses[0][0] = trajectory[i0][0] + (fStart - static_cast<double>(i0)) * dp[0];
                poses[0][1] = trajectory[i0][1] + (fStart - static_cast<double>(i0)) * dp[1];
                poses[0][2] = mpsv::math::SymmetricalAngle(trajectory[i0][2] + (fStart - static_cast<double>(i0)) * dp[2]);
            }

            // Insert interpolated last pose
            if((iN == iEnd) && ((iN - 1) >= 0) && (iN < maxNumPoses)){
                std::array<double,3> dp = mpsv::math::PoseDifference(trajectory[iN], trajectory[iN - 1]);
                double s = (fEnd - static_cast<double>(iN - 1));
                dp[0] = s*dp[0] + trajectory[iN - 1][0];
                dp[1] = s*dp[1] + trajectory[iN - 1][1];
                dp[2] = mpsv::math::SymmetricalAngle(s*dp[2] + trajectory[iN - 1][2]);
                poses.push_back(dp);
            }
        }

        /**
         * @brief Generate a set of convex polygons along a given line.
         * @param[out] convexPolygons Container of generated convex polygons.
         * @param[in] poses The line for which to generate the set of convex polygons.
         * @param[in] thresholdPosition The maximum position error to be allowed for line simplification. This value must be positive.
         * @param[in] thresholdAngle The maximum angular error to be allowed for line simplification. This value must be positive.
         */
        void GenerateConvexPolygonsAlongLine(std::vector<mpsv::geometry::ConvexPolygon>& convexPolygons, const std::vector<std::array<double,3>>& poses, double thresholdPosition, double thresholdAngle) noexcept {
            convexPolygons.clear();

            // Simplify poses
            std::vector<size_t> indices = mpsv::geometry::DouglasPeuckerSE2(poses, thresholdPosition, thresholdAngle);
            if(indices.empty()){
                return;
            }

            // Go through all major line segments
            for(size_t j = 1; j < indices.size(); ++j){
                std::array<double,3> pose0 = poses[indices[j - 1]];
                std::array<double,3> pose1 = poses[indices[j]];

                // Get number of subdivided segments and adapt deltaPose to segment size
                std::array<double, 3> deltaPose = mpsv::math::PoseDifference(pose1, pose0);
                int32_t numSegments = std::max(static_cast<int32_t>(1), static_cast<int32_t>(std::ceil(std::fabs(deltaPose[2]) / thresholdAngle)));
                double d = 1.0 / static_cast<double>(numSegments);
                deltaPose[0] *= d;
                deltaPose[1] *= d;
                deltaPose[2] *= d;

                // Generate convex polygons for all subdivided segments
                double c0 = std::cos(pose0[2]);
                double s0 = std::sin(pose0[2]);
                double c1, s1;
                for(int32_t i = 0; i < numSegments; ++i){
                    // Start and end pose for this segment
                    pose1[0] = pose0[0] + deltaPose[0];
                    pose1[1] = pose0[1] + deltaPose[1];
                    pose1[2] = pose0[2] + deltaPose[2];
                    c1 = std::cos(pose1[2]);
                    s1 = std::sin(pose1[2]);

                    // Create convex hull for current subdivided segment and move to output container
                    mpsv::geometry::ConvexPolygon hullPolygon = CreateTransformedConvexHull(pose0[0], pose0[1], c0, s0, pose1[0], pose1[1], c1, s1);
                    convexPolygons.push_back(std::move(hullPolygon));

                    // Next segment
                    pose0[0] = pose1[0];
                    pose0[1] = pose1[1];
                    pose0[2] = pose1[2];
                    c0 = c1;
                    s0 = s1;
                }
            }
        }
};


} /* namespace: geometry */


} /* namespace: mpsv */

