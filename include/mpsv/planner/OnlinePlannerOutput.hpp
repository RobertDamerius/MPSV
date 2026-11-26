#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv::planner {


/**
 * @brief This class represents the output of the online planning algorithm (online sequential planner).
 */
class OnlinePlannerOutput {
    public:
        double timestamp{};                                        // Monotonically increasing timestamp in seconds (arbitrary time origin defined by the user) indicating the initial timepoint of the @ref trajectory.
        std::array<double, 3U> originLLA{};                        // Geographical origin to which this outputs belongs to, given as {lat,lon,alt}.
        std::vector<std::array<double, 12U>> trajectory{};         // Resulting trajectory, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
        double sampletime{};                                       // The sampletime of the trajectory data in seconds.
        bool performedReset{};                                     // True if reset has been performed, false otherwise.
        mpsv::error_code errorCode{mpsv::error_code::NONE};        // Error code of the sequential planning operation.
        struct {
            std::vector<std::array<double, 3U>> path{};            // [PathPlanner] Resulting path of the internal path planning problem, where each pose is given as {x,y,psi}.
            bool goalReached{};                                    // [PathPlanner] True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            std::uint32_t numberOfPerformedIterations{};           // [PathPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC{};                    // [PathPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost{};                                         // [PathPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } pathPlanner{};
        struct {
            std::vector<std::array<double, 3U>> referencePath{};   // [MotionPlanner] Resulting reference path of the internal motion planning problem, where each pose is given as {x,y,psi}.
            std::vector<std::array<double, 12U>> trajectory{};     // [MotionPlanner] Resulting trajectory of the internal motion planning problem, where each element is given as {x,y,psi,u,v,r,X,Y,N,Xc,Yc,Nc}. The initial state and input is not inserted.
            double startingTimepoint{};                            // [MotionPlanner] The starting timepoint of the internal motion planner trajectory (arbitrary time origin defined by the user, same as @ref timestamp).
            bool goalReached{};                                    // [MotionPlanner] True if goal is reached, false otherwise. The goal is reached, if the final pose of the path is equal to the desired final pose of the path planning problem.
            std::uint32_t numberOfPerformedIterations{};           // [MotionPlanner] The total number of iterations that have been performed since the latest prepare step.
            double timestampOfComputationUTC{};                    // [MotionPlanner] Timestamp that indicates the time (seconds of the day, UTC) when the solution was computed.
            double cost{};                                         // [MotionPlanner] The cost of the current solution, given as the total cost from the initial node to the final node of the path along the path.
        } motionPlanner{};

        /**
         * @brief Clear the output and set default values.
         */
        void Clear() noexcept {
            timestamp = 0.0;
            originLLA.fill(0.0);
            trajectory.clear();
            sampletime = 0.0;
            performedReset = false;
            errorCode = mpsv::error_code::NONE;
            pathPlanner.path.clear();
            pathPlanner.goalReached = false;
            pathPlanner.numberOfPerformedIterations = 0U;
            pathPlanner.timestampOfComputationUTC = pathPlanner.cost = 0.0;
            motionPlanner.referencePath.clear();
            motionPlanner.trajectory.clear();
            motionPlanner.goalReached = false;
            motionPlanner.numberOfPerformedIterations = 0U;
            motionPlanner.startingTimepoint = motionPlanner.timestampOfComputationUTC = motionPlanner.cost = 0.0;
        }

        /**
         * @brief Move the data of this object to another object.
         * @param[in] dst The destination object to which to move the data of this object.
         */
        void MoveTo(mpsv::planner::OnlinePlannerOutput& dst) noexcept {
            dst.timestamp = timestamp;
            dst.originLLA = originLLA;
            dst.trajectory.swap(trajectory);
            dst.sampletime = sampletime;
            dst.performedReset = performedReset;
            dst.errorCode = errorCode;
            dst.pathPlanner.path.swap(pathPlanner.path);
            dst.pathPlanner.goalReached = pathPlanner.goalReached;
            dst.pathPlanner.numberOfPerformedIterations = pathPlanner.numberOfPerformedIterations;
            dst.pathPlanner.timestampOfComputationUTC = pathPlanner.timestampOfComputationUTC;
            dst.pathPlanner.cost = pathPlanner.cost;
            dst.motionPlanner.referencePath.swap(motionPlanner.referencePath);
            dst.motionPlanner.trajectory.swap(motionPlanner.trajectory);
            dst.motionPlanner.startingTimepoint = motionPlanner.startingTimepoint;
            dst.motionPlanner.goalReached = motionPlanner.goalReached;
            dst.motionPlanner.numberOfPerformedIterations = motionPlanner.numberOfPerformedIterations;
            dst.motionPlanner.timestampOfComputationUTC = motionPlanner.timestampOfComputationUTC;
            dst.motionPlanner.cost = motionPlanner.cost;
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "timestamp", {1U}, &timestamp, sizeof(timestamp));
            file.WriteField("double", preString + "originLLA", {3U}, &originLLA[0U], sizeof(originLLA));
            file.WriteVectorField(preString + "trajectory", trajectory);
            file.WriteField("double", preString + "sampletime", {1U}, &sampletime, sizeof(sampletime));
            file.WriteField("bool", preString + "performedReset", {1U}, &performedReset, sizeof(performedReset));
            file.WriteField("uint8_t", preString + "errorCode", {1U}, &errorCode, sizeof(errorCode));
            file.WriteVectorField(preString + "pathPlanner.path", pathPlanner.path);
            file.WriteField("bool", preString + "pathPlanner.goalReached", {1U}, &pathPlanner.goalReached, sizeof(pathPlanner.goalReached));
            file.WriteField("uint32_t", preString + "pathPlanner.numberOfPerformedIterations", {1U}, &pathPlanner.numberOfPerformedIterations, sizeof(pathPlanner.numberOfPerformedIterations));
            file.WriteField("double", preString + "pathPlanner.timestampOfComputationUTC", {1U}, &pathPlanner.timestampOfComputationUTC, sizeof(pathPlanner.timestampOfComputationUTC));
            file.WriteField("double", preString + "pathPlanner.cost", {1U}, &pathPlanner.cost, sizeof(pathPlanner.cost));
            file.WriteVectorField(preString + "motionPlanner.referencePath", motionPlanner.referencePath);
            file.WriteVectorField(preString + "motionPlanner.trajectory", motionPlanner.trajectory);
            file.WriteField("double", preString + "motionPlanner.startingTimepoint", {1U}, &motionPlanner.startingTimepoint, sizeof(motionPlanner.startingTimepoint));
            file.WriteField("bool", preString + "motionPlanner.goalReached", {1U}, &motionPlanner.goalReached, sizeof(motionPlanner.goalReached));
            file.WriteField("uint32_t", preString + "motionPlanner.numberOfPerformedIterations", {1U}, &motionPlanner.numberOfPerformedIterations, sizeof(motionPlanner.numberOfPerformedIterations));
            file.WriteField("double", preString + "motionPlanner.timestampOfComputationUTC", {1U}, &motionPlanner.timestampOfComputationUTC, sizeof(motionPlanner.timestampOfComputationUTC));
            file.WriteField("double", preString + "motionPlanner.cost", {1U}, &motionPlanner.cost, sizeof(motionPlanner.cost));
        }
};


} /* namespace: mpsv::planner */
