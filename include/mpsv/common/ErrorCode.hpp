#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


/**
 * @brief Scoped enumeration for MPSV error codes.
 */
enum class error_code : std::uint8_t {
    NONE                                                  = 0U,    // No error.

    // planner inputs
    INPUT_TIMESTAMP                                       = 1U,    // timestamp is not finite.
    INPUT_ORIGIN_LLA                                      = 2U,    // originLLA is not finite.
    INPUT_ORIGIN_OLD_TO_NEW                               = 3U,    // originOldToNew is not finite.
    INPUT_INITIAL_POSE                                    = 4U,    // initialPose is not finite.
    INPUT_INITIAL_STATE_AND_INPUT                         = 5U,    // initialStateAndInput is not finite.
    INPUT_INITIAL_PATH                                    = 6U,    // initialPath is not finite.
    INPUT_FINAL_POSE                                      = 7U,    // finalPose is not finite.
    INPUT_SAMPLING_BOX_CENTER_POSE                        = 8U,    // samplingBoxCenterPose is not finite.
    INPUT_SAMPLING_BOX_DIMENSION                          = 9U,    // samplingBoxDimension is not finite.
    INPUT_INITIAL_POSE_NOT_INSIDE_SAMPLING_BOX            = 10U,   // initalPose is not inside the sampling box.
    INPUT_INITIAL_STATE_NOT_INSIDE_SAMPLING_BOX           = 11U,   // initialStateAndInput is not inside the sampling box.
    INPUT_FINAL_POSE_NOT_INSIDE_SAMPLING_BOX              = 12U,   // finalPose is not inside the sampling box.
    INPUT_STATIC_OBSTACLES                                = 13U,   // Vertices of staticObstacles are not finite, not convex or vertex order is inconsistent.
    INPUT_MIN_TRAJECTORY_DURATION                         = 14U,   // minTrajectoryDuration is not finite or is negative.

    // parameters
    PARAM_MODEL_MATF                                      = 15U,   // matF is not finite.
    PARAM_MODEL_MATB                                      = 16U,   // matB is not finite.
    PARAM_MODEL_INV_MATB                                  = 17U,   // matB is not invertable.
    PARAM_MODEL_VEC_TIMECONSTANTS_XYN                     = 18U,   // vecTimeconstantsXYN is not finite or not positive.
    PARAM_MODEL_VEC_TIMECONSTANTS_INPUT                   = 19U,   // vecTimeconstantsInput is not finite or not positive.
    PARAM_MODEL_LOWER_LIMIT_XYN                           = 20U,   // lowerLimitXYN is not finite.
    PARAM_MODEL_UPPER_LIMIT_XYN                           = 21U,   // upperLimitXYN is not finite.
    PARAM_MODEL_LOWER_GREATER_UPPER_LIMIT                 = 22U,   // lowerLimitXYN is greater than upperLimitXYN.
    PARAM_COSTMAP_RESOLUTION                              = 23U,   // resolution is not finite or is too low.
    PARAM_COSTMAP_DISTANCE_SCALE                          = 24U,   // distanceScale is not finite or negative.
    PARAM_COSTMAP_DISTANCE_DECAY                          = 25U,   // distanceDecay is not finite or not positive.
    PARAM_METRIC_WEIGHT_PSI                               = 26U,   // weightPsi is not finite or not positive.
    PARAM_METRIC_WEIGHT_SWAY                              = 27U,   // weightSway is not finite or negative.
    PARAM_METRIC_WEIGHT_REVERSE_SCALE                     = 28U,   // weightReverseScale is not finite or negative.
    PARAM_METRIC_WEIGHT_REVERSE_DECAY                     = 29U,   // weightReverseDecay is not finite or not positive.
    PARAM_GEOMETRY_MAX_POSITION_DEVIATION                 = 30U,   // collisionCheckMaxPositionDeviation is not finite or too low.
    PARAM_GEOMETRY_MAX_ANGLE_DEVIATION                    = 31U,   // collisionCheckMaxAngleDeviation is not finite or too low.
    PARAM_GEOMETRY_VEHICLE_SHAPE                          = 32U,   // Vertices of vehicleShape are not finite, not convex or vertex order is inconsistent.
    PARAM_GEOMETRY_SKELETAL_POINTS                        = 33U,   // skeletalPoints is empty or not finite.
    PARAM_PATHPLANNER_PERIOD_GOAL_SAMPLING                = 34U,   // periodGoalSampling for path planner is not positive.
    PARAM_MOTIONPLANNER_SAMPLING_RANGE_POSITION           = 35U,   // samplingRangePosition is not finite or not positive.
    PARAM_MOTIONPLANNER_SAMPLING_RANGE_ANGLE              = 36U,   // samplingRangeAngle is not finite or not positive.
    PARAM_MOTIONPLANNER_PERIOD_GOAL_SAMPLING              = 37U,   // periodGoalSampling for motion planner is not positive.
    PARAM_MOTIONPLANNER_SAMPLETIME                        = 38U,   // sampletime is not finite or not positive.
    PARAM_MOTIONPLANNER_MAX_INPUT_PATH_LENGTH             = 39U,   // maxInputPathLength is not finite or not positive.
    PARAM_CONTROLLER_MAT_K                                = 40U,   // matK is not finite.
    PARAM_CONTROLLER_MAX_RADIUS_X                         = 41U,   // maxRadiusX is not finite or not positive.
    PARAM_CONTROLLER_MAX_RADIUS_Y                         = 42U,   // maxRadiusY is not finite or not positive.
    PARAM_CONTROLLER_MAX_RADIUS_PSI                       = 43U,   // maxRadiusPsi is not finite or not positive.
    PARAM_CONTROLLER_MIN_RADIUS_POSITION                  = 44U,   // minRadiusPosition is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_POSE                 = 45U,   // rangePose is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_UVR                  = 46U,   // rangeUVR is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_XYN                  = 47U,   // rangeXYN is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH_RESET   = 48U,   // maxComputationTimePathOnReset is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION_RESET = 49U,   // maxComputationTimeMotionOnReset is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH         = 50U,   // maxComputationTimePath is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION       = 51U,   // maxComputationTimeMotion is not finite or not positive.
    PARAM_ONLINEPLANNER_ADDITIONAL_AHEAD_PLANNING_TIME    = 52U,   // additionalAheadPlanningTime is not finite or not positive.
    PARAM_ONLINEPLANNER_ADDITIONAL_TRAJECTORY_DURATION    = 53U,   // additionalTrajectoryDuration is not finite or not positive.
    PARAM_ONLINEPLANNER_TIME_KEEP_PAST_TRAJECTORY         = 54U,   // timeKeepPastTrajectory is not finite or negative.
    PARAM_ONLINEPLANNER_NUM_SIMULATION_STEPS              = 55U,   // The number of simulation steps is too high: either planning time is too high and/or sampletime is too low.
    PARAM_ASYNCPLANNER_TIMESTAMP                          = 56U,   // timestamp is not finite.
    PARAM_ASYNCPLANNER_TIMEOUT_INPUT                      = 57U,   // timeoutInput is not finite.

    // path planner
    PATHPLANNER_INITIAL_POSE_COLLISION                    = 58U,   // The initial pose of the path planner collides with static obstacles.
    PATHPLANNER_OUT_OF_NODES                              = 59U,   // The path planner has run out of nodes and cannot sample new nodes anymore.

    // motion planner
    MOTIONPLANNER_INITIAL_POSE_COLLISION                  = 60U,   // The initial pose of the motion planner collides with static obstacles.
    MOTIONPLANNER_OUT_OF_NODES                            = 61U,   // The motion planner has run out of nodes and cannot sample new nodes anymore.
    MOTIONPLANNER_INFEASIBLE_TRAJECTORY                   = 62U,   // The final motion planner trajectory is dynamically infeasible.

    // sequential planner
    SEQUENTIALPLANNER_INCOMPLETE_MOTION                   = 63U,   // The motion planner has not been executed either due to a requested interrupt or because the path planner failed. The motion planner output is empty.

    // online planner
    ONLINEPLANNER_TIMESTAMP_NOT_INCREASING                = 64U,   // timestamp is not greater than the previous input timestamp for solving sequential problems.

    // asynchronous online planner
    ASYNCONLINEPLANNER_OUTPUT_NOT_AVAILABLE               = 65U    // The output does not contain a planning result yet.
};


} /* namespace: mpsv */
