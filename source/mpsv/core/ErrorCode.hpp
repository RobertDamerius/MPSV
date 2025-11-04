#pragma once


#include <mpsv/core/MPSVCommon.hpp>


namespace mpsv {


/**
 * @brief Scoped enumeration for MPSV error codes.
 */
enum class error_code : uint8_t {
    NONE                                            = 0,    // No error.
    NOT_AVAILABLE                                   = 1,    // Default error code if error is not available yet, but it should be marked as error, e.g. as initial error code. This value is also used to indicate a timeout.

    // planner inputs
    INPUT_TIMESTAMP                                 = 2,    // timestamp is not finite.
    INPUT_ORIGIN_LLA                                = 3,    // originLLA is not finite.
    INPUT_ORIGIN_OLD_TO_NEW                         = 4,    // originOldToNew is not finite.
    INPUT_INITIAL_POSE                              = 5,    // initialPose is not finite.
    INPUT_INITIAL_STATE_AND_INPUT                   = 6,    // initialStateAndInput is not finite.
    INPUT_INITIAL_PATH                              = 7,    // initialPath is not finite.
    INPUT_FINAL_POSE                                = 8,    // finalPose is not finite.
    INPUT_SAMPLING_BOX_CENTER_POSE                  = 9,    // samplingBoxCenterPose is not finite.
    INPUT_SAMPLING_BOX_DIMENSION                    = 10,   // samplingBoxDimension is not finite.
    INPUT_INITIAL_POSE_NOT_INSIDE_SAMPLING_BOX      = 11,   // initalPose is not inside the sampling box.
    INPUT_INITIAL_STATE_NOT_INSIDE_SAMPLING_BOX     = 12,   // initialStateAndInput is not inside the sampling box.
    INPUT_FINAL_POSE_NOT_INSIDE_SAMPLING_BOX        = 13,   // finalPose is not inside the sampling box.
    INPUT_STATIC_OBSTACLES                          = 14,   // Vertices of staticObstacles are not finite, not convex or vertex order is inconsistent.

    // model parameters
    MODEL_MATF                                      = 15,   // matF is not finite.
    MODEL_MATB                                      = 16,   // matB is not finite.
    MODEL_INV_MATB                                  = 17,   // matB is not invertable.
    MODEL_VEC_TIMECONSTANTS_XYN                     = 18,   // vecTimeconstantsXYN is not finite or not positive.
    MODEL_VEC_TIMECONSTANTS_INPUT                   = 19,   // vecTimeconstantsInput is not finite or not positive.
    MODEL_LOWER_LIMIT_XYN                           = 20,   // lowerLimitXYN is not finite.
    MODEL_UPPER_LIMIT_XYN                           = 21,   // upperLimitXYN is not finite.
    MODEL_LOWER_GREATER_UPPER_LIMIT                 = 22,   // lowerLimitXYN is greater than upperLimitXYN.

    // costmap parameters
    COSTMAP_MOD_BREAKPOINTS                         = 23,   // modBreakpoints is not positive.
    COSTMAP_RESOLUTION                              = 24,   // resolution is not finite or is too low.
    COSTMAP_DISTANCE_SCALE                          = 25,   // distanceScale is not finite or negative.
    COSTMAP_DISTANCE_DECAY                          = 26,   // distanceDecay is not finite or not positive.

    // metric parameters
    METRIC_WEIGHT_PSI                               = 27,   // weightPsi is not finite or not positive.
    METRIC_WEIGHT_SWAY                              = 28,   // weightSway is not finite or negative.
    METRIC_WEIGHT_REVERSE_SCALE                     = 29,   // weightReverseScale is not finite or negative.
    METRIC_WEIGHT_REVERSE_DECAY                     = 30,   // weightReverseDecay is not finite or not positive.

    // geometry parameters
    GEOMETRY_MAX_POSITION_DEVIATION                 = 31,   // collisionCheckMaxPositionDeviation is not finite or too low.
    GEOMETRY_MAX_ANGLE_DEVIATION                    = 32,   // collisionCheckMaxAngleDeviation is not finite or too low.
    GEOMETRY_VEHICLE_SHAPE                          = 33,   // Vertices of vehicleShape are not finite, not convex or vertex order is inconsistent.
    GEOMETRY_SKELETAL_POINTS                        = 34,   // skeletalPoints is empty or not finite.

    // path planner parameters
    PATHPLANNER_PERIOD_GOAL_SAMPLING                = 35,   // periodGoalSampling for path planner is not positive.

    // motion planner parameters
    MOTIONPLANNER_SAMPLING_RANGE_POSITION           = 36,   // samplingRangePosition is not finite or not positive.
    MOTIONPLANNER_SAMPLING_RANGE_ANGLE              = 37,   // samplingRangeAngle is not finite or not positive.
    MOTIONPLANNER_PERIOD_GOAL_SAMPLING              = 38,   // periodGoalSampling for motion planner is not positive.
    MOTIONPLANNER_SAMPLETIME                        = 39,   // sampletime is not finite or not positive.
    MOTIONPLANNER_MAX_INPUT_PATH_LENGTH             = 40,   // maxInputPathLength is not finite or not positive.

    // controller parameters
    CONTROLLER_MAT_K                                = 41,   // matK is not finite.
    CONTROLLER_MAX_RADIUS_X                         = 42,   // maxRadiusX is not finite or not positive.
    CONTROLLER_MAX_RADIUS_Y                         = 43,   // maxRadiusY is not finite or not positive.
    CONTROLLER_MAX_RADIUS_PSI                       = 44,   // maxRadiusPsi is not finite or not positive.
    CONTROLLER_MIN_RADIUS_POSITION                  = 45,   // minRadiusPosition is not finite or not positive.

    // region of attraction parameters
    REGION_OF_ATTRACTION_RANGE_POSE                 = 46,   // rangePose is not finite or not positive.
    REGION_OF_ATTRACTION_RANGE_UVR                  = 47,   // rangeUVR is not finite or not positive.
    REGION_OF_ATTRACTION_RANGE_XYN                  = 48,   // rangeXYN is not finite or not positive.

    // online planner parameters
    ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH_RESET   = 49,   // maxComputationTimePathOnReset is not finite or not positive.
    ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION_RESET = 50,   // maxComputationTimeMotionOnReset is not finite or not positive.
    ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH         = 51,   // maxComputationTimePath is not finite or not positive.
    ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION       = 52,   // maxComputationTimeMotion is not finite or not positive.
    ONLINEPLANNER_ADDITIONAL_AHEAD_PLANNING_TIME    = 53,   // additionalAheadPlanningTime is not finite or not positive.
    ONLINEPLANNER_ADDITIONAL_TRAJECTORY_DURATION    = 54,   // additionalTrajectoryDuration is not finite or not positive.
    ONLINEPLANNER_TIME_KEEP_PAST_TRAJECTORY         = 55,   // timeKeepPastTrajectory is not finite or negative.
    ONLINEPLANNER_NUM_SIMULATION_STEPS              = 56,   // The number of simulation steps is too high: either planning time is too high and/or sampletime is too low.

    // async planner parameters
    ASYNCPLANNER_TIMESTAMP                          = 57,   // timestamp is not finite.
    ASYNCPLANNER_TIMEOUT_INPUT                      = 58    // timeoutInput is not finite.
};


} /* namespace: mpsv */

