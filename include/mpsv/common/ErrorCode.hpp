#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


/**
 * @brief Scoped enumeration for MPSV error codes.
 */
enum class error_code : uint8_t {
    NONE                                                  = 0,    // No error.
    NOT_AVAILABLE                                         = 1,    // Default error code if error is not available yet, but it should be marked as error, e.g. as initial error code. This value is also used to indicate a timeout.

    // planner inputs
    INPUT_TIMESTAMP                                       = 2,    // timestamp is not finite.
    INPUT_ORIGIN_LLA                                      = 3,    // originLLA is not finite.
    INPUT_ORIGIN_OLD_TO_NEW                               = 4,    // originOldToNew is not finite.
    INPUT_INITIAL_POSE                                    = 5,    // initialPose is not finite.
    INPUT_INITIAL_STATE_AND_INPUT                         = 6,    // initialStateAndInput is not finite.
    INPUT_INITIAL_PATH                                    = 7,    // initialPath is not finite.
    INPUT_FINAL_POSE                                      = 8,    // finalPose is not finite.
    INPUT_SAMPLING_BOX_CENTER_POSE                        = 9,    // samplingBoxCenterPose is not finite.
    INPUT_SAMPLING_BOX_DIMENSION                          = 10,   // samplingBoxDimension is not finite.
    INPUT_INITIAL_POSE_NOT_INSIDE_SAMPLING_BOX            = 11,   // initalPose is not inside the sampling box.
    INPUT_INITIAL_STATE_NOT_INSIDE_SAMPLING_BOX           = 12,   // initialStateAndInput is not inside the sampling box.
    INPUT_FINAL_POSE_NOT_INSIDE_SAMPLING_BOX              = 13,   // finalPose is not inside the sampling box.
    INPUT_STATIC_OBSTACLES                                = 14,   // Vertices of staticObstacles are not finite, not convex or vertex order is inconsistent.
    INPUT_MIN_TRAJECTORY_DURATION                         = 15,   // minTrajectoryDuration is not finite or is negative.

    // parameters
    PARAM_MODEL_MATF                                      = 16,   // matF is not finite.
    PARAM_MODEL_MATB                                      = 17,   // matB is not finite.
    PARAM_MODEL_INV_MATB                                  = 18,   // matB is not invertable.
    PARAM_MODEL_VEC_TIMECONSTANTS_XYN                     = 19,   // vecTimeconstantsXYN is not finite or not positive.
    PARAM_MODEL_VEC_TIMECONSTANTS_INPUT                   = 20,   // vecTimeconstantsInput is not finite or not positive.
    PARAM_MODEL_LOWER_LIMIT_XYN                           = 21,   // lowerLimitXYN is not finite.
    PARAM_MODEL_UPPER_LIMIT_XYN                           = 22,   // upperLimitXYN is not finite.
    PARAM_MODEL_LOWER_GREATER_UPPER_LIMIT                 = 23,   // lowerLimitXYN is greater than upperLimitXYN.
    PARAM_COSTMAP_MOD_BREAKPOINTS                         = 24,   // modBreakpoints is not positive.
    PARAM_COSTMAP_RESOLUTION                              = 25,   // resolution is not finite or is too low.
    PARAM_COSTMAP_DISTANCE_SCALE                          = 26,   // distanceScale is not finite or negative.
    PARAM_COSTMAP_DISTANCE_DECAY                          = 27,   // distanceDecay is not finite or not positive.
    PARAM_METRIC_WEIGHT_PSI                               = 28,   // weightPsi is not finite or not positive.
    PARAM_METRIC_WEIGHT_SWAY                              = 29,   // weightSway is not finite or negative.
    PARAM_METRIC_WEIGHT_REVERSE_SCALE                     = 30,   // weightReverseScale is not finite or negative.
    PARAM_METRIC_WEIGHT_REVERSE_DECAY                     = 31,   // weightReverseDecay is not finite or not positive.
    PARAM_GEOMETRY_MAX_POSITION_DEVIATION                 = 32,   // collisionCheckMaxPositionDeviation is not finite or too low.
    PARAM_GEOMETRY_MAX_ANGLE_DEVIATION                    = 33,   // collisionCheckMaxAngleDeviation is not finite or too low.
    PARAM_GEOMETRY_VEHICLE_SHAPE                          = 34,   // Vertices of vehicleShape are not finite, not convex or vertex order is inconsistent.
    PARAM_GEOMETRY_SKELETAL_POINTS                        = 35,   // skeletalPoints is empty or not finite.
    PARAM_PATHPLANNER_PERIOD_GOAL_SAMPLING                = 36,   // periodGoalSampling for path planner is not positive.
    PARAM_MOTIONPLANNER_SAMPLING_RANGE_POSITION           = 37,   // samplingRangePosition is not finite or not positive.
    PARAM_MOTIONPLANNER_SAMPLING_RANGE_ANGLE              = 38,   // samplingRangeAngle is not finite or not positive.
    PARAM_MOTIONPLANNER_PERIOD_GOAL_SAMPLING              = 39,   // periodGoalSampling for motion planner is not positive.
    PARAM_MOTIONPLANNER_SAMPLETIME                        = 40,   // sampletime is not finite or not positive.
    PARAM_MOTIONPLANNER_MAX_INPUT_PATH_LENGTH             = 41,   // maxInputPathLength is not finite or not positive.
    PARAM_CONTROLLER_MAT_K                                = 42,   // matK is not finite.
    PARAM_CONTROLLER_MAX_RADIUS_X                         = 43,   // maxRadiusX is not finite or not positive.
    PARAM_CONTROLLER_MAX_RADIUS_Y                         = 44,   // maxRadiusY is not finite or not positive.
    PARAM_CONTROLLER_MAX_RADIUS_PSI                       = 45,   // maxRadiusPsi is not finite or not positive.
    PARAM_CONTROLLER_MIN_RADIUS_POSITION                  = 46,   // minRadiusPosition is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_POSE                 = 47,   // rangePose is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_UVR                  = 48,   // rangeUVR is not finite or not positive.
    PARAM_REGION_OF_ATTRACTION_RANGE_XYN                  = 49,   // rangeXYN is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH_RESET   = 50,   // maxComputationTimePathOnReset is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION_RESET = 51,   // maxComputationTimeMotionOnReset is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH         = 52,   // maxComputationTimePath is not finite or not positive.
    PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION       = 53,   // maxComputationTimeMotion is not finite or not positive.
    PARAM_ONLINEPLANNER_ADDITIONAL_AHEAD_PLANNING_TIME    = 54,   // additionalAheadPlanningTime is not finite or not positive.
    PARAM_ONLINEPLANNER_ADDITIONAL_TRAJECTORY_DURATION    = 55,   // additionalTrajectoryDuration is not finite or not positive.
    PARAM_ONLINEPLANNER_TIME_KEEP_PAST_TRAJECTORY         = 56,   // timeKeepPastTrajectory is not finite or negative.
    PARAM_ONLINEPLANNER_NUM_SIMULATION_STEPS              = 57,   // The number of simulation steps is too high: either planning time is too high and/or sampletime is too low.
    PARAM_ASYNCPLANNER_TIMESTAMP                          = 58,   // timestamp is not finite.
    PARAM_ASYNCPLANNER_TIMEOUT_INPUT                      = 59,   // timeoutInput is not finite.

    // path planner
    PATHPLANNER_INITIAL_POSE_COLLISION                    = 60,   // The initial pose of the path planner collides with static obstacles.
    PATHPLANNER_OUT_OF_NODES                              = 61,   // The path planner has run out of nodes and cannot sample new nodes anymore.

    // motion planner
    MOTIONPLANNER_INITIAL_POSE_COLLISION                  = 62,   // The initial pose of the motion planner collides with static obstacles.
    MOTIONPLANNER_OUT_OF_NODES                            = 63,   // The motion planner has run out of nodes and cannot sample new nodes anymore.
    MOTIONPLANNER_INFEASIBLE_TRAJECTORY                   = 64    // The final motion planner trajectory is dynamically infeasible.
};


} /* namespace: mpsv */
