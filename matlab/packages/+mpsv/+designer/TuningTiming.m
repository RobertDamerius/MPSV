classdef TuningTiming < handle
    properties
        % Maximum computation time in seconds for path planning after a reset.
        maxComputationTimePathOnReset (1,1) double {mustBeFinite, mustBePositive} = 1

        % Maximum computation time in seconds for motion planning after a reset.
        maxComputationTimeMotionOnReset (1,1) double {mustBeFinite, mustBePositive} = 1

        % Additional time added to the estimated computation time in seconds to obtain the future timepoint from where to start the next planning problem. This value must be greater than zero.
        additionalAheadPlanningTime (1,1) double {mustBeFinite, mustBePositive} = 1

        % Additional time added to the ahead planning time (= estimated computation time (no reset) + additional ahead planning time) to obtain the minimum time duration of a trajectory to make sure that the next solve operation can access a future point of the trajectory. This value must be greater than zero.
        additionalTrajectoryDuration (1,1) double {mustBeFinite, mustBePositive} = 1

        % Time in seconds to keep from a past trajectory. The past data of the previous trajectory is inserted at the beginning of a new solution. Inserting past data helps to handle imperfect time synchronization between this trajectory generator and the user of the trajectory data.
        timeKeepPastTrajectory (1,1) double {mustBeFinite, mustBePositive} = 1
    end
end
