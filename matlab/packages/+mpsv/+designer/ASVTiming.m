classdef ASVTiming < handle
    properties
        % Sampletime to be used for fixed-step trajectory simulation.
        sampletime (1,1) double {mustBeFinite, mustBePositive} = 0.1

        % Maximum computation time in seconds for path planning.
        maxComputationTimePath (1,1) double {mustBeFinite, mustBePositive} = 1;

        % Maximum computation time in seconds for motion planning.
        maxComputationTimeMotion (1,1) double {mustBeFinite, mustBePositive} = 1;
    end
end
