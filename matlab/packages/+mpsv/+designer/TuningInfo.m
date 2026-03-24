classdef TuningInfo < handle
    properties
        % Maximum velocity that is reached when driving with satXYN.
        maxUVR (3,1) double {mustBeFinite} = zeros(3,1)

        % Steady-state velocity that is reached when driving with nominal control input
        steadyStateUVR (3,1) double {mustBeFinite} = zeros(3,1)
    end
end
