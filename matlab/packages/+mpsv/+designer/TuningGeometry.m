classdef TuningGeometry < handle
    properties
        % 2-by-10 matrix of skeletal points (b-frame) at which the cost map is to be evaluated.
        skeletalPoints (2,10) double = nan(2,10)

        % Resolution (> 1e-3) of the grid map (dimension of one cell).
        costMapResolution (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(costMapResolution,0.001)} = 1.0
    end
end
