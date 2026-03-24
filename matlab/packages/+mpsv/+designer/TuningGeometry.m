classdef TuningGeometry < handle
    properties
        % 2-by-10 matrix of skeletal points (b-frame) at which the cost map is to be evaluated.
        skeletalPoints (2,10) double = nan(2,10)

        % Resolution (> 1e-3) of the grid map (dimension of one cell).
        costMapResolution (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(costMapResolution,0.001)} = 0.1

        % A modulo factor (> 0) that indicates when to calculate the cost using the objective function and when to do bilinear interpolation.
        costMapModBreakpoints (1,1) int32 {mustBeNonnegative} = 10
    end
end
