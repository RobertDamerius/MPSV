classdef TuningMotionPlanner < handle
    properties
        % Range in meters for sampling the position around a given path.
        samplingRangePosition (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Range in radians for sampling the angle around a given path.
        samplingRangeAngle (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
        maxInputPathLength (1,1) double {mustBeFinite, mustBeNonnegative} = 1
    end
end
