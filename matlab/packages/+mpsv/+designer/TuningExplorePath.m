classdef TuningExplorePath < handle
    properties
        % 3-by-12 control gain matrix for pose control.
        matK (3,12) double {mustBeFinite} = zeros(3,12)

        % Maximum look-ahead distance for longitudinal distance during pose control.
        maxRadiusX (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Maximum look-ahead distance for lateral distance during pose control.
        maxRadiusY (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Maximum look-ahead distance for angular distance during pose control.
        maxRadiusPsi (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
        minRadiusPosition (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Pose box constraints for the region of attraction.
        rangePose (3,1) double {mustBeFinite, mustBeNonnegative} = [1;1;1]

        % Velocity box constraints for the region of attraction.
        rangeUVR (3,1) double {mustBeFinite, mustBeNonnegative} = [1;1;1]

        % The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.
        rangeXYN (3,1) double {mustBeFinite, mustBeNonnegative} = [1;1;1]
    end
end
