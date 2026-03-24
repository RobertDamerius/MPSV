classdef ASVMetric < handle
    properties
        % Weighting (> 0) for heading angle (psi) in distance metric function.
        weightPsi (1,1) double {mustBeFinite, mustBePositive} = 5

        % Weighting (>= 0) for sway movement (heading angle with respect to perpenticular direction of movement).
        weightSway (1,1) double {mustBeFinite, mustBeNonnegative} = 0

        % Weighting (>= 0) for sway and reverse movement (heading angle with respect to line angle).
        weightReverseScale (1,1) double {mustBeFinite, mustBeNonnegative} = 1

        % Decay factor (> 0) for the weighting function that weights sway and reverse movement.
        weightReverseDecay (1,1) double {mustBeFinite, mustBePositive} = 1

        % Scale factor (>= 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
        distanceScale (1,1) double {mustBeFinite, mustBeNonnegative} = 5

        % Decay factor (> 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
        distanceDecay (1,1) double {mustBeFinite, mustBePositive} = 0.01
    end
end
