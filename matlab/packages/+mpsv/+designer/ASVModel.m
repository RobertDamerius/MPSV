classdef ASVModel < handle
    properties
        % 3-by-12 coefficient matrix of dynamical model uvr_dot = matF*[u;v;r;v*r;u*r;u*v;u^2;v^2;r^2;u^3;v^3;r^3] + matB*[X;Y;N].
        matF (3,12) double {mustBeFinite} = zeros(3,12)

        % 3-by-3 input matrix of dynamical model uvr_dot = matF*[u;v;r;v*r;u*r;u*v;u^2;v^2;r^2;u^3;v^3;r^3] + matB*[X;Y;N].
        matB (3,3) double {mustBeFinite} = zeros(3)

        % Timeconstants {TX, TY, TN} for input force dynamics.
        vecTimeconstantsXYN (3,1) double {mustBeFinite, mustBePositive} = ones(3,1)

        % Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
        vecTimeconstantsInput (3,1) double {mustBeFinite, mustBePositive} = ones(3,1)

        % Absolute saturation value for input vector.
        vecSatXYN (3,1) double {mustBeFinite, mustBePositive} = ones(3,1)
    end
end
