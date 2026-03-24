classdef ASVGeometry < handle
    properties
        % 2-by-N matrix representing the vehicle shape. Multiple polygons are separated by non-finite vertices.
        verticesVehicleShape (2,:) double = zeros(2,0)

        % Maximum position deviation for path subdivision during collision checking. Must be at least 0.01 meters. The default value is 0.1.
        collisionCheckMaxPositionDeviation (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(collisionCheckMaxPositionDeviation,0.01)} = 0.1

        % Maximum angle deviation for path subdivision during collision checking. Must be at least 1 degree (0.0174532925199433 radians). The default value is 5 deg.
        collisionCheckMaxAngleDeviation (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(collisionCheckMaxAngleDeviation,0.0174532925199433)} = deg2rad(5)
    end
end
