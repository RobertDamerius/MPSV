function PlotPath(path, vehicleShape)
    %mpsv.PlotPath Plot the path.
    % 
    % PARAMETER
    % path ... Solution path of the path planner.
    % vehicleShape ... The static vehicle shape used by the motion planner.
    N = size(path,2);
    plot(path(1,:), path(2,:),'Marker','o','Color',[0 0.2 0.8]);
    stateHold = ishold();
    hold on;
    shapes = repmat(polyshape, N, 1);
    for k = 1:N
        pose = path(:,k);
        R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
        numShapes = numel(vehicleShape);
        for i = 1:numShapes
            if(1 == i)
                shapes(k) = polyshape((R * vehicleShape{i} + [pose(1); pose(2)])');
            else
                shapes(k) = union(shapes(k), polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
            end
        end
    end
    plot(shapes, 'FaceColor', [0 0.2 0.8], 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1);
    if(~stateHold), hold off; end
end

