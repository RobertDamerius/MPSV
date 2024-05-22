function PlotTrajectory(trajectory, vehicleShape, sampletime)
    %mpsv.PlotTrajectory Plot the trajectory.
    % 
    % PARAMETER
    % trajectory ... Solution trajectory of the motion planner.
    % vehicleShape ... The static vehicle shape used by the motion planner.
    % sampletime ... The sampletime of the trajectory.
    stride = 1 / sampletime;
    path = trajectory(1:3,1:stride:end);
    N = size(path,2);
    plot(path(1,:), path(2,:),'Marker','.','Color',[0.8 0.2 0.0]);
    stateHold = ishold();
    hold on;
    shapes = repmat(polyshape, N, 1);
    for j = 1:N
        pose = path(:,j);
        R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
        numShapes = numel(vehicleShape);
        for i = 1:numShapes
            if(1 == i)
                shapes(j) = polyshape((R * vehicleShape{i} + [pose(1); pose(2)])');
            else
                shapes(j) = union(shapes(j), polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
            end
        end
    end
    if(size(trajectory,2))
        pose = trajectory(1:3,end);
        R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
        numShapes = numel(vehicleShape);
        vshape = polyshape.empty();
        for i = 1:numShapes
            if(1 == i)
                vshape = polyshape((R * vehicleShape{i} + [pose(1); pose(2)])');
            else
                vshape = union(vshape, polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
            end
        end
        shapes = [shapes; vshape];
    end
    plot(shapes, 'FaceColor', [0.8 0.2 0.0], 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1);
    if(~stateHold), hold off; end
end

