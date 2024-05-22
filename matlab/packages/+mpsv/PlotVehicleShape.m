function PlotVehicleShape(vehicleShape, pose)
    %mpsv.PlotVehicleShape Plot a given static vehicle shape at the specified pose.
    % 
    % PARAMETER
    % vehicleShape ... The static vehicle shape to be plotted.
    % pose ... The pose to which to plot the static vehicle shape.
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
    plot(vshape, 'FaceColor', [0 0.2 0.8]);
end

