function [skeletalPoints, vehicleDimension] = BuildSkeletalPoints(verticesVehicleShape)
    %BuildSkeletalPoints
    % Find the min/max values for the x coordinate and calculate the average y coordinate for all vertices.
    % The skeletal points are then two points [xmax; ymean] and [xmin; ymean].
    % 
    % RETURN
    % vehicleDimension ... 2-by-1 vector denoting the AABB dimension over all vertices [x_dim; y_dim].
    N = size(verticesVehicleShape,2);
    xmax = -inf;
    xmin = inf;
    ymax = -inf;
    ymin = inf;
    for i = 1:N
        if(isfinite(verticesVehicleShape(1,i)) && isfinite(verticesVehicleShape(2,i)))
            xmin = min(xmin, verticesVehicleShape(1,i));
            ymin = min(ymin, verticesVehicleShape(2,i));
            xmax = max(xmax, verticesVehicleShape(1,i));
            ymax = max(ymax, verticesVehicleShape(2,i));
        end
    end
    skeletalPoints = [zeros(2,1), nan(2,9)];
    vehicleDimension = zeros(2,1);
    if(isfinite(xmax))
        y = ymin + (ymax - ymin) / 2;
        skeletalPoints(:,1) = [xmax; y];
        skeletalPoints(:,2) = [xmin; y];
        vehicleDimension = [xmax - xmin; ymax - ymin];
    end
end

