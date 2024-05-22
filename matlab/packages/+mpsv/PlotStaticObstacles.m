function PlotStaticObstacles(staticObstacles)
    %mpsv.PlotStaticObstacles Plot static obstacles.
    % 
    % PARAMETER
    % staticObstacles ... The static obstacles to be plotted.
    N = numel(staticObstacles);
    shapes = repmat(polyshape, N, 1);
    for k = 1:N
        shapes(k) = polyshape(staticObstacles{k}');
    end
    plot(shapes,'FaceColor',[0 0 0]);
end
