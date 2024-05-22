function PlotTestScenario(data)
    %mpsv.PlotTestScenario Plot results of a test scenario.
    % 
    % PARAMETER
    % data ... Recorded data of a test scenario

    % Check fields
    assert(isfield(data,'parameter'), 'Input data must contain parameter field!');
    assert(isfield(data.parameter,'geometry'), 'Input data must contain parameter.geometry field!');
    assert(isfield(data,'sequentialPlanner'), 'Input data must contain sequentialPlanner field!');
    assert(isfield(data,'input'), 'Input data must contain pathPlanner.input field!');
    assert(isfield(data,'output'), 'Input data must contain pathPlanner.output field!');

    % Initial and final pose
    xI = data.input.initialStateAndInput(1:3);
    xG = data.input.finalPose;

    % Big left 2D plot
    subplot(4,3,[1,2,4,5,7,8,10,11]);
    hold on;
    if(isfield(data.input,'staticObstacles')), mpsv.PlotStaticObstacles(data.input.staticObstacles); end
    if(isfield(data.sequentialPlanner.pathPlanner,'tree')), mpsv.PlotTreePathPlanner(data.sequentialPlanner.pathPlanner.tree); end
    %if(isfield(data.output.motionPlanner,'referencePath')), mpsv.PlotPath(data.output.motionPlanner.referencePath, data.parameter.geometry.vehicleShape); end
    if(isfield(data.output.pathPlanner,'path')), mpsv.PlotPath(data.output.pathPlanner.path, data.parameter.geometry.vehicleShape); end
    if(isfield(data.output.motionPlanner,'trajectory')), mpsv.PlotTrajectory(data.output.motionPlanner.trajectory, data.parameter.geometry.vehicleShape, data.output.motionPlanner.sampletime); end
    if(isfield(data.sequentialPlanner.motionPlanner,'tree')), mpsv.PlotTreeMotionPlanner(data.sequentialPlanner.motionPlanner.tree); end
    mpsv.PlotVehicleShape(data.parameter.geometry.vehicleShape, xI); text(xI(1),xI(2),'START');
    mpsv.PlotVehicleShape(data.parameter.geometry.vehicleShape, xG); text(xG(1),xG(2),'GOAL');
    axis equal;
    grid on;
    box on;
    view(90,-90);
    xlabel('x (m)');
    ylabel('y (m)');

    % Time plots (right)
    t = data.output.motionPlanner.sampletime * (0:(size(data.output.motionPlanner.trajectory,2)-1))';
    lineWidth = 1.5;

    s(1) = subplot(4,3,3);
    hold on;
    plot(t, data.output.motionPlanner.trajectory(1,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(2,:), 'LineWidth', lineWidth);
    plot(t, rad2deg(data.output.motionPlanner.trajectory(3,:)), 'LineWidth', lineWidth);
    ylabel('x, y, \psi (deg)');
    hLegend = legend('x','y','\psi');
    hLegend.BoxFace.ColorType = 'truecoloralpha';
    hLegend.BoxFace.ColorData = uint8([255;255;255;180]);
    grid on;
    box on;

    s(2) = subplot(4,3,6);
    hold on;
    plot(t, data.output.motionPlanner.trajectory(4,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(5,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(6,:), 'LineWidth', lineWidth);
    ylabel('u, v, r');
    hLegend = legend('u','v','r');
    hLegend.BoxFace.ColorType = 'truecoloralpha';
    hLegend.BoxFace.ColorData = uint8([255;255;255;180]);
    grid on;
    box on;

    s(3) = subplot(4,3,9);
    hold on;
    plot(t, data.output.motionPlanner.trajectory(7,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(8,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(9,:), 'LineWidth', lineWidth);
    ylabel('X, Y, N');
    hLegend = legend('X','Y','N');
    hLegend.BoxFace.ColorType = 'truecoloralpha';
    hLegend.BoxFace.ColorData = uint8([255;255;255;180]);
    grid on;
    box on;

    s(4) = subplot(4,3,12);
    hold on;
    plot(t, data.output.motionPlanner.trajectory(10,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(11,:), 'LineWidth', lineWidth);
    plot(t, data.output.motionPlanner.trajectory(12,:), 'LineWidth', lineWidth);
    ylabel('Xc, Yc, Nc');
    hLegend = legend('Xc','Yc','Nc');
    hLegend.BoxFace.ColorType = 'truecoloralpha';
    hLegend.BoxFace.ColorData = uint8([255;255;255;180]);
    grid on;
    box on;

    linkaxes(s, 'x');
    subplot(4,3,[1,2,4,5,7,8,10,11]);
end

