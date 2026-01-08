% Make sure, that the package is in the path
clear all;
thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
addpath(fullfile(thisDirectory,'..','..','matlab','packages'));

% Decode the data log file
fprintf('Reading data file ... ');
data = mpsv.ReadLogFile(fullfile(thisDirectory,'data'));
fprintf('data contains %d entries\n',numel(data));
staticObstacles = data{1}.input.staticObstacles;
vehicleShape = data{1}.parameter.geometry.vehicleShape;
q0 = data{1}.input.initialStateAndInput(1:3);
qG = data{1}.input.finalPose;
path = data{1}.output.pathPlanner.path;
trajectory = data{1}.output.motionPlanner.trajectory;
sampletime = data{1}.output.motionPlanner.sampletime;
tree = data{1}.planner.motionPlanner.tree;

% figure 1: display planning problem
f1 = figure(1); clf; hold on;
f1.Position = [f1.Position(1:2), 600, 400];
mpsv.PlotStaticObstacles(staticObstacles);
PlotPose(q0, vehicleShape, GetColor(0));
PlotPose(qG, vehicleShape, GetColor(1));
PlotPathGray(path, vehicleShape);
view(90,-90); grid on; box on; axis image; xlabel('x in m'); ylabel('y in m');
xlim([-150,100]);
ylim([-200,400]);
f1.Renderer = "painters";

% figure 2: display result
f2 = figure(2); clf; hold on;
f2.Position = [f2.Position(1:2), 600, 400];
mpsv.PlotStaticObstacles(staticObstacles);
PlotTrajectory(trajectory, vehicleShape, sampletime);
PlotPose(q0, vehicleShape, GetColor(0));
PlotPose(qG, vehicleShape, GetColor(1));
view(90,-90); grid on; box on; axis image; xlabel('x in m'); ylabel('y in m');
xlim([-150,100]);
ylim([-200,400]);
f2.Renderer = "painters";

% figure 3: velocity / commanded force
f3 = figure(3); clf;
f3.Position = [f3.Position(1:2), 500, 400];
subplot(2,1,1); hold on;
t = ((1:size(trajectory,2))-1)*sampletime;
u = trajectory(4,:);
v = trajectory(5,:);
r = trajectory(6,:) * 180.0 / pi * 60;
yyaxis left;
plot(t,u,'LineWidth',2,'Color','#3088CD');
plot(t,v,'--','LineWidth',2,'Color','#3088CD');
ylim([-1.2,1.2]*max(max(abs(u)),max(abs(v))));
ylabel('u, v in m/s');
yyaxis right;
plot(t,r,'LineWidth',2,'Color','#CC332F');
ylim([-1.2,1.2]*max(abs(r)));
ylabel('r in °/min');
grid on; box on;
legend('u','v','r');
subplot(2,1,2); hold on;
X = trajectory(10,:) / 1000.0;
Y = trajectory(11,:) / 1000.0;
N = trajectory(12,:) / 1000.0;
yyaxis left;
plot(t,X,'LineWidth',2,'Color','#3088CD');
plot(t,Y,'--','LineWidth',2,'Color','#3088CD');
ylim([-1.2,1.2]*max(max(abs(X)),max(abs(Y))));
ylabel('X_c, Y_c in kN');
yyaxis right;
plot(t,N,'LineWidth',2,'Color','#CC332F');
ylim([-1.2,1.2]*max(abs(N)));
ylabel('N_c in kN');
xlabel('t in s');
grid on; box on;
legend('X_c','Y_c','N_c');
f3.Renderer = "painters";

% figure 4: motion planner tree
f4 = figure(4); clf; hold on;
f4.Position = [f4.Position(1:2), 600, 400];
mpsv.PlotStaticObstacles(staticObstacles);
PlotTree(tree);
PlotTrajectoryLine(trajectory);
plot(q0(1), q0(2), '.', 'MarkerSize', 12, 'Color', GetColor(0));
plot(qG(1), qG(2), '.', 'MarkerSize', 12, 'Color', GetColor(1));
view(90,-90); grid on; box on; axis image; xlabel('x in m'); ylabel('y in m');
xlim([-150,100]);
ylim([-200,400]);
f1.Renderer = "painters";


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% HELPER FUNCTIONS FOR NICE COLORFUL PLOTS
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function c = GetColor(ratio)
    ratio = min(max(ratio, 0.0), 1.0);
    colorBlue = [hex2dec('30'), hex2dec('88'), hex2dec('CD')] / 255;
    colorGreen = [hex2dec('55'), hex2dec('CC'), hex2dec('25')] / 255;
    c = colorGreen + ratio * (colorBlue - colorGreen);
end

function s = PositionToRatio(x,y)
    x = reshape(x, [1,numel(x)]);
    y = reshape(y, [1,numel(y)]);
    s = zeros(1,numel(x));
    L = 0;
    for i = 2:numel(x)
        dx = x(i) - x(i-1);
        dy = y(i) - y(i-1);
        L = L + dx*dx + dy*dy;
        s(i) = L;
    end
    s = s / L;
end

function PlotColoredLine(x, y, s)
    x = reshape(x, [1,numel(x)]);
    y = reshape(y, [1,numel(y)]);
    x = [x, NaN];
    y = [y, NaN];
    s = [s, 1];
    c = GetColor(s');
    C = zeros([size(x),3]);
    C(1,:,:) = c;
    patch(x,y,C, 'edgecolor', 'interp', 'LineWidth', 2);
end

function PlotTrajectory(trajectory, vehicleShape, sampletime)
    stride = 10 / sampletime;
    path = trajectory(1:3,1:stride:end);
    N = size(path,2);
    x = path(1,:);
    y = path(2,:);
    s = PositionToRatio(x,y);
    PlotColoredLine(x,y,s);
    alpha = 0.25;
    if(~isempty(vehicleShape))
        for k = 1:N
            pose = path(:,k);
            R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
            P = polyshape((R * vehicleShape{1} + [pose(1); pose(2)])');
            for i = 2:numel(vehicleShape)
                P = union(P, polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
            end
            plot(pose(1), pose(2),'Marker','.','Color',GetColor(s(k)), 'LineWidth',1.5);
            plot(P, 'FaceColor', GetColor(s(k)), 'FaceAlpha', alpha, 'EdgeColor', [0 0 0], 'EdgeAlpha', alpha);
        end
        if(size(trajectory,2))
            pose = trajectory(1:3,end);
            R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
            P = polyshape((R * vehicleShape{1} + [pose(1); pose(2)])');
            for i = 2:numel(vehicleShape)
                P = union(P, polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
            end
            plot(pose(1), pose(2),'Marker','.','Color',GetColor(s(k)), 'LineWidth',1.5);
            plot(P, 'FaceColor', GetColor(s(k)), 'FaceAlpha', alpha, 'EdgeColor', [0 0 0], 'EdgeAlpha', alpha);
        end
    end
end

function PlotPose(pose, vehicleShape, color)
    R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
    P = polyshape((R * vehicleShape{1} + [pose(1); pose(2)])');
    alpa = 1.0;
    for i = 2:numel(vehicleShape)
        P = union(P, polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
    end
    plot(pose(1), pose(2),'Marker','.','Color', color, 'LineWidth',1.2);
    plot(P, 'FaceColor', color, 'FaceAlpha', alpa, 'EdgeColor', [0 0 0], 'EdgeAlpha', alpa);
end

function PlotPathGray(path, vehicleShape)
    alpha = 0.25;
    lineColor = '#262626';
    faceColor = '#dddddd';
    edgeColor = '#262626';
    N = size(path,2);
    for k = 1:N
        pose = path(:,k);
        R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
        P = polyshape((R * vehicleShape{1} + [pose(1); pose(2)])');
        for i = 2:numel(vehicleShape)
            P = union(P, polyshape((R * vehicleShape{i} + [pose(1); pose(2)])'));
        end
        plot(pose(1), pose(2),'Marker','.','Color',lineColor, 'LineWidth',1.2);
        plot(P, 'FaceColor', faceColor, 'FaceAlpha', alpha, 'EdgeColor', edgeColor, 'EdgeAlpha', alpha);
    end
    x = path(1,:);
    y = path(2,:);
    plot(x,y,'o-','LineWidth', 2,'LineStyle','-','Color',lineColor);
end

function PlotTree(tree)
    edgeColor = [0.8 0.8 0.8];
    nodeColor = [0.6 0.6 0.6];
    activeIndices = (tree.activeIndices + int16(1));
    N = numel(activeIndices);
    for k = 1:N
        idx = activeIndices(k);
        if(tree.idxParent(idx) < int16(0)), continue; end
        x = tree.trajectoryFromRoot{idx}(1,:);
        y = tree.trajectoryFromRoot{idx}(2,:);
        plot(x,y,'Color',edgeColor);
        plot(x(end),y(end),'Color',nodeColor,'Marker','.');
    end
end

function PlotThickerColoredLine(x, y, s)
    x = reshape(x, [1,numel(x)]);
    y = reshape(y, [1,numel(y)]);
    x = [x, NaN];
    y = [y, NaN];
    s = [s, 1];
    c = GetColor(s');
    C = zeros([size(x),3]);
    C(1,:,:) = c;
    patch(x,y,C, 'edgecolor', 'interp', 'LineWidth', 1.5);
end

function PlotTrajectoryLine(trajectory)
    x = trajectory(1,:);
    y = trajectory(2,:);
    s = PositionToRatio(x,y);
    PlotColoredLine(x,y,s);
end

