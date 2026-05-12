% Make sure, that the package is in the path
clear all;
thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
addpath(fullfile(thisDirectory,'..','..','matlab','packages'));

% Decode the data log file
fprintf('Reading data file ... ');
data = mpsv.ReadLogFile('dataIterationPerformanceTest');
fprintf('data contains %d entries\n',numel(data));

% plot scene by scene if all data is recorded by the benchmark app (see IterationPerformanceTest.cpp:135)
% for i = 1:numel(data)
%     figure(1);
%     clf; hold on;
%     mpsv.PlotStaticObstacles(data{i}.input.staticObstacles);
%     mpsv.PlotPath(data{i}.output.pathPlanner.path, data{i}.parameter.geometry.vehicleShape);
%     mpsv.PlotTrajectory(data{i}.output.motionPlanner.trajectory, data{i}.parameter.geometry.vehicleShape, data{i}.output.motionPlanner.sampletime);
%     axis equal; grid on; box on; view(90,-90); xlabel('x'); ylabel('y');
%     title(['i=',num2str(i)]);
%     drawnow();
%     pause(2);
% end

% get number of iterations from data
numIterationsPathPlanner = zeros(numel(data),1,'uint32');
numIterationsMotionPlanner = zeros(numel(data),1,'uint32');
for i = 1:numel(data)
    numIterationsPathPlanner(i) = data{i}.output.pathPlanner.numberOfPerformedIterations;
    numIterationsMotionPlanner(i) = data{i}.output.motionPlanner.numberOfPerformedIterations;
    if(~data{i}.output.pathPlanner.goalReached || ~data{i}.output.motionPlanner.goalReached)
        warning(['goal not reached for scene ',num2str(i)]);
    end
end

% plot histograms
f1 = figure(1); clf;
f1.Position = [f1.Position(1:2), 600, 400];
histogram(numIterationsPathPlanner);
grid on; box on; xlabel('Iterations'); ylabel('Frequency'); title('Path Planner');
f1.Renderer = "painters";

f2 = figure(2); clf;
f2.Position = [f2.Position(1:2), 600, 400];
histogram(numIterationsMotionPlanner);
grid on; box on; xlabel('Iterations'); ylabel('Frequency'); title('Motion Planner');
f2.Renderer = "painters";
