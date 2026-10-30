% Make sure, that the package is in the path
clear all;
thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
addpath(fullfile(thisDirectory,'..','..','matlab','packages'));

% Decode the data log file
fprintf('Reading data file ... ');
data = mpsv.ReadLogFile(fullfile(thisDirectory,'dataExamplePlanner'));
fprintf('dataExamplePlanner contains %d entries\n',numel(data));

% static obstacles and vehicle shape
staticObstacles = data{1}.input.staticObstacles;
vehicleShape = data{1}.parameter.sequentialPlanner.geometry.vehicleShape;

% create two figures
% figure1: x-y-plot for reference path and trajectory
% figure2: trajectory timeline
figure(1); clf; hold on; mpsv.PlotStaticObstacles(staticObstacles);
figure(2); clf;

% go through all data sets and plot to booth figures
tmin = data{2}.examplePlanner.timestamp;
tmax = data{end}.examplePlanner.timestamp + (size(data{2}.examplePlanner.trajectory,2) - 1) * data{end}.examplePlanner.sampletime;
colors = turbo(numel(data));
finalTraj.t = [];
finalTraj.x = [];
finalTraj.y = [];
finalTraj.psi = [];
finalTraj.u = [];
finalTraj.v = [];
finalTraj.r = [];
finalTraj.X = [];
finalTraj.Y = [];
finalTraj.N = [];
finalTraj.Xc = [];
finalTraj.Yc = [];
finalTraj.Nc = [];
for k = 2:numel(data)
    % figure1: scene plot
    figure(1);
    %path = data{k}.examplePlanner.motionPlanner.referencePath;
    path = data{k}.examplePlanner.pathPlanner.path;
    trajectory = data{k}.examplePlanner.trajectory;
    plot(path(1,:),path(2,:),'o--','Color','#ccc');
    plot(trajectory(1,:),trajectory(2,:),'Color',colors(k,:));
    plot(trajectory(1,1),trajectory(2,1),'o','Color',colors(k,:));
    view(90,-90);
    axis image;
    xlim([-100,100]);
    ylim([-100,100]);
    grid on;
    title([num2str(k-1) '/' num2str(numel(data)-1)]);
    drawnow();

    % figure2: trajectory timeline
    figure(2);
    t0 = data{k}.examplePlanner.timestamp;
    t = t0 + ((1:size(data{k}.examplePlanner.trajectory,2)) - 1) * data{k}.examplePlanner.sampletime;
    x = data{k}.examplePlanner.trajectory(1,:);
    y = data{k}.examplePlanner.trajectory(2,:);
    psi = data{k}.examplePlanner.trajectory(3,:);
    u = data{k}.examplePlanner.trajectory(4,:);
    v = data{k}.examplePlanner.trajectory(5,:);
    r = data{k}.examplePlanner.trajectory(6,:);
    X = data{k}.examplePlanner.trajectory(7,:);
    Y = data{k}.examplePlanner.trajectory(8,:);
    N = data{k}.examplePlanner.trajectory(9,:);
    Xc = data{k}.examplePlanner.trajectory(10,:);
    Yc = data{k}.examplePlanner.trajectory(11,:);
    Nc = data{k}.examplePlanner.trajectory(12,:);
    s(1) = subplot(3,4,1); hold on; plot(t,x,'Color',colors(k,:)); plot(t(1),x(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('x in m');
    s(2) = subplot(3,4,2); hold on; plot(t,u,'Color',colors(k,:)); plot(t(1),u(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('u in m/s');
    s(3) = subplot(3,4,3); hold on; plot(t,X,'Color',colors(k,:)); plot(t(1),X(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('X in N');
    s(4) = subplot(3,4,4); hold on; plot(t,Xc,'Color',colors(k,:)); plot(t(1),Xc(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('X_c in N');
    s(5) = subplot(3,4,5); hold on; plot(t,y,'Color',colors(k,:)); plot(t(1),y(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('y in m');
    s(6) = subplot(3,4,6); hold on; plot(t,v,'Color',colors(k,:)); plot(t(1),v(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('v in m/s');
    s(7) = subplot(3,4,7); hold on; plot(t,Y,'Color',colors(k,:)); plot(t(1),Y(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('Y in N');
    s(8) = subplot(3,4,8); hold on; plot(t,Yc,'Color',colors(k,:)); plot(t(1),Yc(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('Y_c in N');
    s(9) = subplot(3,4,9); hold on; plot(t,psi,'Color',colors(k,:)); plot(t(1),psi(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('\psi in rad');
    s(10) = subplot(3,4,10); hold on; plot(t,r,'Color',colors(k,:)); plot(t(1),r(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('r in rad/s');
    s(11) = subplot(3,4,11); hold on; plot(t,N,'Color',colors(k,:)); plot(t(1),N(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('N in N');
    s(12) = subplot(3,4,12); hold on; plot(t,Nc,'Color',colors(k,:)); plot(t(1),Nc(1),'o','Color',colors(k,:)); grid on; xlim([tmin, tmax]); ylabel('N_c in N');
    linkaxes(s,'x');
    drawnow();
    %pause(3);

    % save data for final trajectory
    t1 = t(end);
    if((k + 1) <= numel(data))
        t1 = data{k+1}.examplePlanner.timestamp;
    end
    steps = floor((t1 - t0) / data{k}.examplePlanner.sampletime);
    steps = min(steps, numel(t));
    finalTraj.t = [finalTraj.t, t(1:steps)];
    finalTraj.x = [finalTraj.x, x(1:steps)];
    finalTraj.y = [finalTraj.y, y(1:steps)];
    finalTraj.psi = [finalTraj.psi, psi(1:steps)];
    finalTraj.u = [finalTraj.u, u(1:steps)];
    finalTraj.v = [finalTraj.v, v(1:steps)];
    finalTraj.r = [finalTraj.r, r(1:steps)];
    finalTraj.X = [finalTraj.X, X(1:steps)];
    finalTraj.Y = [finalTraj.Y, Y(1:steps)];
    finalTraj.N = [finalTraj.N, N(1:steps)];
    finalTraj.Xc = [finalTraj.Xc, Xc(1:steps)];
    finalTraj.Yc = [finalTraj.Yc, Yc(1:steps)];
    finalTraj.Nc = [finalTraj.Nc, Nc(1:steps)];
end

% plot timeline for final trajectory
figure(3); clf
s(1) = subplot(3,4,1); hold on; plot(finalTraj.t,finalTraj.x); grid on; xlim([tmin, tmax]); ylabel('x in m');
s(2) = subplot(3,4,2); hold on; plot(finalTraj.t,finalTraj.u); grid on; xlim([tmin, tmax]); ylabel('u in m/s');
s(3) = subplot(3,4,3); hold on; plot(finalTraj.t,finalTraj.X); grid on; xlim([tmin, tmax]); ylabel('X in N');
s(4) = subplot(3,4,4); hold on; plot(finalTraj.t,finalTraj.Xc);  grid on; xlim([tmin, tmax]); ylabel('X_c in N');
s(5) = subplot(3,4,5); hold on; plot(finalTraj.t,finalTraj.y); grid on; xlim([tmin, tmax]); ylabel('y in m');
s(6) = subplot(3,4,6); hold on; plot(finalTraj.t,finalTraj.v); grid on; xlim([tmin, tmax]); ylabel('v in m/s');
s(7) = subplot(3,4,7); hold on; plot(finalTraj.t,finalTraj.Y); grid on; xlim([tmin, tmax]); ylabel('Y in N');
s(8) = subplot(3,4,8); hold on; plot(finalTraj.t,finalTraj.Yc);  grid on; xlim([tmin, tmax]); ylabel('Y_c in N');
s(9) = subplot(3,4,9); hold on; plot(finalTraj.t,finalTraj.psi); grid on; xlim([tmin, tmax]); ylabel('\psi in rad');
s(10) = subplot(3,4,10); hold on; plot(finalTraj.t,finalTraj.r); grid on; xlim([tmin, tmax]); ylabel('r in rad/s');
s(11) = subplot(3,4,11); hold on; plot(finalTraj.t,finalTraj.N); grid on; xlim([tmin, tmax]); ylabel('N in N');
s(12) = subplot(3,4,12); hold on; plot(finalTraj.t,finalTraj.Nc);  grid on; xlim([tmin, tmax]); ylabel('N_c in N');
linkaxes(s,'x');
