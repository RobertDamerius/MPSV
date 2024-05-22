function PlotTreeMotionPlanner(tree)
    %mpsv.PlotTreeMotionPlanner Plot the tree from the motion planner.
    % 
    % PARAMETER
    % tree ... Tree from motion planner.
    activeIndices = (tree.activeIndices + int16(1));
    N = numel(activeIndices);
    stateHold = ishold();
    hold on;
    for k = 1:N
        idx = activeIndices(k);
        if(tree.idxParent(idx) < int16(0)), continue; end
        x = tree.trajectoryFromRoot{idx}(1,:);
        y = tree.trajectoryFromRoot{idx}(2,:);
        plot(x,y,'Color',[0.8 0.1 0.4]);
        plot(x(end),y(end),'Color',[0.8 0.1 0.4],'Marker','.');
    end
    if(~stateHold), hold off; end
end

