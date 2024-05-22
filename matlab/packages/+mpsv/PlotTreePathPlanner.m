function PlotTreePathPlanner(tree)
    %mpsv.PlotTreePathPlanner Plot the tree from the path planner.
    % 
    % PARAMETER
    % tree ... Tree from path planner.
    activeIndices = (tree.activeIndices + int16(1));
    plot(tree.pose(1,activeIndices),tree.pose(2,activeIndices),'Marker','.','Color',[0.8 0.8 0.8],'LineStyle','none');
    N = numel(activeIndices);
    stateHold = ishold();
    hold on;
    for k = 1:N
        idx = activeIndices(k);
        if(tree.idxParent(idx) < int16(0)), continue; end
        pointParent = tree.pose(1:2,tree.idxParent(idx) + 1);
        pointThis = tree.pose(1:2,idx);
        plot([pointParent(1), pointThis(1)],[pointParent(2), pointThis(2)],'Color',[0.8 0.8 0.8]);
    end
    if(~stateHold), hold off; end
end

