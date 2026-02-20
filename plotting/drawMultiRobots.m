function drawMultiRobots(q, base, manipulationObject, p)
    n_robots = numel(q);
    for i = 1:n_robots
        drawExpBaseRobot(q{i},base{i},p{i})
    end
    
    drawManipulationObject(manipulationObject)
    
    xHR = [-0.5 0.5 0.5 -0.5];
    yHR = [0.25 0.25 -0.25 -0.25];
    
    ax = gca;
    patch(ax,xHR,yHR,'black')
    axis off
end
