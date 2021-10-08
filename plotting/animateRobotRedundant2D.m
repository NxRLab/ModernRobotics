function animateRobotRedundant2D(q, Td, prob, param)
    %% animation
    Base = eye(4);
    Base(3,4) = 0;

    param.Base = Base;

    draw2DRobot(q, Td, prob, param)
end