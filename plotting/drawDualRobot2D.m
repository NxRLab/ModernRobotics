function drawDualRobot2D()
    close all;
    n = 5;

    param = Planer5DOF();
    object.position = [-0.4;1.5];
    object.orientation = 0;
    [t, qDual, base, objectTrajectory] = dualRobotManipulationTrajectory(object);
    
    baseDual = {zeros(6,1), zeros(6,1)};
    baseDual{1}(1) = -0.4;
    baseDual{2}(1) = 0.4;
    baseDual{1}(4:6) = rotm2eul(base{1}(1:3,1:3));
    baseDual{2}(4:6) = rotm2eul(base{2}(1:3,1:3));

    param.view = 2;

    paramDual = {param, param, object};
    
    Anim.filename = 'dual_dof.gif';
    Anim.speed = 2;
    Anim.plotFunc = @(t,q,base,object)( drawMultiRobots(q, base, object, paramDual) );
    Anim.verbose = true;
    animateMultiRobotExp(t,qDual,baseDual,objectTrajectory,Anim);

    %     hold on
    %     plot3(fk_updated(1,:), fk_updated(2,:), fk_updated(3,:), 'r*')
    %     plot3(fk_actual(1,:), fk_actual(2,:), fk_actual(3,:), 'k*')
    % 
    %     figure(2)
    %     for i = 1:5
    %        plot(q(i,:)) 
    %        hold on
    %     end
end