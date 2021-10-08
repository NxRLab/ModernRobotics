function drawDualRobot(q, Bases)
    close all;
    n = 7;

    param = KUKA_Model();

    nData = 100;
    t = linspace(0,10,nData);

    q0 = 2*rand(7,1)-1;
    T0 = FKinSpace(param.M, param.Slist, q0);
    Tn = eye(4);
    Tn(1,4) = 0.1;
    Tn(2,4) = 0.05;
    Tn(3,4) = 0.9;
    cartTraj = CartesianTrajectory(T0, Tn , 10, nData, 3); 
    q = [];
    fk_updated = zeros(3, nData);
    fk_actual  = zeros(3, nData);

    for i = 1:nData
        q_ = IKinSpace3D(param.Slist, param.M, cartTraj{i}, q0, 0.01, 0.01, param);
        % q_ =  projectJointSpace(q_, param);
        fk_ = FKinSpace(param.M, param.Slist, q_);
        fk_actual(:,i) = cartTraj{i}(1:3,4);
        fk_updated(:,i) = fk_(1:3,4);
        q = [q q_]; 
    end
    qDual = {q,q};
    T1 = eye(4); T2 = eye(4);
    T1(1,4) = 0.4;
    T2(1,4) = -0.4;
    baseDual = {T1, T2};
    paramDual = {param, param};

    Anim.filename = 'dual_dof.gif';
    Anim.speed = 2;
    Anim.plotFunc = @(t,q)( drawMultiRobots(qDual,baseDual, paramDual) );
    Anim.verbose = true;
    animateExp(t,q,Anim);

    hold on
    plot3(fk_updated(1,:), fk_updated(2,:), fk_updated(3,:), 'r*')
    plot3(fk_actual(1,:), fk_actual(2,:), fk_actual(3,:), 'k*')

    figure(2)
    for i = 1:7
       plot(q(i,:)) 
       hold on
    end
end