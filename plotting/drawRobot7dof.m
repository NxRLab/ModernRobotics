function drawRobot7dof(x, param)
    %% animation
    close all
    n = 7;
    x_vec = x(1:param.ndof*6);
    x_exp = reshape(x_vec,6,[])'; % 
 
%     x_exp(6,4:6) = x_exp(5,4:6);
%     x_exp(7,4:6) = x_exp(5,4:6);

    h = 0;
    Slist = [];
    W = [];
    for i = 1:param.ndof
        wi = eul2rotm(x_exp(i,1:3)) * [0 0 1]';
        qi = x_exp(i,4:6)';
        Si = ScrewToAxis(qi,wi, h);
        Slist = [Slist Si];
        W = [W wi];
    end

    Base = eye(4);
    
    base_rot = eul2rotm(x(43:45));
    M = eye(4);
    M(1:3,1:3) = base_rot;
    M(1:3,4) = x(46:48);
    
    param.M = M;
    param.Slist = Slist;
    param.W = W;
    param.L = x_exp(:,4:6)';
    param.Base = Base;
    param.view = 3;
    
    nData = 100;
    t = linspace(0,10,nData);
    q0 = 2*rand(7,1)-1;
    T0 = FKinSpace(param.M, param.Slist, q0);


    Tn(1,4) = 0.25;
    Tn(2,4) = 0.25;
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
    hold on
%     plot3(fk_updated(1,:), fk_updated(2,:), fk_updated(3,:), 'r*')
%     plot3(fk_actual(1,:), fk_actual(2,:), fk_actual(3,:), 'k*')


    % draw7DofRobot(q, param)
    t = linspace(0,10, size(q,2));

    Anim.filename = '7doftest.gif';
    Anim.speed = 2;
    Anim.plotFunc = @(t,q)( drawExpRobot(q,param) );
    Anim.verbose = true;
    animateExp(t,q,Anim);
end