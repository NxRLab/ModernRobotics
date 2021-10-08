function animateRobot7dof(qStore, param)
    %% animation
    n = 7;
    W = [0 0 0 0 0 0 0; 0 -1 0 1 0 -1 0;1 0 1 0 1 0 1];
    L = [0 0 0 0 0 0 0;0 0 0 0 0 0 0;0.3600 0.3600 0.3600 0.7800 0.7800 1.1800 1.4210];

    h = 0;
    S = [];
    for i = 1:7
        Si = ScrewToAxis(L(:,i), W(:,i), h);
        S = [S Si];
    end

    b2 = eye(4);
    b2(2,4) = -0.3; b2(1,4) = 0.1;

    Base = eye(4);

    param.Base = Base;
    param.W = W;
    param.L = L;
    param.Slist = S;
    param.M = [1 0 0 0;0 1 0 0;0 0 1 1.421;0 0 0 1];
    param.view = 3;
    param.qmin = (2/3)*[-pi -pi -pi -pi -pi -pi -pi]';
    param.qmax = (2/3)*[pi pi pi pi pi pi pi]';
    

    % draw7DofRobot(q, param)
    t = linspace(0,10, size(qStore,2));

    Anim.filename = 'kukaTestTrajectories.gif';
    Anim.speed = 2;
    Anim.plotFunc = @(t,q)( drawExpRobot(q,param) );
    Anim.verbose = true;
    animateExp(t,qStore,Anim);
end