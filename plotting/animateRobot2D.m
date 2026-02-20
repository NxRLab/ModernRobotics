function animateRobot2D(x, q, param)
    %% animation
    n = 3;
    W = [0 0 0;0 0 0;1 1 1];
    param.W = W;
    param.L = reshape(x(1:6), [], 3)';
    param.L = [param.L zeros(3,1)];
    param.L = param.L';

    Base = eye(4);

    param.Base = Base;

    h = 0;
    S = [];
    for i = 1:n
        Si = ScrewToAxis(param.L(:,i), param.W(:,i), h);
        S = [S Si];
    end
    param.Slist = S;

    param.M = [1 0 0 x(7);0 1 0 x(8);0 0 1 0;0 0 0 1];
    param.ndofs = n;
    param.view = 2;
    
    draw2DRobot(q, param)
end