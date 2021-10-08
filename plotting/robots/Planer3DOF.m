function param = Planer3DOF()

    xvec = [0. 0. 0.5  0.0  1.0  0.0  1.5  0];

    % robot definition
    [M, Slist, W, L] = robotEXP(xvec);
    param.M = M;
    param.Slist = Slist;
    param.W = W;
    param.L = L;
    param.Base = eye(4);
    param.qmin = 1.5*[-(1/6)*pi -pi/2 -pi/2]';
    param.qmax = 1.5*[(1/6)*pi pi/2 pi/2]'; 
    param.ndofs = 3;

end