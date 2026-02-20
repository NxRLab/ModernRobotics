function param = YUMI_Model()

    n = 7;

    W = [0  0 0  0 1  0  1; ...
         0 -1 0 -1 0 -1 0; ...
         1  0 1  0 0  0  0];
     
    L = [0   0 0   40.5   40.5  305.5 341.5;...
         0 -30 0   0      0     0     0;...
         0 166 166 417.5  458   431   458]/1000;

    h = 0;
    S = [];
    for i = 1:n
        Si = ScrewToAxis(L(:,i), W(:,i), h);
        S = [S Si];
    end

    Base = eye(4);

    param.Base = Base;
    param.W = W;
    param.L = L;
    param.Slist = S;
    param.M = [0  0 1 341.5/1000;...
               0  1 0 0;...
               -1 0 0 458/1000;...
               0 0 0 1];

    param.view = 3;
    param.qmin = deg2rad([-168.5 -143.5 -123.5 -290 -88 -229 -168.5]');
    param.qmax = deg2rad([168.5 43.5 80 290 138 229 168.5]');
    param.ndofs = 7;
end