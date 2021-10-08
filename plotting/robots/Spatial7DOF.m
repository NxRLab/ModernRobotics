function param = Spatial7DOF()
 
    W = [0 0 0 0 0 0 0; 0 -1 0 1 0 -1 0;1 0 1 0 1 0 1];
    L = [0 0 0 0 0 0 0;0 0 0 0 0 0 0;0.3600 0.3600 0.3600 0.7800 0.7800 1.1800 1.4210];

    h = 0;
    S = [];
    for i = 1:7
        Si = ScrewToAxis(L(:,i), W(:,i), h);
        S = [S Si];
    end
    
    Base = eye(4);

    param.Base = Base;
    param.W = W;
    param.L = L;
    param.Slist = S;
    param.M = [1 0 0 0;0 1 0 0;0 0 1 1.421;0 0 0 1];
    param.view = 3;
    param.qmin = [-(3.9/4)*pi -(3/4)*pi -(3.9/4)*pi -(3/4)*pi -(3.9/4)*pi -(3/4)*pi -(3.9/4)*pi]';
    param.qmax = [(3.9/4)*pi (3/4)*pi (3.9/4)*pi (3/4)*pi (3.9/4)*pi (3/4)*pi (3.9/4)*pi]';
    param.ndofs = 7;
    
end