function [M, Slist, W, L] = robotEXP(x)

    %% manipulator twists
    ndofs = numel(x)/2 - 1;
    h = 0;
    W = [];
    Slist = [];
    L = [];
    for i = 1:ndofs
        wi = [0;0;1]; qi = [x(2*i-1);x(2*i);0];
        W = [W wi];
        L = [L qi];
        
        Si = ScrewToAxis(qi,wi, h);
        Slist = [Slist Si];
    end
    
    M = eye(4);
    M(1,4) = x(end-1);
    M(2,4) = x(end);

    
end


