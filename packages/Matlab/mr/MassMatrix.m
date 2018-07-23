function M = MassMatrix(thetalist, Mlist, Glist, Slist)
% *** CHAPTER 8: DYNAMICS OF OPEN CHAINS ***
% Takes thetalist: A list of joint variables,
%       Mlist: List of link frames i relative to i-1 at the home position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.
% Returns M: The numerical inertia matrix M(thetalist) of an n-joint serial
%            chain at the given configuration thetalist.
% This function calls InverseDynamics n times, each time passing a 
% ddthetalist vector with a single element equal to one and all other 
% inputs set to zero. Each call of InverseDynamics generates a single 
% column, and these columns are assembled to create the inertia matrix.
% Example Input (3 Link Robot):
% 
% clear; clc;
% thetalist = [0.1; 0.1; 0.1];
% M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.089159]; [0, 0, 0, 1]];
% M12 = [[0, 0, 1, 0.28]; [0, 1, 0, 0.13585]; [-1, 0 ,0, 0]; [0, 0, 0, 1]];
% M23 = [[1, 0, 0, 0]; [0, 1, 0, -0.1197]; [0, 0, 1, 0.395]; [0, 0, 0, 1]];
% M34 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.14225]; [0, 0, 0, 1]];
% G1 = diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7]);
% G2 = diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393]);
% G3 = diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275]);
% Glist = cat(3, G1, G2, G3);
% Mlist = cat(3, M01, M12, M23, M34);  
% Slist = [[1; 0; 1;      0; 1;     0], ...
%        [0; 1; 0; -0.089; 0;     0], ...
%        [0; 1; 0; -0.089; 0; 0.425]];
% M = MassMatrix(thetalist, Mlist, Glist, Slist)
% 
% Output:
% M =
%   22.5433   -0.3071   -0.0072
%   -0.3071    1.9685    0.4322
%   -0.0072    0.4322    0.1916

n = size(thetalist, 1);
M = zeros(n);
for i = 1: n
   ddthetalist = zeros(n, 1);
   ddthetalist(i) = 1;
   M(:, i) = InverseDynamics(thetalist, zeros(n, 1), ddthetalist, ...
                             [0; 0; 0], [0; 0; 0; 0; 0; 0],Mlist, ...
                             Glist, Slist);
end
end