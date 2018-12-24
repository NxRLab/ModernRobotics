function c = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
% *** CHAPTER 8: DYNAMICS OF OPEN CHAINS ***
% Takes thetalist: A list of joint variables,
%       dthetalist: A list of joint rates,
%       Mlist: List of link frames i relative to i-1 at the home position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns,
% Returns c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
%            terms for a given thetalist and dthetalist.
% This function calls InverseDynamics with g = 0, Ftip = 0, and 
% ddthetalist = 0.
% Example Input (3 Link Robot):
% 
% clear; clc;
% thetalist = [0.1; 0.1; 0.1];
% dthetalist = [0.1; 0.2; 0.3];
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
% c = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
% 
% Output:
% c =
%    0.2645
%   -0.0551
%   -0.0069

c = InverseDynamics(thetalist, dthetalist, ...
                    zeros(size(thetalist, 1), 1), [0; 0; 0], ...
                    [0; 0; 0; 0; 0; 0], Mlist, Glist, Slist);
end