function traj = ScrewTrajectory(Xstart, Xend, Tf, N, method)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes Xstart: The initial end-effector configuration,
%       Xend: The final end-effector configuration,
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns traj: The discretized trajectory as a list of N matrices in SE(3)
%               separated in time by Tf/(N-1). The first in the list is 
%               Xstart and the Nth is Xend .
% This function calculates a trajectory corresponding to the screw motion 
% about a space screw axis.
% Example Input:
% 
% clear; clc;
% Xstart = [[1 ,0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
% Xend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
% Tf = 5;
% N = 4;
% method = 3;
% traj = ScrewTrajectory(Xstart, Xend, Tf, N, method)
% 
% Output:
% traj =
%    1.0000         0         0    1.0000
%         0    1.0000         0         0
%         0         0    1.0000    1.0000
%         0         0         0    1.0000
%
%    0.9041   -0.2504    0.3463    0.4410
%    0.3463    0.9041   -0.2504    0.5287
%   -0.2504    0.3463    0.9041    1.6007
%         0         0         0    1.0000
%
%    0.3463   -0.2504    0.9041   -0.1171
%    0.9041    0.3463   -0.2504    0.4727
%   -0.2504    0.9041    0.3463    3.2740
%         0         0         0    1.0000
%
%   -0.0000    0.0000    1.0000    0.1000
%    1.0000   -0.0000    0.0000   -0.0000
%    0.0000    1.0000   -0.0000    4.1000
%         0         0         0    1.0000

timegap = Tf / (N - 1);
traj = cell(1, N);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    traj{i} = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
end
end

