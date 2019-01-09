function traj = JointTrajectory(thetastart, thetaend, Tf, N, method)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes thetastart: The initial joint variables,
%       thetaend: The final joint variables,
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns traj: A trajectory as an N x n matrix, where each row is an
%               n-vector of joint variables at an instant in time. The 
%               first row is thetastart and the Nth row is thetaend . The 
%               elapsed time between each row is Tf/(N - 1).
% The returned trajectory is a straight-line motion in joint space.
% Example Input:
% 
% clear; clc;
% thetastart = [1; 0; 0; 1; 1; 0.2; 0; 1];
% thetaend = [1.2; 0.5; 0.6; 1.1; 2;2; 0.9; 1];
% Tf = 4;
% N = 6;
% method = 3;
% traj = JointTrajectory(thetastart, thetaend, Tf, N, method)
% 
% Output:
% traj =
%   1.0000        0        0   1.0000   1.0000   0.2000        0   1.0000
%   1.0208   0.0520   0.0624   1.0104   1.1040   0.3872   0.0936   1.0000
%   1.0704   0.1760   0.2112   1.0352   1.3520   0.8336   0.3168   1.0000
%   1.1296   0.3240   0.3888   1.0648   1.6480   1.3664   0.5832   1.0000
%   1.1792   0.4480   0.5376   1.0896   1.8960   1.8128   0.8064   1.0000
%   1.2000   0.5000   0.6000   1.1000   2.0000   2.0000   0.9000   1.0000

timegap = Tf / (N - 1);
traj = zeros(size(thetastart, 1), N);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    traj(:, i) = thetastart + s * (thetaend - thetastart);
end
traj = traj';
end