%**********************************************************************************************
%****************************  CHAPTER 9: TRAJECTORY GENERATION  ******************************
%**********************************************************************************************

function traj = CartesianTrajectory(Xstart,Xend,Tf,N,method)
% Takes Xstart: The initial end-effector configuration,
% Xend: The final end-effector configuration,
% Tf: Total time of the motion in seconds from rest to rest,
% N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory,
% method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling
% and 5 indicates quintic (fifth-order polynomial) time scaling.
%
% Returns traj: The discretized trajectory as a list of N matrices in SE(3) separated in 
% time by Tf/(N-1). The first in the list is Xstart and the Nth is Xend .
% This function is Similar to ScrewTrajectory, except the origin of the end-effector frame
% follows a straight line, decoupled from the rotational motion.
% Animation example can be seen at https://www.youtube.com/watch?v=ycaGRk_0AE8
% 
% Example Input:
%{ 
  clear;clc;
  Xstart = [[1,0,0,1];[0,1,0,0];[0,0,1,1];[0,0,0,1]];
  Xend = [[0,0,1,0.1];[1,0,0,0];[0,1,0,4.1];[0,0,0,1]];
  Tf = 5;
  N = 4;
  method = 5;
  traj = CartesianTrajectory(Xstart,Xend,Tf,N,method)
%}
% Output:
% traj =
%    1.0000         0         0    1.0000
%         0    1.0000         0         0
%         0         0    1.0000    1.0000
%         0         0         0    1.0000
%    0.9366   -0.2140    0.2774    0.8111
%    0.2774    0.9366   -0.2140         0
%   -0.2140    0.2774    0.9366    1.6506
%         0         0         0    1.0000
%    0.2774   -0.2140    0.9366    0.2889
%    0.9366    0.2774   -0.2140         0
%   -0.2140    0.9366    0.2774    3.4494
%         0         0         0    1.0000
%   -0.0000    0.0000    1.0000    0.1000
%    1.0000   -0.0000    0.0000         0
%    0.0000    1.0000   -0.0000    4.1000
%         0         0         0    1.0000
traj=[];
timegap = Tf/(N-1);
[Rstart,Pstart]=TransToRp(Xstart);
[Rend,Pend]=TransToRp(Xend);
if size(Xstart,1)==4 && size(Xend,1)==4 && size(Xstart,2)==4 && size(Xend,2)==4 && N>1 && (method == 3 || method == 5)
    for j=1:N
        if method == 3
            s = CubicTimeScaling(Tf,timegap*(j-1));
        else
            s = QuinticTimeScaling(Tf,timegap*(j-1));
        end
        Rcurrent=Rstart*MatrixExp3(MatrixLog3(RotInv(Rstart)*Rend)*s);
        Pcurrent=Pstart+s*(Pend-Pstart);
        tr=[Rcurrent,Pcurrent;0,0,0,1];
        traj=[traj;tr];
    end
else
    msg = 'Input is not appropriate.';
    error(msg);
end
end


