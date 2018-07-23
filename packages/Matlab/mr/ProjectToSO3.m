%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function R = ProjectToSO3(mat)
% Takes mat: A matrix near SO(3) to project to SO(3).
% Returns R representing the closest rotation matrix that is in SO(3).
% This function uses singular-value decomposition and is only compatible 
% with matrices close to SO(3).
% Example Inputs:
%{
  clear; clc;
  mat = [ 0.675, 0.150,  0.720;
          0.370, 0.771, -0.511;
         -0.630, 0.619,  0.472];
  R = ProjectToSO3(mat)
%}
% Output:
% R =
%    0.6790    0.1489    0.7189
%    0.3732    0.7732   -0.5127
%   -0.6322    0.6164    0.4694

[U, S, V] = svd(mat);
R = U * V';
if det(R) < 0
    R = [R(:, 1: 2); -R(:, 3)];
end
end