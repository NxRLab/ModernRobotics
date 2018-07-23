%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function d = DistanceToSO3(R)
% Takes mat: A 3x3 matrix.
% Returns d, a quantity describing the distance of R from the SO(3) 
% manifold.
% Computes the distance from R to the SO(3) manifold using the following 
% method:
% If det(R) <= 0, return a large number. 
% If det(R) > 0, return norm(R' * R - I).
% Example Inputs:
%{
  clear; clc;
  R = [1.0, 0.0,   0.0;
       0.0, 0.1, -0.95;
       0.0, 1.0,   0.1];
  d = DistanceToSO3(R)
%}
% Output:
% d =
%     0.0884

if det(R) > 0
	d = norm(R' * R - eye(3), 'fro');
else
    d = 1e+9;
end
end