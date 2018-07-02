%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function d = DistanceToSE3(T)
% Takes mat: A 4x4 matrix.
% Returns d, a quantity describing the distance of T from the SE(3) 
% manifold.
% Compute the determinant of R, the top 3x3 submatrix of T. If 
% det(R) <= 0, return a large number. If det(R) > 0, replace the top 3x3 
% submatrix of T with R' * R, and set the first three entries of the 
% fourth column of T to zero. Then return norm(T - I).
% Example Inputs:
%{
  clear; clc;
  T = [1.0, 0.0,   0.0,  1.2;
       0.0, 0.1, -0.95,  1.5;
       0.0, 1.0,   0.1, -0.9;
       0.0, 0.0,   0.1, 0.98];
  d = DistanceToSE3(T)
%}
% Output:
% d =
%     0.0884

[R, p] = TransToRp(T);
if det(R) > 0
	d = norm([R' * R, [0; 0; 0]; T(4, :)] - eye(4), 'fro');
else
    d = 1e+9;
end
end