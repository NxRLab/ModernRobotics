function d = DistanceToSO3(mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes mat: A 3x3 matrix.
% Returns the Frobenius norm to describe the distance of mat from the SO(3) 
% manifold.
% Computes the distance from R to the SO(3) manifold using the following 
% method:
% If det(mat) <= 0, return a large number. 
% If det(mat) > 0, return norm(mat' * mat - I).
% Example Inputs:
% 
% clear; clc;
% mat = [1.0, 0.0,   0.0;
%        0.0, 0.1, -0.95;
%        0.0, 1.0,   0.1];
% d = DistanceToSO3(mat)
% 
% Output:
% d =
%     0.0884

if det(mat) > 0
	d = norm(mat' * mat - eye(3), 'fro');
else
    d = 1e+9;
end
end