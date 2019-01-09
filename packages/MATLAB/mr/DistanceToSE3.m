function d = DistanceToSE3(mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes mat: A 4x4 matrix.
% Returns the Frobenius norm to describe the distance of mat from the SE(3) 
% manifold.
% Compute the determinant of matR, the top 3x3 submatrix of mat. If 
% det(matR) <= 0, return a large number. If det(matR) > 0, replace the top 
% 3x3 submatrix of mat with matR' * matR, and set the first three entries 
% of the fourth column of mat to zero. Then return norm(mat - I).
% Example Inputs:
% 
% clear; clc;
% mat = [1.0, 0.0,   0.0,  1.2;
%        0.0, 0.1, -0.95,  1.5;
%        0.0, 1.0,   0.1, -0.9;
%        0.0, 0.0,   0.1, 0.98];
% d = DistanceToSE3(mat)
% 
% Output:
% d =
%     0.1349

[R, p] = TransToRp(mat);
if det(R) > 0
	d = norm([R' * R, [0; 0; 0]; mat(4, :)] - eye(4), 'fro');
else
    d = 1e+9;
end
end