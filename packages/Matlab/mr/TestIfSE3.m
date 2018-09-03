function judge = TestIfSE3(mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes mat: A 4x4 matrix.
% Check if mat is close to or on the manifold SE(3).
% Example Inputs:
% 
% clear; clc;
% mat = [1.0, 0.0,   0.0,  1.2;
%        0.0, 0.1, -0.95,  1.5;
%        0.0, 1.0,   0.1, -0.9;
%        0.0, 0.0,   0.1, 0.98];
% judge = TestIfSE3(mat)
% 
% Output:
% judge =
%     0

judge = norm(DistanceToSE3(mat)) < 1e-3;
end