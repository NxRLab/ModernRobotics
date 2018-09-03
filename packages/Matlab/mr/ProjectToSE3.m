function T = ProjectToSE3(mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes mat: A matrix near SE(3) to project to SE(3).
% Returns T representing the closest rotation matrix that is in SE(3).
% This function uses singular-value decomposition (see
% http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review)
% and is only appropriate for matrices close to SE(3).
% Example Inputs:
% 
% clear; clc;
% mat = [ 0.675, 0.150,  0.720, 1.2;
%         0.370, 0.771, -0.511, 5.4;
%        -0.630, 0.619,  0.472, 3.6;
%         0.003, 0.002,  0.010, 0.9];
% T = ProjectToSE3(mat)
% 
% Output:
% T =
%     0.6790    0.1489    0.7189    1.2000
%     0.3732    0.7732   -0.5127    5.4000
%    -0.6322    0.6164    0.4694    3.6000
%          0         0         0    1.0000

T = RpToTrans(ProjectToSO3(mat(1: 3, 1: 3)), mat(1: 3, 4));
end