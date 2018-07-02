%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function judge = TestIfSE3(T)
% Takes mat: A 4x4 matrix T.
% Check if T is close to or on the manifold SE(3).
% Example Inputs:
%{
  clear; clc;
  T = [1.0, 0.0,   0.0,  1.2;
       0.0, 0.1, -0.95,  1.5;
       0.0, 1.0,   0.1, -0.9;
       0.0, 0.0,   0.1, 0.98];
  judge = TestIfSE3(T)
%}
% Output:
% dudge =
%     0

judge = NearZero(DistanceToSE3(T));
end