%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function judge = TestIfSO3(R)
% Takes mat: A 3x3 matrix R.
% Check if R is close to or on the manifold SO(3).
% Example Inputs:
%{
  clear; clc;
  R = [1.0, 0.0,   0.0;
       0.0, 0.1, -0.95;
       0.0, 1.0,   0.1];
  judge = TestIfSO3(R)
%}
% Output:
% dudge =
%     0

judge = NearZero(DistanceToSO3(R));
end