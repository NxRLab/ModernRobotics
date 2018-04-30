%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function se3mat = VecTose3(V)
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.
% Example Input:
%{
  clear; clc;
  V = [1; 2; 3; 4; 5; 6];
  se3mat = VecTose3(V)
%} 
% Output:
% se3mat =
%     0    -3     2     4
%     3     0    -1     5
%    -2     1     0     6
%     0     0     0     0 

se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
end