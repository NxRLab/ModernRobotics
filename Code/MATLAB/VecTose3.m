%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function se3mat = VecTose3(V)
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.
% Example Input:
%{
  clear;clc;
  V = [1,2,3,4,5,6];
  se3mat = VecTose3(V)
%} 
% Output:
% se3mat =
%     0    -3     2     4
%     3     0    -1     5
%    -2     1     0     6
%     0     0     0     0 
if length(V)==6
    so3mat=VecToso3(V(1:3));
    se3mat=[so3mat(1,1),so3mat(1,2),so3mat(1,3),V(4);so3mat(2,1),so3mat(2,2),so3mat(2,3),V(5);so3mat(3,1),so3mat(3,2),so3mat(3,3),V(6);0,0,0,0];
else
    msg = 'Input vector is the wrong size.';
    error(msg);
end
end

