%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function V = se3ToVec(se3mat)
% Takes se3mat a 4x4 se(3) matrix
% Returns the corresponding 6-vector (representing spatial velocity)
% Example Input:
%{
  clear;clc;
  se3mat = [[0, -3, 2, 4]; [3, 0, -1, 5]; [-2, 1, 0, 6]; [0, 0, 0, 1]];
  V = se3ToVec(se3mat)
%} 
% Output:
% V =
%     1
%     2
%     3
%     4
%     5
%     6
[m,n]=size(se3mat);
if m==4 && n==4
    omg=se3mat(1:3,1:3);
    omg2=so3ToVec(omg);
    V=[omg2;se3mat(1:3,4)];
else
    msg = 'Input matrix is the wrong size.';
    error(msg);
end
end

