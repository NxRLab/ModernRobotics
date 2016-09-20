%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function invR = RotInv(R)
% Takes a 3x3 rotation matrix and returns the transpose (inverse)
% Example Input:
%{ 
  clear;clc;
  R = [[0, 0,1]; [1, 0, 0]; [0, 1, 0]];
  invR = RotInv(R)
%} 
% Output:
% invR =
%     0     1     0
%     0     0     1
%     1     0     0
[m,n]=size(R);
if norm(R'*R-eye(n))<0.05 && norm(det(R)-1)<0.05
    invR=R';
else
    msg = 'Inverse cannot be calculated.';
    error(msg);
end
end