%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function invT = TransInv(T)
% Takes T a transformation matrix 
% Returns its inverse. 
% Uses the structure of transformation matrices to avoid taking a matrix inverse, for efficiency.
% Example Input:
%{
  clear;clc;
  T = [[1,0,0,0]; [0,0,-1,0];  [0,1,0,3]; [0,0,0,1]];
  invT = TransInv(T)
%}
% Ouput:
% invT =
%     1     0     0     0
%     0     0     1    -3
%     0    -1     0     0
%     0     0     0     1
[m,n]=size(T);
if m==4 && n==4
    [R,p]=TransToRp(T);
    invT=[R',-1*R'*p;0,0,0,1];
else
    msg = 'Input matrix is the wrong size.';
    error(msg);
end
end

