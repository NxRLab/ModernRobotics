%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function so3mat = VecToso3(omg)
% Takes a 3-vector(angular velocity) and returns the skew symmetric matrix in so3
% Example Input:
%{
  clear;clc;
  omg = [1,2,3];
  so3mat = VecToso3(omg)
%}
% Output:
% so3mat =
%     0    -3     2
%     3     0    -1
%    -2     1     0

if length(omg)==3
    so3mat=[0,-1*omg(3),omg(2);omg(3),0,-1*omg(1);-1*omg(2),omg(1),0];
else
    msg = 'Input vector is the wrong size.';
    error(msg);
end
end
