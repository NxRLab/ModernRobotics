%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function omg = so3ToVec(so3mat)
% Takes a 3x3 skew-symmetric matrix (an element of so(3)) and returns the corresponding vector (angular velocity)
% Example Input: 
%{
  clear;clc;
  so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
  omg = so3ToVec(so3mat)  
%}
% Output:
% omg =
%     1
%     2
%     3

[m,n]=size(so3mat);
if m==3 && n==3 && norm(so3mat(1,1))<10^-3 && norm(so3mat(2,2))<10^-3 && norm(so3mat(3,3))<10^-3 && norm(so3mat(1,2)+so3mat(2,1))<10^-3     && norm(so3mat(1,3)+so3mat(3,1))<10^-3 && norm(so3mat(3,2)+so3mat(2,3))<10^-3
    omg=[so3mat(3,2);so3mat(1,3);so3mat(2,1)];
else
    msg = 'Input is not a skew-symmetric matrix.';
    error(msg);
end
end

