%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function T = RpToTrans(R,p)
% Takes rotation matrix R and position p
% Returns corresponding homogeneous transformation matrix T SE(3)
% Example Input:
%{
  clear;clc;
  R = [[1, 0, 0]; [0, 0, -1]; [0, 1, 0]];
  p = [1,2,5];
  T = RpToTrans(R,p)
%} 
% Output:
% T =
%     1     0     0     1
%     0     0    -1     2
%     0     1     0     5
%     0     0     0     1
[m,n]=size(R);
if m==3 && n==3 && length(p)==3
    if norm(R'*R-eye(3))<0.05 && norm(det(R)-1)<0.05
        T=[R(1,1),R(1,2),R(1,3),p(1);R(2,1),R(2,2),R(2,3),p(2);R(3,1),R(3,2),R(3,3),p(3);0,0,0,1];
    else
        msg = 'Input R is not a rotation matrix.';
        error(msg);
    end
else
    msg = 'Input rotation matrix or position vector are the wrong size.';
    error(msg);
end
end

