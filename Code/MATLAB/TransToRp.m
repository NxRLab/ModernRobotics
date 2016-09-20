%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function  [R,p]= TransToRp(T)
% Take transformation matrix T SE(3) 
% Returns R the corresponding rotation matrix,
% and p the corresponding position vector
% Example Input:
%{
  clear;clc;
  T = [[1,0,0,0]; [0,0,-1,0]; [0,1,0,3]; [0,0,0,1]];
  [R,p]= TransToRp(T)
%}
% Output:
% R =
%     1     0     0
%     0     0    -1
%     0     1     0
% p =
%     0
%     0
%     3
rt=T(1:3,1:3);
pt=T(1:3,4);
[m,n]=size(T);
if m==4 && n==4
    if norm(rt'*rt-eye(3))<0.01 && norm(det(rt)-1)<0.01
        R=rt;
        p=pt;
    else
        msg = 'Input is not a transformation matrix.';
        error(msg);
    end
else
    msg = 'Input Transformation matrix is the wrong size.';
    error(msg);
end
end

