%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function S = ScrewToAxis(q,s,h)
% Takes q: a point lying on the screw axis,
% s: a unit vector in the direction of the screw axis, 
% h: the pitch of the screw axis.
%
% Returns the corresponding normalized screw axis
% Example Input:
%{
  clear;clc;
  q = [3,0,0];
  s = [0,0,1];
  h = 2;
  S = ScrewToAxis(q,s,h)
%} 
% Output:
% S =
%     0
%     0
%     1
%     0
%    -3
%     2
if length(q)==3 && length(s)==3 && norm(Magnitude(s)-1)<0.01 
    v=cross(q,s)+h*s;
    S=[s(1);s(2);s(3);v(1);v(2);v(3)];
    if norm(Magnitude(s))>0.01
        S=S/Magnitude(s);
    else
        S=S/Magnitude(v);
    end
else
    msg = 'Inputs are the wrong size. -->  qE3, sE3, h scaler.';
    error(msg);
end
end

