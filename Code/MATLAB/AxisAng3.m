%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function [omghat,theta] = AxisAng3(expc3)
% Takes A 3-vector of exponential coordinates for rotation and
% returns unit rotation axis omghat and the corresponding rotation angle theta
% Example Input:
%{
  clear;clc;
  expc3 = [1,2,3];
  [omghat,theta] = AxisAng3(expc3)  
%}
% Output:
% omghat =
%    0.2673    0.5345    0.8018
% theta =
%    3.7417
if length(expc3)==3
    theta=Magnitude(expc3);
    omghat=Normalize(expc3);
else
    msg = 'Input vector is the wrong size.';
    error(msg);
end
end

