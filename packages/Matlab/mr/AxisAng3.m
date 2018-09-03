function [omghat, theta] = AxisAng3(expc3)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes A 3-vector of exponential coordinates for rotation.
% Returns the unit rotation axis omghat and the corresponding rotation 
% angle theta.
% Example Input:
% 
% clear; clc;
% expc3 = [1; 2; 3];
% [omghat, theta] = AxisAng3(expc3)  
% 
% Output:
% omghat =
%    0.2673
%    0.5345
%    0.8018
% theta =
%    3.7417

theta = norm(expc3);
omghat = expc3 / theta;
end