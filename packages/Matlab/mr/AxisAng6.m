function [S, theta] = AxisAng6(expc6)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 6-vector of exponential coordinates for rigid-body motion
% S*theta. 
% Returns S: the corresponding normalized screw axis,
%         theta: the distance traveled along/about S.
% Example Input:
% 
% clear; clc;
% expc6 = [1; 0; 0; 1; 2; 3];
% [S, theta] = AxisAng6(expc6)
%  
% Output:
% S =
%     1
%     0
%     0 
%     1
%     2
%     3
% theta =
%     1

theta = norm(expc6(1: 3));
if NearZero(theta)
    theta = norm(expc6(4: 6));
end
S = expc6 / theta;      
end