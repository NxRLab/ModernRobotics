%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function [S,theta] = AxisAng6(expc6)
% Takes a 6-vector of exponential coordinates for rigid-body motion S*theta. 
% Returns S: the corresponding normalized screw axis,
% theta: the distance traveled along/about S.
% Example Input:
%{
  clear;clc;
  expc6 = [1,0,0,1,2,3];
  [S,theta] = AxisAng6(expc6)
%} 
% Output:
% S =
%     1     0     0     1     2     3
% theta =
%     1
if length(expc6)==6
    if norm(Magnitude(expc6(1:3)))>0.01
        theta=Magnitude(expc6(1:3));
        S=expc6/theta;
    else
        if norm(Magnitude(expc6(4:6)))>0.01
        theta=Magnitude(expc6(4:6));
        S=expc6/theta;
        else
            theta=0;
            S=[0,0,0,0,0,0];    
        end
    end        
else
    msg = 'Input vector is the wrong size.';
    error(msg);
end
end

