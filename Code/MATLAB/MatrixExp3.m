%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function  R = MatrixExp3(expc3)
% Takes a 3-vector of exponential coordinates
% Returns R (SO(3)) that is achieved by rotating about omghat by theta 
% from an initial orientation R = I
% Rodriguez R = I + sin(theta)*omghat + (1-cos(theta))*omghat^2
% Example Input:
%{
  clear;clc;
  expc3 = [1,2,3];
  R = MatrixExp3(expc3)  
%} 
% Output:
% R =
%   -0.6949    0.7135    0.0893
%   -0.1920   -0.3038    0.9332
%    0.6930    0.6313    0.3481
if length(expc3)==3
    if norm(expc3)<0.001
        R=eye(3);
    else
    [omghat,theta]=AxisAng3(expc3);
    R=eye(3)+sin(theta)*VecToso3(omghat)+(1-cos(theta))*VecToso3(omghat)*VecToso3(omghat);
    end
else
    msg = 'Input vector is the wrong size.';
    error(msg);
end
end

