%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function T = MatrixExp6(expc6)
% Takes a 6-vector of exponential coordinates (S*theta) 
% Returns a T matrix SE(3) that is achieved by traveling along/about the screw axis S 
% for a distance theta from an initial configuration T = I
% Rodriguez R = I + sin(theta)*omg + (1-cos(theta))*omg^2
% Example Input:
%{
  clear;clc;
  expc6 = [1.5707963267948966, 0.0, 0.0, 0.0, 2.3561944901923448,2.3561944901923457];
  T = MatrixExp6(expc6)
%}
% Output:
% T =
%    1.0000         0         0         0
%         0    0.0000   -1.0000   -0.0000
%         0    1.0000    0.0000    3.0000
%         0         0         0    1.0000

if length(expc6)==6
    m=size(expc6);
    if m(1)==1 && m(2)==6
        expc6=expc6';
    end
    omg=expc6(1:3);
    v=expc6(4:6);
    if norm(omg)>10^-5
        [omg2,theta]=AxisAng3(omg);
        UL=eye(3)+sin(theta)*VecToso3(omg2)+(1-cos(theta))*VecToso3(omg2)*VecToso3(omg2);
        v2=v/theta;
        UR=(eye(3)*theta+(1-cos(theta))*VecToso3(omg2)+(theta-sin(theta))*VecToso3(omg2)*VecToso3(omg2))*v2;
        T=[UL,UR;0,0,0,1];
    else
        T=[1,0,0,expc6(4);0,1,0,expc6(5);0,0,1,expc6(6);0,0,0,1];
    end
else
    msg = 'Input is not appropriate.';
    error(msg);
end
end

