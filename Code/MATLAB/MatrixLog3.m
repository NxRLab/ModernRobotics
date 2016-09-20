%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function expc3 = MatrixLog3(R)
% Takes R (rotation matrix)
% Returns the corresponding 3-vector of exponential coordinates (expc3 = omghat*theta)
% Example Input:
%{
  clear;clc;
  R = [[0, 0,1];[1, 0, 0];[0, 1, 0]];
  expc3 = MatrixLog3(R)
%} 
% Output:
% expc3 =
%    1.2092
%    1.2092
%    1.2092
[m,n]=size(R);
if norm(R'*R-eye(n))<0.01 && norm(det(R)-1)<0.01 && m==3 && n==3
    if norm(R-eye(3))<0.01
        expc3=[0;0;0];
    elseif norm(R(1,1)+R(2,2)+R(3,3)+1)<0.0001
        theta=pi;
        if (1+R(3,3))>0.01
            omg=(1/(2*(1+R(3,3)))^0.5)*[R(1,3);R(2,3);1+R(3,3)];
        elseif (1+R(2,2))>0.01
            omg=(1/(2*(1+R(2,2)))^0.5)*[R(1,2);1+R(2,2);R(3,2)];
        elseif (1+R(1,1))>0.01
            omg=(1/(2*(1+R(1,1)))^0.5)*[1+R(1,1);R(2,1);R(3,1)];
        end
        expc3=theta*omg;
    else
        theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
        omg=(1/(2*sin(theta)))*(R-R');
        omg2=so3ToVec(omg);
        expc3=theta*omg2;
    end
else
    msg = 'Input matrix is the wrong size.';
    error(msg);
end

end

