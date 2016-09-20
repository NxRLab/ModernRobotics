%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function expc6 = MatrixLog6(T)
% Takes a transformation matrix T SE(3)
% Returns the corresponding 6-vector of exponential coordinates S*theta
% Example Input:
%{
  clear;clc;
  T = [[1,0,0,0]; [0,0,-1,0]; [0,1,0,3]; [0,0,0,1]];
  expc6 = MatrixLog6(T)
%} 
% Output:
% expc6 =
%    1.5708
%         0
%         0
%         0
%    2.3562
%    2.3562
[m,n]=size(T);
if m==4 && n==4
    [R,p]=TransToRp(T);
    if norm(R-eye(3))<0.001
        omg=[0;0;0];
        v=p/Magnitude(p);
        theta=Magnitude(p);
    elseif norm(R(1,1)+R(2,2)+R(3,3)+1)<0.001
        theta=pi;
        omg=MatrixLog3(R);
        v=(eye(3)/theta-0.5*VecToso3(omg)+(1/theta-0.5*cot(theta/2))*VecToso3(omg)*VecToso3(omg))*p;
    else
        theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
        w1=1/(2*sin(theta))*(R-R');
        omg=so3ToVec(w1);
        v=(eye(3)/theta-0.5*VecToso3(omg)+(1/theta-0.5*cot(theta/2))*VecToso3(omg)*VecToso3(omg))*p;
    end
    expc6=theta*[omg;v];
else
    msg = 'Input matrix is the wrong size.';
    error(msg);
end

end

