%**********************************************************************************************
%**********************  CHAPTER 5: VELOCITY KINEMATICS AND STATICS  **************************
%**********************************************************************************************

function Js = JacobianSpace(Slist,thetalist)
% Takes Slist: The joint screw axes in the space frame 
% when the manipulator is at the home position,
% thetalist: A list of joint coordinates. 
% Returns the corresponding space Jacobian (6xn real numbers).
% Example Input:
%{
  clear;clc;
  Slist = [[0,0,1,0,0.2,0.2]; [1,0,0,2,0,3]; [0,1,0,0,2,1];[1,0,0,0.2,0.3,0.4]];
  thetalist = [0.2,1.1,0.1,1.2];
  Js = JacobianSpace(Slist,thetalist)
%} 
% Output:
% Js =
%         0    0.9801   -0.0901    0.9575
%         0    0.1987    0.4446    0.2849
%    1.0000         0    0.8912   -0.0453
%         0    1.9522   -2.2164   -0.5116
%    0.2000    0.4365   -2.4371    2.7754
%    0.2000    2.9603    3.2357    2.2251
n=length(thetalist);
Slist = Slist';
Js=zeros(6,n);
if n==size(Slist,2)
    Js(:,1)=Slist(:,1);
    for i=2:n
        e=eye(4);
        for j=1:i-1
            e=e*MatrixExp6(Slist(:,j)*thetalist(j));
        end
        Js(:,i)=Adjoint(e)*Slist(:,i);
    end
else
    msg = 'Input is not appropriate.';
    error(msg);
end
end

