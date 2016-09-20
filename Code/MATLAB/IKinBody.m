%**********************************************************************************************
%*****************************  CHAPTER 6: INVERSE KINEMATICS  ********************************
%**********************************************************************************************

function [thetalist,success] = IKinBody(Blist,M,T,thetalist0,eomg,ev)
% Takes Blist: The joint screw axes in the end-effector frame 
% when the manipulator is at the home position.
% M: The home configuration of the end-effector.
% T: The desired end-effector configuration Tsd
% thetalist0: An initial guess of joint angles that are close to satisfying Tsd
% eomg: A small positive tolerance on the end-effector orientation error. The returned joint angles
% must give an end-effector orientation error less than eomg.
% ev: A small positive tolerance on the end-effector linear position error. The returned joint
% angles must give an end-effector position error less than ev.
%
% The maximum number of iterations before the algorithm is terminated has been hardcoded in
% as a variable called maxiterations. It is set to 20 at the start of the function, but can be changed if needed.  
%
% Returns thetalist: Joint angles that achieve T within the specified tolerances,
% success: A logical value where TRUE means that the function found a solution and FALSE
% means that it ran through the set number of maximum iterations without finding a solution
% within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method
% 
% Example Inputs:
%{
  clear;clc;
  Blist = [[0,1,0,0.191,0,0.817]; [0,0,1,0.095,-0.817,0];[0,0,1,0.095,-0.392,0]; [0,0,1,0.095,0,0]; [0,-1,0,-0.082,0,0]; [0,0,1,0,0,0]];
  M = [[1,0,0,-0.817]; [0,0,-1,-0.191]; [0,1,0,-0.006]; [0,0,0,1]];
  T = [[0,1,0,-0.6]; [0,0,-1,0.1]; [-1,0,0,0.1]; [0,0,0,1]];
  thetalist0 =[0,0,0,0,0,0];
  eomg = 0.01;
  ev = 0.001;
  [thetalist,success] = IKinBody(Blist,M,T,thetalist0,eomg,ev)
%}
% Output:
% thetalist =
%   -0.4692   -0.8345    1.3953   -0.5611   -0.4673   -1.5706
% success =
%     1
maxiterations = 20;
success = true;
i=0;
vb=MatrixLog6(TransInv(FKinBody(M,Blist,thetalist0))*T);
w=vb(1:3);
v=vb(4:6);
thf(1,:)=thetalist0;
while ( Magnitude(w)>eomg || Magnitude(v)>ev ) && i<maxiterations
    thf(i+2,:)=thf(i+1,:)+(pinv(JacobianBody(Blist,thf(i+1,:)))*vb)';
    i=i+1;
    vb=MatrixLog6(TransInv(FKinBody(M,Blist,thf(i+1,:)))*T);
    w=vb(1:3);
    v=vb(4:6);
    thetalist = thf(i+1,:);
    if (i == maxiterations-1)
        success = false;
    end
end
