%**********************************************************************************************
%*****************************  CHAPTER 6: INVERSE KINEMATICS  ********************************
%**********************************************************************************************

function [thetalist,success] = IKinSpace(Slist,M,T,thetalist0,eomg,ev)
% Takes Slist: The joint screw axes in the space frame
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
  Slist = [[0,0,1,0,0,0]; [0,1,0,0,0,0]; [0,0,1,0,0,0];[0,1,0,-0.550,0,0.045]; [0,0,1,0,0,0]; [0,1,0,-0.850,0,0]; [0,0,1,0,0,0]];
  M = [[1,0,0,0]; [0,1,0,0]; [0,0,1,0.910]; [0,0,0,1]];
  T = [[1,0,0,0.4]; [0,1,0,0]; [0,0,1,0.4]; [0,0,0,1]];
  thetalist0 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1];
  eomg = 0.01;
  ev = 0.001;
  [thetalist,success] = IKinSpace(Slist,M,T,thetalist0,eomg,ev)
%}
% Output:
% thetalist =
%    6.9348    7.1213  -17.4424  -20.5590   -8.6012   17.3067   13.6980
% success =
%     1
maxiterations = 20;
success = true;
i=0;
vs=MatrixLog6(TransInv(FKinSpace(M,Slist,thetalist0))*T);
w=vs(1:3);
v=vs(4:6);
thf(1,:)=thetalist0;
while ( Magnitude(w)>eomg || Magnitude(v)>ev ) && i<maxiterations
    Jb = (Adjoint(TransInv(FKinSpace(M, Slist, thf(i+1,:)))))*(JacobianSpace(Slist,thf(i+1,:)));
    thf(i+2,:)=thf(i+1,:)+(pinv(Jb)*vs)';
    i=i+1;
    vs=MatrixLog6(TransInv(FKinSpace(M,Slist,thf(i+1,:)))*T);
    w=vs(1:3);
    v=vs(4:6);
    thetalist = thf(i+1,:);
    if (i == maxiterations-1)
        success = false;
    end
end

