%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function JTFtip = EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist)
% Takes thetalist: A list of joint variables,
% Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
% Mlist: List of link frames i relative to i-1 at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame.
% 
% Returns JTFtip: The joint forces and torques required only to create the end-effector force Ftip.
% This function calls InverseDynamics with g = 0, dthetalist = 0, and ddthetalist = 0.
%
% Example Input (3 Link Robot):
%{
  clear;clc;
  thetalist = [0.1,0.1,0.1];
 
  Ftip = [1,1,1,1,1,1];
 
  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';
 
  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];
 
  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];
 
  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  JTFtip = EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist)
%}
% Output:
% JTFtip =
%    2.8226    1.5305    1.6826
n=size(Slist,1);
dthetalist=zeros(1,n);
ddthetalist=zeros(1,n);
g=[0,0,0];
JTFtip=InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist);
end

