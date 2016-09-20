%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
% Takes thetalist: A list of joint variables,
% dthetalist: A list of joint rates,
% taulist: An n-vector of joint forces/torques,
% g: Gravity vector g,
% Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
% Mlist: List of link frames i relative to i-1 at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame.
% 
% Returns ddthetalist: The resulting joint accelerations.
% This function computes ddthetalist by solving:
% Mlist(thetalist)ddthetalist = taulist - c(thetalist,dthetalist) - g(thetalist) - Jtr(thetalist)Ftip
% 
% Example Input (3 Link Robot):
%{
  clear;clc;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];
 
  taulist = [0.5, 0.6, 0.7];
 
  g = [0,0,-9.8];
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
  ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
%}
% Output:
% ddthetalist =
%  -17.3288   20.9946   10.3651
RHS = ((taulist-VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)-GravityForces(thetalist,g,Mlist,Glist,Slist)-EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist))');
InMatTrans=(MassMatrix(thetalist,Mlist,Glist,Slist))';
ddthetalist= (InMatTrans*RHS)';
end

