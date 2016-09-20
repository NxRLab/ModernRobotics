%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
% Takes thetalist: A list of joint variables,
% dthetalist: A list of joint rates,
% Mlist: List of link frames i relative to i-1 at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame.
%
% Returns c: The vector c(thetalist,dthetalist) of Coriolis and centripetal terms
% for a given thetalist and dthetalist
% This function calls InverseDynamics with g = 0, Ftip = 0, and ddthetalist = 0
%
% Example Input (3 Link Robot):
%{
  clear;clc;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];
 
  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';
 
  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];
 
  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];
 
  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
%}
% Output:
% c =
%         0   -0.0277   -0.0069
n=size(Slist,1);
ddthetalist=zeros(1,n);
g=[0,0,0];
Ftip=zeros(1,6);
c=InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist);
end
