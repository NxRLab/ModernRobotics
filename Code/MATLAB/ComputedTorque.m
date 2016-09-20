%**********************************************************************************************
%********************************  CHAPTER 11: ROBOT CONTROL  *********************************
%**********************************************************************************************

function taulist = ComputedTorque(thetalist,dthetalist,eint,g,Mlist,Glist,Slist,thetalistd,dthetalistd,ddthetalistd,Kp,Ki,Kd)
% Takes thetalist: n-vector of joint variables,
% dthetalist: n-vector of joint rates,
% eint: n-vector of the time-integral of joint errors,
% g: Gravity vector g,
% Mlist: List of link frames {i} relative to {i-1} at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame,
% thetalistd: n-vector of reference joint variables,
% dthetalistd: n-vector of reference joint velocities,
% ddthetalistd: n-vector of reference joint accelerations,
% Kp: The feedback proportional gain (identical for each joint),
% Ki: The feedback integral gain (identical for each joint),
% Kd: The feedback derivative gain (identical for each joint).
%
% Returns taulist: The vector of joint forces/torques computed by the 
% feedback linearizing controller at the current instant.
% Example Input:
%{
  clc;clear;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];
  eint = [0.2,0.2,0.2];
  g = [0,0,-9.8];

  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';

  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];

  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];

  Slist = [[1.,0.,1.,0.,0.23,0.1];[0.,1.,1.,-.089,0.,0.2];[0.,1.,0.,-.089,0.,.425]];

  thetalistd = [1,1,1];
  dthetalistd = [2,1.2,2];
  ddthetalistd = [0.1,0.1,0.1];

  Kp = 1.3;
  Ki = 1.2;
  Kd = 1.1;
  taulist = ComputedTorque(thetalist,dthetalist,eint,g,Mlist,Glist,Slist,thetalistd,dthetalistd,ddthetalistd,Kp,Ki,Kd)
%}
% Output:
% taulist =
%    7.4418    1.3249   -4.8181
e = thetalistd-thetalist;
eder = dthetalistd-dthetalist;

M = MassMatrix(thetalist,Mlist,Glist,Slist);
c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist);
grav = GravityForces(thetalist,g,Mlist,Glist,Slist); 

taulist = (M*((ddthetalistd + (Kp*e) + (Ki*eint) + (Kd*eder))'))' + c + grav;
end


