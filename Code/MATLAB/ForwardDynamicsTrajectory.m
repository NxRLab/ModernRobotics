%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function [thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt, intRes)
% Takes thetalist: n-vector of initial joint variables.
% dthetalist: n-vector of initial joint rates.
% taumat: An N x n matrix of joint forces/torques, where each row is the joint effort at any time step,
% g: Gravity vector g,
% Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (If there are no tip
% forces, the user should input a zero and a zero matrix will be used),
% Mlist: List of link frames {i} relative to {i-1} at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame,
% dt: The timestep between consecutive joint forces/torques.
% intRes: Integration resolution is the number of times integration (Euler) takes places
% between each time step. Must be an integer value greater than or equal to 1
%
% Returns thetamat: The N x n matrix of robot joint angles resulting from the 
% specified joint forces/torques,
% dthetamat: The N x n matrix of robot joint velocities.
% This function simulates the motion of a serial chain given an open-loop history 
% of joint forces/torques.
% It calls a numerical integration procedure that uses ForwardDynamics.
% Example Inputs (3 Link Robot):
%{
  clc;clear;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];
  taumat = [[3.63,-6.58,-5.57]; [3.74,-5.55,-5.5];
  [4.31,-0.68,-5.19];[5.18,5.63,-4.31];[5.85,8.17,-2.59];[5.78,2.79,-1.7 ];[4.99,-5.3 ,-1.19];[4.08,-9.41,0.07];[3.56,-10.1,0.97];[3.49,-9.41,1.23]];

  %Initialise robot description (Example with 3 links)
  g = [0,0,-9.8];
  Ftipmat = ones(size(taumat,1),6);

  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';

  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];

  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];

  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  dt = 0.1;
  intRes = 8;

  [thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt, intRes);

  %Output using matplotlib to plot the joint forces/torques
  Tf = size(taumat,1);
  time=0:(Tf/size(thetamat,1)):(Tf-(Tf/size(thetamat,1)));
  plot(time,thetamat(:,1),'b')
  hold on
  plot(time,thetamat(:,2),'g')
  plot(time,thetamat(:,3),'r')
  plot(time,dthetamat(:,1),'c')
  plot(time,dthetamat(:,2),'m')
  plot(time,dthetamat(:,3),'y')
  title('Plot of Joint Angles and Joint Velocities')
  xlabel('Time')
  ylabel('Joint Angles/Velocities')
  legend('Theta1','Theta2','Theta3', 'DTheta1','DTheta2','DTheta3')
%}
if Ftipmat == 0
    NewFtipmat = zeros(size(taumat,1),6);
else
    NewFtipmat = Ftipmat;
end
thetamat(1,:)=thetalist;
dthetamat(1,:)=dthetalist;
for i=1:size(taumat,1)
    for j=1:intRes
        ddthetalist = ForwardDynamics(thetalist,dthetalist,taumat(i,:),g,NewFtipmat(i,:),Mlist,Glist,Slist);
        [thetalist,dthetalist] = EulerStep(thetalist,dthetalist,ddthetalist,(dt/intRes));
        thetamat(size(thetamat,1)+1,:) = thetalist;
        dthetamat(size(dthetamat,1)+1,:) = dthetalist;
    end
end
end

