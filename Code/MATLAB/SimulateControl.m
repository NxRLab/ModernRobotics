%**********************************************************************************************
%********************************  CHAPTER 11: ROBOT CONTROL  *********************************
%**********************************************************************************************

function [taumat,thetamat] = SimulateControl(thetalist,dthetalist,g,Ftipmat,Mlist,Glist,Slist,thetamatd,dthetamatd,ddthetamatd,gtilde,Mtildelist,Gtildelist,Kp,Ki,Kd,dt,intRes)
% Takes thetalist: n-vector of initial joint variables,
% dthetalist: n-vector of initial joint velocities,
% g: Actual gravity vector g,
% Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (If there are no tip
% forces, the user should input a zero and a zero matrix will be used),
% Mlist: Actual list of link frames i relative to i−1 at the home position,
% Glist: Actual spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame,
% thetamatd: An Nxn matrix of desired joint variables from the reference trajectory,
% dthetamatd: An Nxn matrix of desired joint velocities,
% ddthetamatd: An Nxn matrix of desired joint accelerations,
% gtilde: The (possibly incorrect) model of the gravity vector,
% Mtildelist: The (possibly incorrect) model of the link frame locations,
% Gtildelist: The (possibly incorrect) model of the link spatial inertias,
% Kp: The feedback proportional gain (identical for each joint),
% Ki: The feedback integral gain (identical for each joint),
% Kd: The feedback derivative gain (identical for each joint),
% dt: The timestep between points on the reference trajectory.
% intRes: Integration resolution is the number of times integration (Euler) takes places
% between each time step. Must be an integer value greater than or equal to 1
% 
% Returns taumat: An Nxn matrix of the controller’s commanded joint forces/torques, where each row
% of n forces/torques corresponds to a single time instant,
% thetamat: An Nxn matrix of actual joint angles.
% The end of this function plots all the actual and desired joint angles.
% Example Usage
%{
  clc;clear;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];

  %Initialise robot description (Example with 3 links)
  g = [0,0,-9.8];

  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';

  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];

  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];

  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  dt = 0.01;

  %Create a trajectory to follow
  thetaend =[pi/2,pi,1.5*pi];
  Tf = 1;
  N = (Tf/dt);

  method = 5;
  traj = JointTrajectory(thetalist, thetaend, Tf, N, method);
  thetamatd = [thetalist];
  dthetamatd = [dthetalist];
  ddthetamatd = [0,0,0];

  for i=2:size(traj,1)
      thetamatd = [thetamatd;traj(i,:)];
      dthetamatd = [dthetamatd;(thetamatd(i,:)-thetamatd(i-1,:))/dt];
      ddthetamatd = [ddthetamatd;(dthetamatd(i,:)-dthetamatd(i-1,:))/dt];
  end

  %Possibly wrong robot description (Example with 3 links)
  gtilde = [0.8,0.2,-8.8];

  Mhat01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.1,1.]]';
  Mhat12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.3,.2,0.,1.]]';
  Mhat23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.2,.4,1]]';

  gtilde1 = [[.1,0.,0.,0.,0.,0.];[0.,.1,0.,0.,0.,0.];[0.,0.,.1,0.,0.,0.];[0.,0.,0.,4.0,0.,0.];[0.,0.,0.,0.,4.0,0.];[0.,0.,0.,0.,0.,3.0]];
  gtilde2 = [[.3,0.,0.,0.,0.,0.];[0.,.3,0.,0.,0.,0.];[0.,0.,.1,0.,0.,0.];[0.,0.,0.,9.0,0.,0.];[0.,0.,0.,0.,9.5,0.];[0.,0.,0.,0.,0.,8.0]];
  gtilde3 = [[.1,0.,0.,0.,0.,0.];[0.,.1,0.,0.,0.,0.];[0.,0.,.01,0.,0.,0.];[0.,0.,0.,3.0,0.,0.];[0.,0.,0.,0.,1.5,0.];[0.,0.,0.,0.,0.,2.0]];

  Gtildelist = [gtilde1;gtilde2;gtilde3];
  Mtildelist = [Mhat01;Mhat12;Mhat23];

  Ftipmat = ones(size(traj,1),6);

  Kp = 20;
  Ki = 10;
  Kd = 18;
  intRes = 8;
  [taumat,thetamat] = SimulateControl(thetalist,dthetalist,g,Ftipmat,Mlist,Glist,Slist,thetamatd,dthetamatd,ddthetamatd,gtilde,Mtildelist,Gtildelist,Kp,Ki,Kd,dt,intRes);
%}
n = size(thetamatd,1);
if Ftipmat == 0
    NewFtipmat = zeros(n,6);
else
    NewFtipmat = Ftipmat;
end
thetacurrent = thetalist;
dthetacurrent = dthetalist;
eint = 0;
taulist = ComputedTorque(thetacurrent,dthetacurrent,eint,gtilde,Mtildelist,Gtildelist,Slist,thetamatd(1,:),dthetamatd(1,:),ddthetamatd(1,:),Kp,Ki,Kd);
for j=1:intRes
    ddthetalist = ForwardDynamics(thetacurrent,dthetacurrent,taulist,g,NewFtipmat(1,:),Mlist,Glist,Slist);
    [thetacurrent,dthetacurrent] = EulerStep(thetacurrent,dthetacurrent,ddthetalist,(dt/intRes));
    taumat(1,:) = taulist;
    thetamat(1,:) = thetacurrent;
end
eint = eint + (dt*(thetamatd(1,:) - thetacurrent));

for i=2:n
    taulist = ComputedTorque(thetacurrent,dthetacurrent,eint,gtilde,Mtildelist,Gtildelist,Slist,thetamatd(i,:),dthetamatd(i,:),ddthetamatd(i,:),Kp,Ki,Kd);
    for j=1:intRes
        ddthetalist = ForwardDynamics(thetacurrent,dthetacurrent,taulist,g,NewFtipmat(i,:),Mlist,Glist,Slist);    
        [thetacurrent,dthetacurrent] = EulerStep(thetacurrent,dthetacurrent,ddthetalist,(dt/intRes));
        taumat(size(taumat,1)+1,:) = taulist;
        thetamat(size(thetamat,1)+1,:) = thetacurrent;
    end
    eint = eint + (dt*(thetamatd(i,:) - thetacurrent));
end

%Output using matplotlib
links = size(thetamat,2);
leg = cell(1,2*links);
leg{1} = 'ActualTheta1'; leg{2} = 'DesiredTheta1';
time=0:(dt/intRes):((dt*n)-(dt));
timed=0:(dt):((dt*n)-(dt))
disp(length(thetamatd(:,1)'));
plot(time,(thetamat(:,1)'),'-r')
hold on
plot(timed,(thetamatd(:,1)'),'.r')
for i=2:links
    col = rand(1,3);
    plot(time,(thetamat(:,i)'),'-','Color',col)
    plot(timed,(thetamatd(:,i)'),'.','Color',col)
    leg{(2*i)-1} = (strcat('ActualTheta', num2str(i)));
    leg{(2*i)} = (strcat('DesiredTheta', num2str(i)));
end
title('Plot of Actual and Desired Joint Angles')
xlabel('Time')
ylabel('Joint Angles')
legend(leg, 'Location', 'NorthWest')

end

