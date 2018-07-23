function [taumat, thetamat] ...
         = SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, ...
                           Glist, Slist, thetamatd, dthetamatd, ...
                           ddthetamatd, gtilde, Mtildelist, Gtildelist, ...
                           Kp, Ki, Kd, dt, intRes)
% *** CHAPTER 11: ROBOT CONTROL ***                       
% Takes thetalist: n-vector of initial joint variables,
%       dthetalist: n-vector of initial joint velocities,
%       g: Actual gravity vector g,
%       Ftipmat: An N x 6 matrix of spatial forces applied by the
%                end-effector (If there are no tip forces, the user should 
%                input a zero and a zero matrix will be used),
%       Mlist: Actual list of link frames i relative to i? at the home 
%              position,
%       Glist: Actual spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns,
%       thetamatd: An Nxn matrix of desired joint variables from the 
%                  reference trajectory,
%       dthetamatd: An Nxn matrix of desired joint velocities,
%       ddthetamatd: An Nxn matrix of desired joint accelerations,
%       gtilde: The gravity vector based on the model of the actual robot
%               (actual values given above),
%       Mtildelist: The link frame locations based on the model of the 
%                   actual robot (actual values given above),
%       Gtildelist: The link spatial inertias based on the model of the 
%                   actual robot (actual values given above),
%       Kp: The feedback proportional gain (identical for each joint),
%       Ki: The feedback integral gain (identical for each joint),
%       Kd: The feedback derivative gain (identical for each joint),
%       dt: The timestep between points on the reference trajectory.
%       intRes: Integration resolution is the number of times integration 
%               (Euler) takes places between each time step. Must be an 
%               integer value greater than or equal to 1.
% Returns taumat: An Nxn matrix of the controller commanded joint 
%                 forces/torques, where each row of n forces/torques 
%                 corresponds to a single time instant,
%         thetamat: An Nxn matrix of actual joint angles.
% The end of this function plots all the actual and desired joint angles.
% Example Usage
% 
% clc; clear;
% thetalist = [0.1; 0.1; 0.1];
% dthetalist = [0.1; 0.2; 0.3];
% %Initialize robot description (Example with 3 links)
% g = [0; 0; -9.8];
% M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.089159]; [0, 0, 0, 1]];
% M12 = [[0, 0, 1, 0.28]; [0, 1, 0, 0.13585]; [-1, 0 ,0, 0]; [0, 0, 0, 1]];
% M23 = [[1, 0, 0, 0]; [0, 1, 0, -0.1197]; [0, 0, 1, 0.395]; [0, 0, 0, 1]];
% M34 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.14225]; [0, 0, 0, 1]];
% G1 = diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7]);
% G2 = diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393]);
% G3 = diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275]);
% Glist = cat(3, G1, G2, G3);
% Mlist = cat(3, M01, M12, M23, M34); 
% Slist = [[1; 0; 1;      0; 1;     0], ...
%        [0; 1; 0; -0.089; 0;     0], ...
%        [0; 1; 0; -0.089; 0; 0.425]];
% dt = 0.01;
% %Create a trajectory to follow
% thetaend =[pi / 2; pi; 1.5 * pi];
% Tf = 1;
% N = Tf / dt;
% method = 5;
% thetamatd = JointTrajectory(thetalist, thetaend, Tf, N, method);
% dthetamatd = zeros(N, 3);
% ddthetamatd = zeros(N, 3);
% dt = Tf / (N - 1);
% for i = 1: N - 1
%   dthetamatd(i + 1, :) = (thetamatd(i + 1, :) - thetamatd(i, :)) / dt;
%   ddthetamatd(i + 1, :) = (dthetamatd(i + 1, :) ...
%                           - dthetamatd(i, :)) / dt;
% end
% %Possibly wrong robot description (Example with 3 links)
% gtilde = [0.8; 0.2; -8.8];
% Mhat01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.1]; [0, 0, 0, 1]];
% Mhat12 = [[0, 0, 1, 0.3]; [0, 1, 0, 0.2]; [-1, 0 ,0, 0]; [0, 0, 0, 1]];
% Mhat23 = [[1, 0, 0, 0]; [0, 1, 0, -0.2]; [0, 0, 1, 0.4]; [0, 0, 0, 1]];
% Mhat34 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.2]; [0, 0, 0, 1]];
% Ghat1 = diag([0.1, 0.1, 0.1, 4, 4, 4]);
% Ghat2 = diag([0.3, 0.3, 0.1, 9, 9, 9]);
% Ghat3 = diag([0.1, 0.1, 0.1, 3, 3, 3]);
% Gtildelist = cat(3, Ghat1, Ghat2, Ghat3);
% Mtildelist = cat(4, Mhat01, Mhat12, Mhat23, Mhat34); 
% Ftipmat = ones(N, 6);
% Kp = 20;
% Ki = 10;
% Kd = 18;
% intRes = 8;
% [taumat, thetamat] ...
% = SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, ...
%                 Slist, thetamatd, dthetamatd, ddthetamatd, gtilde, ...
%                 Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes);
% 

Ftipmat = Ftipmat';
thetamatd = thetamatd';
dthetamatd = dthetamatd';
ddthetamatd = ddthetamatd';
n = size(thetamatd, 2);
taumat = zeros(size(thetamatd));
thetamat = zeros(size(thetamatd));
thetacurrent = thetalist;
dthetacurrent = dthetalist;
eint = zeros(size(thetamatd, 1), 1);
for i=1: n
    taulist ...
    = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, ...
                     Mtildelist, Gtildelist, Slist, thetamatd(:, i), ...
                     dthetamatd(:, i), ddthetamatd(:, i), Kp, Ki, Kd);
    for j=1: intRes
        ddthetalist ...
        = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, ...
                          Ftipmat(:, i), Mlist, Glist, Slist);    
        [thetacurrent, dthetacurrent] ...
        = EulerStep(thetacurrent, dthetacurrent, ddthetalist, ...
                    (dt / intRes));
    end
    taumat(:, i) = taulist;
    thetamat(:, i) = thetacurrent;    
    eint = eint + (dt*(thetamatd(:, i) - thetacurrent));
end
%Output using matplotlib
links = size(thetamat, 1);
leg = cell(1, 2 * links);
time=0: dt: dt * n - dt;
timed=0: dt: dt * n - dt;
figure
hold on
for i=1: links
    col = rand(1, 3);
    plot(time, (thetamat(i, :)'), '-', 'Color', col)
    plot(timed, (thetamatd(i, :)'), '.', 'Color', col)
    leg{2 * i - 1} = (strcat('ActualTheta', num2str(i)));
    leg{2 * i} = (strcat('DesiredTheta', num2str(i)));
end
title('Plot of Actual and Desired Joint Angles')
xlabel('Time')
ylabel('Joint Angles')
legend(leg, 'Location', 'NorthWest')
taumat = taumat';
thetamat = thetamat';
end