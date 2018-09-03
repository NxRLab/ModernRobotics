function taumat ...
         = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, ...
                                     g, Ftipmat, Mlist, Glist, Slist)
% *** CHAPTER 8: DYNAMICS OF OPEN CHAINS ***
% Takes thetamat: An N x n matrix of robot joint variables,
%       dthetamat: An N x n matrix of robot joint velocities,
%       ddthetamat: An N x n matrix of robot joint accelerations,
%       g: Gravity vector g,
%       Ftipmat: An N x 6 matrix of spatial forces applied by the 
%                end-effector (If there are no tip forces, the user should
%                input a zero and a zero matrix will be used),
%       Mlist: List of link frames i relative to i-1 at the home position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.
% Returns taumat: The N x n matrix of joint forces/torques for the 
%                 specified trajectory, where each of the N rows is the 
%                 vector of joint forces/torques at each time step.
% This function uses InverseDynamics to calculate the joint forces/torques 
% required to move the serial chain along the given trajectory.
% Example Inputs (3 Link Robot)

% clc; clear;
% %Create a trajectory to follow using functions from Chapter 9
% thetastart = [0; 0; 0];
% thetaend = [pi / 2; pi / 2; pi / 2];
% Tf = 3;
% N= 1000;
% method = 5 ;
% traj = JointTrajectory(thetastart, thetaend, Tf, N, method);
% thetamat = traj;
% dthetamat = zeros(1000, 3);
% ddthetamat = zeros(1000, 3);
% dt = Tf / (N - 1);
% for i = 1: N - 1
%   dthetamat(i + 1, :) = (thetamat(i + 1, :) - thetamat(i, :)) / dt;
%   ddthetamat(i + 1, :) = (dthetamat(i + 1, :) - dthetamat(i, :)) / dt;
% end
% %Initialise robot descripstion (Example with 3 links)
% g = [0; 0; -9.8];
% Ftipmat = ones(N, 6); 
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
% taumat = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, ...
%                                  g, Ftipmat, Mlist, Glist, Slist);
% %Output using matplotlib to plot the joint forces/torques
% time=0: dt: Tf;
% plot(time, taumat(:, 1), 'b')
% hold on
% plot(time, taumat(:, 2), 'g')
% plot(time, taumat(:, 3), 'r')
% title('Plot for Torque Trajectories')
% xlabel('Time')
% ylabel('Torque')
% legend('Tau1', 'Tau2', 'Tau3')
% 

thetamat = thetamat';
dthetamat = dthetamat';
ddthetamat = ddthetamat';
Ftipmat = Ftipmat';
taumat = thetamat;
for i = 1: size(thetamat, 2)
   taumat(:, i) ...
   = InverseDynamics(thetamat(:, i), dthetamat(:, i), ddthetamat(:, i), ...
                     g, Ftipmat(:, i), Mlist, Glist, Slist);
end
taumat = taumat';
end