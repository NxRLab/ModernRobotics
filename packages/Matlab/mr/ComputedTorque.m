function taulist = ComputedTorque(thetalist, dthetalist, eint, g, ...
                                  Mlist, Glist, Slist, thetalistd, ...
                                  dthetalistd, ddthetalistd, Kp, Ki, Kd)
% *** CHAPTER 11: ROBOT CONTROL ***
% Takes thetalist: n-vector of joint variables,
%       dthetalist: n-vector of joint rates,
%       eint: n-vector of the time-integral of joint errors,
%       g: Gravity vector g,
%       Mlist: List of link frames {i} relative to {i-1} at the home 
%              position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns,
%       thetalistd: n-vector of reference joint variables,
%       dthetalistd: n-vector of reference joint velocities,
%       ddthetalistd: n-vector of reference joint accelerations,
%       Kp: The feedback proportional gain (identical for each joint),
%       Ki: The feedback integral gain (identical for each joint),
%       Kd: The feedback derivative gain (identical for each joint).
% Returns taulist: The vector of joint forces/torques computed by the 
%                  feedback linearizing controller at the current instant.
% Example Input:
% 
% clc; clear;
% thetalist = [0.1; 0.1; 0.1];
% dthetalist = [0.1; 0.2; 0.3];
% eint = [0.2; 0.2; 0.2];
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
% thetalistd = [1; 1; 1];
% dthetalistd = [2; 1.2; 2];
% ddthetalistd = [0.1; 0.1; 0.1];
% Kp = 1.3;
% Ki = 1.2;
% Kd = 1.1;
% taulist ...
% = ComputedTorque(thetalist, dthetalist, eint, g, Mlist, Glist, Slist, ...
%                thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd)
% 
% Output:
% taulist =
%  133.0053
%  -29.9422
%   -3.0328

e = thetalistd - thetalist;
taulist ...
= MassMatrix(thetalist, Mlist, Glist, Slist) ...
  * (Kp * e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist)) ...
  + InverseDynamics(thetalist, dthetalist, ddthetalistd, g, ...
                    zeros(6, 1), Mlist, Glist, Slist);
end