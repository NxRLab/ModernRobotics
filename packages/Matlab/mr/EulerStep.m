function [thetalistNext, dthetalistNext] ...
         = EulerStep(thetalist, dthetalist, ddthetalist, dt)
% *** CHAPTER 8: DYNAMICS OF OPEN CHAINS ***
% Takes thetalist: n-vector of joint variables,
%       dthetalist: n-vector of joint rates,
%       ddthetalist: n-vector of joint accelerations,
%       dt: The timestep delta t.
% Returns thetalistNext: Vector of joint variables after dt from first 
%                        order Euler integration,
%         dthetalistNext: Vector of joint rates after dt from first order 
%                         Euler integration.
% Example Inputs (3 Link Robot):
% 
% clear; clc;
% thetalist = [0.1; 0.1; 0.1];
% dthetalist = [0.1; 0.2; 0.3];
% ddthetalist = [2; 1.5; 1];
% dt = 0.1;
% [thetalistNext, dthetalistNext] = EulerStep(thetalist, dthetalist, ...
%                                           ddthetalist, dt)
% 
% Output:
% thetalistNext =
%    0.1100
%    0.1200
%    0.1300
% dthetalistNext =
%    0.3000
%    0.3500
%    0.4000

thetalistNext = thetalist + dt * dthetalist;
dthetalistNext = dthetalist + dt * ddthetalist;
end