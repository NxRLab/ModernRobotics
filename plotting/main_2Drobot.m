
% robot definition
xvec = [0.    0.   0.5    0.0   1.0    0.0   1.5    0];
[M, Slist] = robotEXP(xvec);
param.M = M;
param.Slist = Slist;
L1 = Link('d', 0, 'a', 0.5, 'alpha', 0);
param.ndof = 3;

% joint limits
param.qmin = 1.5*[-pi/2 -pi/2 -pi/2]';
param.qmax = 1.5*[pi/2 pi/2 pi/2]';

qStore = [0 0 0; pi/3 pi/4 0]';
param.figNum = 1;

animateRobot2D(xvec, qStore, param)
