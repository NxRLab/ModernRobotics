function drawKukaRobot(q, Bases, W, L)
close all;
n = 7;

W = [0 0 0 0 0 0 0; 0 -1 0 1 0 -1 0;1 0 1 0 1 0 1];
L = [0 0 0 0 0 0 0;0 0 0 0 0 0 0;0.3600 0.3600 0.3600 0.7800 0.7800 1.1800 1.4210];
 
h = 0;
S = [];
for i = 1:n
    Si = ScrewToAxis(L(:,i), W(:,i), h);
    S = [S Si];
end

b2 = eye(4);
b2(2,4) = -0.3; b2(1,4) = 0.1;

Base = eye(4);

param.Base = Base;
param.W = W;
param.L = L;
param.Slist = S;
param.M = [1 0 0 0;0 1 0 0;0 0 1 1.421;0 0 0 1];
param.view = 3;
param.qmin = (2/3)*[-pi -pi -pi -pi -pi -pi -pi]';
param.qmax = (2/3)*[pi pi pi pi pi pi pi]';

% drawExpRobot(4*rand(7,1)-2, param)
n = size(W,2);

nData = 100;
t = linspace(0,10,nData);

q0 = 2*rand(7,1)-1;
T0 = FKinSpace(param.M, param.Slist, q0);
Tn = eye(4);
Tn(1,4) = 0.1;
Tn(2,4) = 0.05;
Tn(3,4) = 0.9;
cartTraj = CartesianTrajectory(T0, Tn , 10, nData, 3); 
q = [];
fk_updated = zeros(3, nData);
fk_actual  = zeros(3, nData);

for i = 1:nData
    q_ = IKinSpace3D(param.Slist, param.M, cartTraj{i}, q0, 0.01, 0.01, param);
    % q_ =  projectJointSpace(q_, param);
    fk_ = FKinSpace(param.M, param.Slist, q_);
    fk_actual(:,i) = cartTraj{i}(1:3,4);
    fk_updated(:,i) = fk_(1:3,4);
    q = [q q_]; 
end

Anim.filename = '7dof.gif';
Anim.speed = 2;
Anim.plotFunc = @(t,q)( drawExpRobot(q,param) );
Anim.verbose = true;
animateExp(t,q,Anim);

hold on
plot3(fk_updated(1,:), fk_updated(2,:), fk_updated(3,:), 'r*')
plot3(fk_actual(1,:), fk_actual(2,:), fk_actual(3,:), 'k*')

figure(2)
for i = 1:7
   plot(q(i,:)) 
   hold on
end
end