function drawExpRobot(q, p)
% drawRobot(q,p)
%
% This function draws the robot with configuration q and parameters p
%
% INPUTS:
%   q = [5, 1] = column vector of a single robot configuration
%   p = parameter struct
%
% close all
W = p.W;
L = p.L;
Base = p.Base;
Base(3,4) = 0;

nDoF = p.ndofs;
set(gcf,'color','w');

% Colors:
colorGround = [118,62,12]/255;
colorStance = [200,60,60]/255;
colorAxis = [0,0,0]/255;

% Set up the figure
% hold off;
% xBnd = [-1,1] * 2.0;
% yBnd = [-1,1] * 2.0;
% zBnd = [-1,1] * 2.0;

xBnd = [-1,1] * 2.5;
yBnd = [-1,1] * 2.5;
zBnd = [-1,1] * 1.4;

% Plot the ground:
% plot(xBnd,[0,0],'LineWidth',6,'Color',colorGround);

hold on;
T0 = Base;
x0 = T0(1:3,4);
w0 = [0 0 1]';
L = [L;ones(1,nDoF)];
L = Base * L;
L = L(1:3, :);
xE = L(:,1);



plot3(0, 0, 0,'b.','MarkerSize',40);
% Plot the base 
A1 = x0;
A2 = x0 - w0*0.1;
plot3([A1(1) A2(1)], [A1(2) A2(2)], [A1(3) A2(3)], 'LineWidth',8,'Color',colorGround)
    
view(p.view)

axang = [w0' q(1)];
rotm = axang2rotm(axang);
rotmAxis = eye(3);

% Plot the links:
plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'LineWidth',6,'Color',colorStance);


x0 = xE;
    
for i = 2:nDoF
    
    w = W(:,i-1); 
    axang = [w' q(i-1)];
    rotm = axang2rotm(axang);
    rotmAxis = rotmAxis * rotm;
    
    l = rotmAxis * (L(:,i)-L(:,i-1));
    xE = x0 + l;
    % xC = x0 + l/2;
    % plot3(xC(1), xC(2), xC(3),'k.','MarkerSize',30);
    
    % Plot the links:
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'LineWidth',10,'Color',colorStance);


    % Format the axis:
    axis([xBnd,yBnd,zBnd]);
    %     axis equal; 
    %     axis off;

    % Plot the joints:
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'k.','MarkerSize',60);

    % Plot the joint axis
    A1 = x0 + rotmAxis*w*0.1;
    A2 = x0 - rotmAxis*w*0.1;
    plot3([A1(1) A2(1)], [A1(2) A2(2)], [A1(3) A2(3)], 'LineWidth',3,'Color',colorAxis)
    
    
    % update the parameters
    x0 = xE;
    
    % plot the manipulability ellipsoid
    J = JacobianSpace(p.Slist, q);
    fk = FKinSpace(p.M, p.Slist, q);
    

end
    w = W(:,end); 
    axang = [w' q(end)];
    rotm = axang2rotm(axang);
    rotmAxis = rotmAxis * rotm;
    
    % Plot the joint axis
    A1 = x0 + rotmAxis*w*0.1;
    A2 = x0 - rotmAxis*w*0.1;
    plot3([A1(1) A2(1)], [A1(2) A2(2)], [A1(3) A2(3)], 'LineWidth',3,'Color',colorAxis)
    
    
    % plot the tool length
    l = rotmAxis * (p.M(1:3,4)-L(:,end));
    xE = x0 + l;
    xE(3) = xE(3);
    
    % Plot the links:
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'LineWidth',10,'Color',colorStance);
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'k.','MarkerSize',40);
    plot3([xE(1)], [xE(2)], [xE(3)],'s','MarkerSize',15, ...
    'MarkerFaceColor',[1 .6 .6]);

    fk = FKinSpace(p.M, p.Slist, q);
    % plot the trajectory

end