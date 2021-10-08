function drawManipulationObject(p)
% drawRobot(q,p)
%
% This function draws the robot with configuration q and parameters p
%
% INPUTS:
%   q = [5, 1] = column vector of a single robot configuration
%   p = parameter struct
%

position = p(1:2);
orientation = p(3);
scale = 0.2;

xHR = [-0.5 0.5 0.5 -0.5]/10 + scale;
yHR = [0.5 0.5 -0.5 -0.5]/10;

xHL = [-0.5 0.5 0.5 -0.5]/10 - scale;
yHL = [0.5 0.5 -0.5 -0.5]/10;

t = linspace(0,2*pi,50);
xy = scale * [cos(t);sin(t)];


R = rotz(orientation);
xy = R(1:2,1:2) * xy + position;

x = xy(1,:);
y = xy(2,:);
z = zeros(1,numel(t));

xyHR = R(1:2,1:2) * [xHR;yHR] + position;
xyHL = R(1:2,1:2) * [xHL;yHL] + position;

set(gcf,'color','w');

% Colors:
colorGround = [0,0,0]/255;

xBnd = [-1,1] * 1.4;
yBnd = [-0.25,1] * 2.5;
zBnd = [-0.0,1] * 2.4;


%% Plot the links:
plot3(x, y,z,'LineWidth',6,'Color',colorGround);
view(2)

ax = gca;
hold on
patch(ax,xyHR(1,:),xyHR(2,:),'black')
patch(ax,xyHL(1,:),xyHL(2,:),'black')

axis([xBnd,yBnd,zBnd]);

% axis equal
% axis off

end