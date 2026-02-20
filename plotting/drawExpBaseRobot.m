function drawExpBaseRobot(q, base, p)

W = p.W;
L = p.L;
Base = eye(4);
Base(1:3, 1:3) = eul2rotm(base(4:6)');
Base(1:3, 4)   = base(1:3);

nDoF = size(W,2);
set(gcf,'color','w');

% Colors:
colorGround = [118,62,12]/255;
colorStance = [200,60,60]/255;
colorAxis = [0,0,0]/255;

% Set up the figure
hold off;
% xBnd = [-1,1] * 2.0;
% yBnd = [-1,1] * 2.0;
% zBnd = [-1,1] * 2.0;

xBnd = [-1,1] * 2.5;
yBnd = [-1,1] * 2.5;
zBnd = [-0.5,1] * 2.4;

% Plot the ground:
% plot(xBnd,[0,0],'LineWidth',6,'Color',colorGround);

hold on;
T0 = Base;
x0 = T0(1:3,4);
w0 = [0 0 1]';
L = [L;ones(1,nDoF)];
L = T0 * L;
L = L(1:3, :);
xE = L(:,1);



plot3(T0(1,4), T0(2,4), T0(3,4),'b.','MarkerSize',100);
% Plot the base 
A1 = x0;
A2 = x0 - w0*0.1;
plot3([A1(1) A2(1)], [A1(2) A2(2)], [A1(3) A2(3)], 'LineWidth',15,'Color',colorGround)
    
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
    l = rotmAxis * (T0(1:3,1:3)*p.M(1:3,4)+T0(1:3,4)-L(:,end));
    xE = x0 + l;
    
    % Plot the links:
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'LineWidth',10,'Color',colorStance);
    plot3([x0(1) xE(1)], [x0(2) xE(2)], [x0(3) xE(3)],'k.','MarkerSize',40);
    plot3([xE(1)], [xE(2)], [xE(3)],'s','MarkerSize',15, ...
    'MarkerFaceColor',[1 .6 .6]);

    fk = FKinSpace(p.M, p.Slist, q);


end