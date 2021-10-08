function draw2DBaseRobot(q, base, Td, prob, heatMap, param)
param.view = 2;

t = linspace(0,100,size(q,2));

Anim.speed = 5.25;
Anim.plotFunc = @(t,q, b)( drawExpBaseRobot(q, b, param) );
Anim.verbose = true;
Anim.filename = 'robot2DBaseRedundant.gif';
animateBaseExp(t, q, base, param, Td, prob, heatMap, Anim);
end