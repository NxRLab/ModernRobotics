function draw2DRobot(q, param)

t = linspace(0,100,size(q,2));

Anim.speed = 5.25;
Anim.plotFunc = @(t,q)( drawExpRobot(q, param) );
Anim.verbose = true;
Anim.filename = 'robot2DRedundant.gif';
animateExp(t, q, param, Anim);

end