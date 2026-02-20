
addpath('robots/')

% robot definition
params = kukar820_model();

qStore = [0 0 0 0 0 0 0; pi/3 pi/4 0 pi/6 0 pi/4 0]';

t = linspace(0,10, size(qStore,2));
    
Anim.filename = 'kukaTestTrajectories.gif';
Anim.speed = 2;
Anim.plotFunc = @(t,q)(drawExpRobot(q,params) );
Anim.verbose = true;
animateExp(t, qStore, params, Anim);

