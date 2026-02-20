function animateExp(t,x, robotParam, P)
%animate(t,x,P)
%
%FUNCTION:
%   Animate is used to animate a system with state x at the times in t.
%
%INPUTS:
%   t = [1xM] vector of times, Must be monotonic: t(k) < t(k+1)
%   x = [NxM] matrix of states, corresponding to times in t
%   P = animation parameter struct, with fields:
%     .plotFunc = @(t,x) = function handle to create a plot
%       	t = a scalar time
%       	x = [Nx1] state vector
%     .speed = scalar multiple of time, for playback speed
%     .figNum = (optional) figure number for plotting. Default = 1000.
%     .verbose = set to false to prevent printing details. Default = true;
%
%OUTPUTS:
%   Animation based on data in t and x.
%
%NOTES:
%
%   Keyboard commands during simulation:
%
%       'space' - toggle pause
%
%       'r' - reset animation
%
%       'uparrow' - go faster
%
%       'downarrow' - go slower
%
%       'rightarrow' - jump forward by 5 frames
%
%       'leftarrow' - jump backward by 5 frames
%
%       'esc' - quit animation
%
%

if ~isfield(P,'figNum')
    P.figNum=1000;  %Default to figure 1000
end
if ~isfield(P,'verbose')
    P.verbose = true;
end
if ~isfield(P,'frameRate')
    P.targetFrameRate = 10;
end

% Animation call-back variables:
IS_PAUSED = false;
VERBOSE = P.verbose;
SPEED = P.speed;
QUIT = false;
START_TIME = t(1);
SIM_TIME = START_TIME;

% Set up the figure, and attach keyboard events.
fig = figure(P.figNum); clf(fig);
fig.Position = [400 400 1080 1080];
set(fig,'KeyPressFcn',@keyDownListener)

tic;    %Start a timer
timeBuffer(1:3) = toc;
myVideo = VideoWriter('myVideoFile','Motion JPEG AVI'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
filename = P.filename;
init = 0;
fkCurrentStore = []; fkDesired = []; probNow = [];

while SIM_TIME < t(end)
    init = init + 1;
    clf(fig)
    %Interpolate to get the new point:
    xNow = interp1(t',x',SIM_TIME,'linear','extrap')';
    
    %Call the plot command
    feval(P.plotFunc,SIM_TIME,xNow);
    drawnow;
    pause(0.005);
    
    %Plot the current trajectory
    fk = FKinSpace(robotParam.M, robotParam.Slist, xNow);
    fkCurrentStore = [fkCurrentStore fk(1:3,4)];
    
    for k = 1:numel(probNow)
        plot3(fkCurrentStore(1,k), fkCurrentStore(2,k), fkCurrentStore(3,k), 'k.', 'MarkerSize', probNow(k)); 
    end

    
    %Set up targets for timing
    dtReal = 0.2*(timeBuffer(1) - timeBuffer(3));
    if IS_PAUSED
        dtSim = 0;
    else
        dtSim = SPEED*dtReal;
    end
    SIM_TIME = SIM_TIME + dtSim;
    
    %Record the frame rate:
    timeBuffer(3) = timeBuffer(2);
    timeBuffer(2) = timeBuffer(1);
    timeBuffer(1) = toc;
    
    frame = getframe(gcf); %get frame
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if (init == 1)
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','DelayTime',0, 'WriteMode','append');
    end

    
    % Check exit conditions:
    if QUIT
        break
    end

    
end

% close(myVideo)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Graphics call-back functions                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


    function keyDownListener(~,event)
        switch event.Key
            case 'space'
                IS_PAUSED = ~IS_PAUSED;
                if VERBOSE
                    if IS_PAUSED
                        fprintf('--> animation paused...');
                    else
                        fprintf(' resumed! \n');
                    end
                end
            case 'r'
                SIM_TIME = START_TIME;
                if VERBOSE
                    disp('--> restarting animation');
                end
            case 'uparrow'
                SPEED = 2*SPEED;
                if VERBOSE
                    fprintf('--> speed set to %3.3f x real time\n',SPEED);
                end
            case 'downarrow'
                SPEED = SPEED/2;
                if VERBOSE
                    fprintf('--> speed set to %3.3f x real time\n',SPEED);
                end
            case 'rightarrow'
                timeSkip = 5*SPEED*dtReal;
                SIM_TIME = SIM_TIME + timeSkip;
                if VERBOSE
                    fprintf('--> skipping forward by %3.3f seconds\n',timeSkip);
                end
            case 'leftarrow'
                timeSkip = 5*SPEED*dtReal;
                SIM_TIME = SIM_TIME - timeSkip;
                if VERBOSE
                    fprintf('--> skipping backward by %3.3f seconds\n',timeSkip);
                end
            case 'escape'
                QUIT = true;
                if VERBOSE
                    disp('--> animation aborted');
                end
            otherwise
        end
    end


end %animate.m
