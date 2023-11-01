%% Robotics
% Lab 11 - Question 2 skeleton code

%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information


%% Set up robot
r = RacketBot                   % Create copy called 'robot'
%r.model.tool = transl(0.1,0,0);   % Define tool frame on end-effector


%% Start "real-time" simulation
q = zeros(1,7);                 % Set initial robot configuration 'q'

%HF = figure(1);         % Initialise figure to display robot
r.model.animate(zeros(1,7));          % Plot robot in initial configuration
r.model.delay = 0.001;    % Set smaller delay when animating
%set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 1000;  % Set duration of the simulation (seconds)
dt = 0.5;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    % 2 - use J inverse to calculate joint velocity
    % 3 - apply joint velocity to step robot joint angles 
    % -------------------------------------------------------------
    Kv = 0.5;
    Kw = 0.8;
    vx = Kv*axes(1);
    vy = Kv*axes(2);
    vz = Kv*(buttons(5)-buttons(7));
    wx = Kw*axes(4);
    wy = Kw*axes(5);
    w = Kw*(buttons(6)-buttons(8));

    dx = [vx;vy;vz;wx;wy;w];
    J = r.model.jacob0(q);
    dq = pinv (J)*dx;
    q = q + dq'*dt;
    % Update plot
    r.model.animate(q);  
    T = r.model.fkine(q);
    hold all
    Hq = findobj(gca, '-depth' , 1, 'type', 'quiver');
    delete(Hq);

    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
      
