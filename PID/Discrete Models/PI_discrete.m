type = 'y=sin(t)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define a global variable to keep track of information
time = 20;
deltaT = 0.1;
finalIndex = time/deltaT - 1;

% Set up initial arrays of positions
x = zeros(6, finalIndex-1);
x_ref = x;
xTilda = x;

%% Define reference function
switch type
    case 'y=0'
        x(:,1) = [0;1;pi/2];
        xr_fun = @(t_in) [t_in, zeros(length(t_in),1), zeros(length(t_in),1)];
        kp_v = [.5,.5,0];
        kp_th = [0,.1,1];
    case 'y=x'
        x(:,1) = [0;1;pi];
        xr_fun = @(t_in) [t_in, t_in, atan2(ones(length(t_in),1), ones(length(t_in),1))];
        kp_v = [.4,.4,0];
        kp_th = [-.1,.1,.5];
    case 'y=sin(t)'
        x(:,1) = [0;1;pi];
        % Static feedback arrays
        kp_v = [1.5,1.5,.5];
        kp_th = [0,.5,1];
        ki_v = [1.5,1.5,.5];
        ki_th = [0,.5,1]; 
        
        xr_fun = @(t_in) [t_in, sin(t_in), atan2(cos(t_in), 1)];
    case 'circle'
        x(:,1) = [1 0 pi/2]';
        sl = 100;
        xr_fun = @(t_in) [cos(t_in/sl), sin(t_in/sl), atan2(1/sl*cos(t_in/sl), -1/sl*sin(t_in/sl))];
        kp_v = [10,10,0];
        kp_th = [0,0,1];
    otherwise
        error('Reference function not recognized')
end

%% Calculate Discrete Model
t_all = (0:finalIndex-1)*deltaT;
    
for i = 1:finalIndex
    t = t_all(i);
    x(3,i) = wrapToPi(x(3,i));
    
    % Define reference trajectory
    x_ref(:,i) = [xr_fun(t); 0;0;0];
    x_ref(3,i) = wrapToPi(x_ref(3,i));
    
    % Define controllers based on errors
    xTilda(:,i) = x_ref(:,i) - x(:,i); 
    v(:,i) = kp_v*xTilda(:,i);
    w(:,i) = kp_th*xTilda(:,i);
    
    % Update x based on xTilda error
    B = [cos(x(3,i)), 0; sin(x(3,i)), 0; 0, 1];
    x(:,i+1) = x(:,i) + deltaT*B*[v(:,i); w(:,i)];
    
end


%% Plot results
f = figure('units','normalized','outerposition',[0 0 1 1]);

% State plot
subplot(1,2,1)
plot(x_ref(1,:), x_ref(2,:))
hold on
plot(x(1,1:end-1), x(2,1:end-1))
title('Trajectory')
legend('Reference','Path')

% Error plot
subplot(1,2,2)
err_out = sqrt((x_ref(1,:)-x(1,1:end-1)).^2 + (x_ref(2,:)-x(2,1:end-1)).^2);
plot(t_all, err_out );
title('Error')
xlim([0, max(t)])
ylim([0, max(err_out)])


% Plot error accumulation
f2 = figure('units','normalized','outerposition',[0 0 1 1]);
plot(t_all, cumtrapz(t_all, err_out))
title('Accumulation of Error over Time')
ylabel(sprintf('Error Accumulation\n(Position Units)'))
xlabel('Time (s)')


err_total = trapz(t_all, err_out);
