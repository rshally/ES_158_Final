function err_total = PI_model_V3(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Varargin is for determining optimal feedback matrices

switch nargin
    case 0
        type = 'y=sin(t)';
    otherwise
        type = varargin{1};
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global v_all w_all
t1 = tic;
tspan = [0,20];
v_all = [];
w_all = [];

%% Define reference function
switch type
    case 'y=0'
        x0 = [0;1;pi/2;0;0;0];
        xr_fun = @(t_in) [t_in, zeros(length(t_in),1), zeros(length(t_in),1)];
        kp_v = [.5,.5,0];
        kp_th = [0,.1,1];
        ki_v = [.05,.05,0];
        ki_th = [0,.1,.15];
    case 'y=x'
        x0 = [0;1;pi;0;0;0];
        xr_fun = @(t_in) [t_in, t_in, atan2(ones(length(t_in),1), ones(length(t_in),1))];
        kp_v = [.8,.8,0];
        kp_th = [-.1,.1,.5];
        ki_v = [.1,.1,0];
        ki_th = [-.05,.05,.2];
    case 'y=sin(t)'
        x0 = [0;1;pi;0;0;0];
        % Static feedback arrays
        kp_v = [1.5,1.5,0];
        kp_th = [0,.5,1];
        ki_v = [.1,.1,0];
        ki_th = [0,.05,.2];     
        xr_fun = @(t_in) [t_in, sin(t_in), atan2(cos(t_in), 1)];
    case 'circle'
        x0 = [1.4;0;pi/8;0;0;0];
        xr_fun = @(t_in) [cos(t_in), sin(t_in), atan2(cos(t_in), -sin(t_in))];
        kp_v = [1,1,.2];
        kp_th = [-.1,.5,1];
        ki_v = [.2,.2,0];
        ki_th = [.1,.1,.2];
    otherwise
        error('Reference function not recognized')
end

% Load best model if set to 1
if 0
    load(['best_PImodel_',type])
    kp_v = kp_v_best;
    kp_th = kp_th_best;
    ki_v = ki_v_best;
    ki_th = ki_th_best;
end

% Varargin is for determining optimal feedback matrices
switch nargin
    case 1
        % Do nothing
    case 5
        % Replace feedback matrices
        kp_v = varargin{1};
        kp_th = varargin{2};
        ki_v = varargin{3};
        ki_th = varargin{4};
    otherwise
        error('Varargin sequence not recognized')
end

% PI Model
options = odeset('MaxStep', .4);
[t, x_out] = ode45(@(t,x) ode_fun1(t,x,xr_fun, kp_v, kp_th, ki_v, ki_th, t1), tspan, x0, options);

%% Plotting output
% Plot results
f = figure('units','normalized','outerposition',[0 0 1 1]);
% State plot
subplot(2,2,1)
x_ref = xr_fun(t);
plot(x_ref(:,1), x_ref(:,2))
hold on
plot(x_out(:,1), x_out(:,2))
title('Trajectory')
legend('Reference','Path')
% Error plot
subplot(2,2,2)
err_out = sqrt((x_ref(:,1)-x_out(:,1)).^2 + (x_ref(:,2)-x_out(:,2)).^2);
plot(t, err_out );
title('Error')
ylim([0, max(err_out)])
% Plot influence of controllers:
subplot(2,2,3)
plot(1:size(v_all,2), v_all)
title('Controllers on V')
legend('Prop','Int')
subplot(2,2,4)
plot(1:size(w_all,2),w_all)
title('Controllers on W')
legend('Prop','Int')

% Plot error accumulation
f2 = figure('units','normalized','outerposition',[0 0 1 1]);
plot(t, cumtrapz(t, err_out))
title('Accumulation of Error over Time')
ylabel(sprintf('Error Accumulation\n(Position Units)'))
xlabel('Time (s)')

err_total = trapz(t, err_out);


%% Functions
function dx = ode_fun1(t, x, xr_fun, kp_v, kp_th, ki_v, ki_th, t1)
global v_all w_all 

% Time out if taking too long
if toc(t1) > 6; return; end

% Define a reference trajectory
xr = xr_fun(t); 
xr = reshape(xr,3,1);
x(3) = wrapToPi(x(3));

% Define error
e = [xr;0;0;0] - x;

% Determine direction of angle rotation
e(3) = -angdiff(xr(3),x(3));

% Create Controllers
v_all(1:2, end+1) = [kp_v*e(1:3); ki_v*x(4:6)];
w_all(1:2, end+1) = [kp_th*e(1:3); ki_th*x(4:6)];

v = sum(v_all(:,end));
w = sum(w_all(:,end));

%%%%%%%%%%%%%%%%
% PI model
dx(1,1) = cos(x(3)) * v;
dx(2,1) = sin(x(3)) * v;
dx(3,1) = w;
dx(4,1) = e(1);
dx(5,1) = e(2);
dx(6,1) = e(3);

