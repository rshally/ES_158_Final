function err_total = P_model_V2(type, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOTE: Need to load best model to test it
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define a global variable to keep track of information
global v_all w_all
t1 = tic;
tspan = [0,20];
v_all = [];
w_all = [];

%% Define reference function
switch type
    case 'y=0'
        x0 = [0;1;pi/2];
        xr_fun = @(t_in) [t_in, zeros(length(t_in),1), zeros(length(t_in),1)];
        kp_v = [.5,.5,0];
        kp_th = [0,.1,1];
    case 'y=x'
        x0 = [0;1;pi];
        xr_fun = @(t_in) [t_in, t_in, atan2(ones(length(t_in),1), ones(length(t_in),1))];
        kp_v = [.4,.4,0];
        kp_th = [-.1,.1,.5];
    case 'y=sin(t)'
        x0 = [0;1;pi];
        % Static feedback arrays
        kp_v = [10,2,.5];
        kp_th = [0,.5,1];        
        xr_fun = @(t_in) [t_in, sin(t_in), atan2(cos(t_in), 1)];
    case 'circle'
        x0 = [1.4 0 pi/8]';
        xr_fun = @(t_in) [cos(t_in), sin(t_in), atan2(cos(t_in), -sin(t_in))];
        kp_v = [.2,.2,0];
        kp_th = [-.1,.1,1];
    otherwise
        error('Reference function not recognized')
end

% Load best model if set to 1
if 1
    load(['best_Pmodel_',type])
    kp_v = kp_v_best;
    kp_th = kp_th_best;
end


% Varargin is for determining optimal feedback matrices
switch nargin
    case 1
        % Do nothing
    case 3
        % Replace feedback matrices
        kp_v = varargin{1};
        kp_th = varargin{2};
    otherwise
        error('Varargin sequence not recognized')
end

% P Model
options = odeset('MaxStep', .4);
[t, x_out] = ode45(@(t,x) ode_fun1(t,x,xr_fun, kp_v, kp_th, t1), tspan, x0, options);

%% Plot results
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
t_scaler = max(t)/size(v_all,2);
plot((1:size(v_all,2))*t_scaler, v_all)
title('Controllers on V')
legend('Prop')
subplot(2,2,4)
plot((1:size(w_all,2))*t_scaler,w_all)
title('Controllers on W')
legend('Prop')

% Plot error accumulation
f2 = figure('units','normalized','outerposition',[0 0 1 1]);
plot(t, cumtrapz(t, err_out))
title('Accumulation of Error over Time')
ylabel(sprintf('Error Accumulation\n(Position Units)'))
xlabel('Time (s)')


err_total = trapz(t, err_out);


function dx = ode_fun1(t, x, xr_fun, kp_v, kp_th, t1)
global v_all w_all 

% Time out if taking too long
if toc(t1) > 6; return; end

% Define a reference trajectory
xr = xr_fun(t); 
xr = reshape(xr,3,1);
x(3) = wrapToPi(x(3));

% Define error
e = xr - x;

% Determine direction of angle rotation
e(3) = -angdiff(xr(3),x(3));

% Create Controllers
v_all(1, end+1) = [kp_v*e(1:3)];
w_all(1, end+1) = [kp_th*e(1:3)];

v = sum(v_all(:,end));
w = sum(w_all(:,end));

%%%%%%%%%%%%%%%%
% PI model
dx(1,1) = cos(x(3)) * v;
dx(2,1) = sin(x(3)) * v;
dx(3,1) = w;

