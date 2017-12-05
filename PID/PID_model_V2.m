% Define a global variable to keep track of information
global e_prev t_prev v_all w_all

tspan = [0,60];
e_prev(1:3,1) = [0;0;0];
t_prev = tspan(1);
v_all = [];
w_all = [];

%%%%%%%%%%%%%%%%%
% Define reference function
% Straight Line
x0 = [0;1;pi/2;0;0;0];
path_type = 'line';
xr_fun = @(t_in) [t_in, zeros(length(t_in),1), zeros(length(t_in),1)];

%Circle
% x0 = [2;0;-2*pi/3;0;0;0];
% path_type = 'circle';
% xr_fun = @(t_in) [cos(t_in), sin(t_in), atan2(cos(t_in), -sin(t_in))];
%%%%%%%%%%%%%%%%

% PI Model
options = odeset('MaxStep', .4);
[t, x_out] = ode45(@(t,x) ode_fun1(t,x,xr_fun,path_type), tspan, x0, options);

% Plot results
figure
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
legend('Prop','Int','Deriv')
subplot(2,2,4)
plot(1:size(w_all,2),w_all)
title('Controllers on W')
legend('Prop','Int','Deriv')

function dx = ode_fun1(t, x, xr_fun, path_type)
global e_prev t_prev v_all w_all 
% Define a reference trajectory
xr = xr_fun(t); 
xr = reshape(xr,3,1);
x(3) = wrapToPi(x(3));

% Define error
e = [xr;0;0;0] - x;

% Determine direction of angle rotation
e(3) = -angdiff(xr(3),x(3));

% Calculate discrete derivatives
deriv = (e(1:3) - e_prev(1:3)) ./ (t - t_prev);
deriv(isnan(deriv)) = 0;
d_cut = 50;
deriv(deriv > d_cut) = d_cut;
deriv(deriv < -d_cut) = -d_cut;

% Define Controller Constants
switch path_type
    case 'line'
        % Controller Constants
        kp_v = [.1,.1,0];
        kp_th = [0,.1,1];
        ki_v = [.1,.1,0];
        ki_th = [0,.1,.15];
        kd_v = [.01,.01,0];
        kd_th = [0,0,0];
        
        % Create Controllers
        v_all(1:3, end+1) = [kp_v*e(1:3); ki_v*x(4:6); kd_v*deriv];
        w_all(1:3, end+1) = [kp_th*e(1:3); ki_th*x(4:6); kd_th*deriv];
    case 'circle'
        kp_v = [.5,.5,0];
        kp_th = [-.1,.1,1];
        ki_v = [.1,.1,0];
        ki_th = [0,0,.15];
        kd_v = [.2,.2,0];
        kd_th = [.001,.001,.05];
        % Create Controllers
        v_all(1:3, end+1) = [kp_v*e(1:3); ki_v*x(4:6); kd_v*abs(deriv)];
        w_all(1:3, end+1) = [kp_th*e(1:3); ki_th*x(4:6); kd_th*deriv];
    otherwise
        error('Path Type Note Defined Well')
end

v = sum(v_all(:,end));
w = sum(w_all(:,end));
% v = kp_v*abs(e(1:3)) + ki_v*abs(x(4:6)) + kd_v*abs(deriv);
% w = kp_th*e(1:3) + ki_th*x(4:6) + kd_th*deriv;

%%%%%%%%%%%%%%%%
% Derivative gain model
dx(1,1) = cos(x(3)) * v;
dx(2,1) = sin(x(3)) * v;
dx(3,1) = w;
dx(4,1) = e(1);
dx(5,1) = e(2);
dx(6,1) = e(3);

% Save values for discrete derivatives
e_prev(1:3) = e(1:3);
t_prev = t;
end
