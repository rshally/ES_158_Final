

% Define a global variable to keep track of information
global e_prev t_prev

tspan = [0,200];

%%%%%%%%%%%%%%%%%
% Define reference function
% Straight Line
%x0 = [0;1;pi/2;0;0;0];
%xr_fun = @(t_in) [t_in, zeros(length(t_in),1), zeros(length(t_in),1)];

%Circle
x0 = [2;0;-2*pi/3;0;0;0];
e_prev(1:3,1) = [0;0;0];
t_prev = tspan(1);
xr_fun = @(t_in) [cos(t_in), sin(t_in), atan2(cos(t_in), -sin(t_in))];
%%%%%%%%%%%%%%%%

% PI Model
options = odeset('MaxStep', .4);
[t, x_out] = ode45(@(t,x) ode_fun1(t,x,xr_fun), tspan, x0, options);

% Plot results
figure
% State plot
subplot(1,2,1)
x_ref = xr_fun(t);
plot(x_ref(:,1), x_ref(:,2))
hold on
plot(x_out(:,1), x_out(:,2))

legend('Reference','Path')
% Error plot
subplot(1,2,2)
err_out = sqrt((x_ref(:,1)-x_out(:,1)).^2 + (x_ref(:,2)-x_out(:,2)).^2);
plot(t, err_out );
ylim([0, max(err_out)])

function dx = ode_fun1(t, x, xr_fun)
global e_prev t_prev
% Define a reference trajectory
xr = xr_fun(t); 
xr = reshape(xr,3,1);
x(3) = wrapToPi(x(3));

% Define error
e = [xr;0;0;0] - x;

% Determine direction of angle rotation
e(3) = -angdiff(xr(3),x(3));

% Define Controller Constants For Linear
kp_v = [.5,.5,0];
kp_th = [-.1,.1,1];
ki_v = [.1,.1,0];
ki_th = [0,0,.15];
kd_v = [-.002,.002,0];
kd_th = [.001,.001,.05];

% Calculate discrete derivatives
deriv = (e(1:3) - e_prev(1:3)) ./ (t - t_prev);
deriv(isnan(deriv)) = 0;
d_cut = 50;
deriv(deriv > d_cut) = d_cut;
deriv(deriv < -d_cut) = -d_cut;

% Create Controllers
v = kp_v*abs(e(1:3)) + ki_v*abs(x(4:6)) + kd_v*abs(deriv);
w = kp_th*e(1:3) + ki_th*x(4:6) + kd_th*deriv;

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
