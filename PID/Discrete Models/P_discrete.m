function cost = P_discrete(varargin)
% Possibility to add in controller constants
switch nargin
    case 0
        % Input controllers
        k1 = 1;
        k2 = 1;
        k3 = 10;
        plotting = 1;
    case 1
        k = varargin{1};
        k1 = k(1); k2 = k(2); k3 = k(3);
        plotting = 1;
    case 2
        k = varargin{1};
        k1 = k(1); k2 = k(2); k3 = k(3);
        plotting = varargin{2};
end

type = 'circle';
set(0,'defaultAxesFontSize',20)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import model constraints
file = '../../Model Constraints.txt';
B = importdata(file);
w_vel_c = B.data(1);
w_acc_c = B.data(2);
v_vel_c = B.data(3);
v_acc_c = B.data(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 10;
deltaT = 0.1;
finalIndex = time/deltaT - 1;

% Set up initial arrays of positions
x = zeros(3, finalIndex-1);
ref_path = x;
e = x;

%% Define reference function
switch type
    case 'circle'
        x(:,1) = [1.2, 0, pi/2 - .1]';
        p = 1;
        xr_fun = @(t_in) [cos(t_in/p), sin(t_in/p), atan2(1/p*cos(t_in/p), -1/p*sin(t_in/p))]';
        vr = @(t, p) sqrt((1/p*-sin(t)).^2 + (1/p*cos(t)).^2);
        wr = @(t, p) (-sin(t)*-sin(t)*1/p^3 - cos(t)*-cos(t)*1/p^3) / ((1/p*-sin(t)).^2 + (1/p*cos(t)).^2);
    otherwise
        error('Reference function not recognized')
end

%% Calculate Discrete Model
t_all = (0:finalIndex-1)*deltaT;

% Define error matrix
th = x(3,1);
rot_mat = [cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1];
x_ref(:,1) = xr_fun(0);
e(:,1) = rot_mat * (x_ref(:,1) - x(:,1)); 
v = zeros(1,finalIndex-1);
w = zeros(1,finalIndex-1);
for i = 1:finalIndex-1
    t = t_all(i);

    % Controller:
    v(i) = -[k1, 0, 0] * e(:,i);
    w(i) = [0, -sign(vr(t,p))*k2, -k3] * e(:,i);
    
    % Apply constraints to controllers:
    if i==1
        prev_vs = [0,0];
    else
        prev_vs = [v(i-1), w(i-1)];
    end
    v_by_acc_const = ( abs(prev_vs(1)) + v_acc_c*deltaT );
    w_by_acc_const = ( abs(prev_vs(2)) + w_acc_c*deltaT );
    v(i) = sign(v(i)) * min([abs(v(i)), v_vel_c, v_by_acc_const]);
    w(i) = sign(w(i)) * min([abs(w(i)), w_vel_c, w_by_acc_const]);
    
    % State update
    e(:,i+1) = e(:,i) + deltaT * ([0,       wr(t,p), 0      ;...
                                  -wr(t,p), 0,       vr(t,p);...
                                  0,        0,       0      ] * e(:,i) + ...
                                  [1,0;0,0;0,1]*[v(i);w(i)]); 
    
    % Store reference path 
    ref_path(:,i) = xr_fun(t);
    
    % Convert back to absolute position
    x(:,i+1) = ref_path(:,i) - inv(rot_mat)*e(:,i+1);
    
end

% Calculate position error
xy_err = sqrt(e(1,:).^2 + e(2,:).^2);

if plotting
    % Plot the model
    plot_model(t_all, x, xy_err, 'static')
end

% Calculate overall cost:
cost = calc_cost(e, [v;w]);
if plotting
    fprintf('Cost:\n')
    disp(cost)
end

