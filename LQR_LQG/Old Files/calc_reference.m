function [xRef, uRef, x] = calc_reference(x_func, y_func, currIndex, finalIndex, deltaT, varargin)
% PURPOSE: Calculates positions and reference feedbacks for the LQR model
% based on the input position functions
%
% INPUTS: x_func and y_func are self-explanatory
%         currIndex  --> the current index. Will be 1 unless there is a
%           change in reference path in the middle of the script
%         finalIndex --> the final index of the calculation
%         deltaT     --> the time step in the calculations
%         varargin   --> contains the final robot position, if doing 
%           real-time updates
%
%
% OUTPUTS: xRef --> the coordinates of the reference path
%          uRef --> the coordinates of the reference feedback
%          x    --> the coordinates of the robot's motion (error or
%          coordinates?)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Values
Q = eye(3);
F = eye(3);
R = eye(2);
x = cell(finalIndex,1);
if currIndex==1
    x{1} = [1.4 0 pi/8]';
    xRef = zeros(finalIndex,3);
    uRef = zeros(finalIndex,2);
else
    x(1:currIndex) = varargin{1}(1:currIndex);
    xRef(1:currIndex,:) = varargin{2}(1:currIndex,:);
    uRef(1:currIndex,:) = varargin{3}(1:currIndex,:);
end
% Calculate A and B matrices
A = cell(finalIndex,1);
B = A;


%% Calculate reference path

theta_fun_sym = atan2(diff(y_func), diff(x_func));
theta_fun_deriv_sym = diff(theta_fun_sym);
theta_fun = matlabFunction(theta_fun_sym);
theta_fun_deriv = matlabFunction(theta_fun_deriv_sym);
x_func_anon = matlabFunction(x_func);
y_func_anon = matlabFunction(y_func);

% Detmerine state matrices
syms x1 x2 x3 u1 u2
state = [u1*cos(x3); u1*sin(x3); u2];
A_1 = jacobian(state, [x1, x2, x3]);
B_1 = jacobian(state, [u1, u2]);

%% Determine Reference Values
fprintf('Calculating A and B\n')
for i = currIndex:finalIndex
    % NOTE: First uRef value is NaN
    % Check if any values are constants and therefore don't take inputs:
    c_check = [nargin(x_func_anon), nargin(y_func_anon), nargin(theta_fun), nargin(theta_fun_deriv)];
    % VERY LONG way to get correct number of inputs for functions
    tin = deltaT*(i-1);
    if c_check(1)==0
        x_val = x_func_anon();
    else
        x_val = x_func_anon(tin);
    end
    
    if c_check(2)==0
        y_val = y_func_anon();
    else
        y_val = y_func_anon(tin);
    end
    
    if c_check(3)==0
        theta_val = theta_fun();
    else
        theta_val = theta_fun(tin);
    end
    
    if c_check(4)==0
        theta_deriv_val = theta_fun_deriv();
    else
        theta_deriv_val = theta_fun_deriv(tin);
    end
    % Now update reference trajectories and feedbacks
    xRef(i,:) = [x_val, y_val, theta_val];
    xRef(isnan(xRef)) = 0;
    uRef(i,:) = [sqrt(x_val^2 + y_val^2), theta_deriv_val];
    uRef(isnan(xRef)) = 0;
    
    % Calculate A and B matrices at each time step
    A{i} = eval(eye(3) + deltaT*subs(A_1, [x1, x2, x3, u1, u2], [xRef(i,1), xRef(i,2), xRef(i,3), uRef(i,1), uRef(i,2)]));
    B{i} = eval(deltaT*subs(B_1, [x1, x2, x3, u1, u2], [xRef(i,1), xRef(i,2), xRef(i,3), uRef(i,1), uRef(i,2)]));
end
fprintf('Finished A and B\n')

%% Compute LQR Controller
S = cell(finalIndex,1);
S{finalIndex} = F;
for i = finalIndex-1:-1:currIndex
S{i} = transpose(A{i})*(S{i+1}-S{i+1}*B{i}*...
    inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1})*A{i}+Q;
end

L = cell(finalIndex-1,1);
u = L;

for i = currIndex:finalIndex-1
    L{i} = inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1}*A{i};
    u{i} = -L{i}*x{i};
    x{i+1} = A{i}*x{i}+B{i}*u{i};
end

end

