function [xRef, uRef, xhat] = calc_reference_V3(x_func, y_func, currIndex, finalIndex, deltaT, varargin)
% PURPOSE: Calculates positions and reference feedbacks for the model
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
global sigmaV sigmaW xTilda yTilda xValues yValues err u remove_K

% Initial Values
Q = eye(3);
F = eye(3);
R = eye(2);
x = cell(finalIndex,1);

% Import model constraints
file = '../Model Constraints.txt';
B = importdata(file);
w_vel_c = B.data(1);
w_acc_c = B.data(2);
v_vel_c = B.data(3);
v_acc_c = B.data(4);

%what you measure is just x and y
C = [1 0 0; 0 1 0; 0 0 0];

if currIndex==1
    x{1} = [.2 0 -.1]';
    xRef = zeros(finalIndex,3);
    uRef = zeros(finalIndex,2);
    xhat = x;
else
    x(1:currIndex) = varargin{1}(1:currIndex);
    xRef(1:currIndex,:) = varargin{2}(1:currIndex,:);
    uRef(1:currIndex,:) = varargin{3}(1:currIndex,:);
    xhat = x;
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
x_func_deriv = matlabFunction(diff(x_func));
y_func_deriv = matlabFunction(diff(y_func));

% Detmerine state matrices
syms x1 x2 x3 u1 u2
state = [u1*cos(x3); u1*sin(x3); u2];
A_1 = jacobian(state, [x1, x2, x3]);
B_1 = jacobian(state, [u1, u2]);
A_1_anon = matlabFunction(A_1);
B_1_anon = matlabFunction(B_1);

%% Determine Reference Values

%so covariance matrices are sigma^2*identity matrix (3x3)
noiseV = cell(finalIndex,1);
for i = 1:finalIndex
    noiseV{i} = [normrnd(0,sigmaV); normrnd(0,sigmaV); normrnd(0,sigmaV)];
end

noiseVcovar = sigmaV^2*eye(3);
noiseWcovar = sigmaW^2*eye(3);

noiseW = cell(finalIndex,1);
for i = 1:finalIndex
    noiseW{i} = [normrnd(0,sigmaW); normrnd(0,sigmaW); normrnd(0,sigmaW)];
end

for i = currIndex:finalIndex
    % NOTE: First uRef value is NaN
    % Check if any values are constants and therefore don't take inputs:
    c_check = [nargin(x_func_anon), nargin(y_func_anon), nargin(theta_fun), nargin(theta_fun_deriv)...
                nargin(x_func_deriv), nargin(y_func_deriv)];
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
    
    if c_check(5)==0
        x_deriv_val = x_func_deriv();
    else
        x_deriv_val = x_func_deriv(tin);
    end
    
    if c_check(6)==0
        y_deriv_val = y_func_deriv();
    else
        y_deriv_val = y_func_deriv(tin);
    end
    
    % Now update reference trajectories and feedbacks
    xRef(i,:) = [x_val, y_val, theta_val];
    xRef(isnan(xRef)) = 0;
    uRef(i,:) = [sqrt(x_deriv_val^2 + y_deriv_val^2), theta_deriv_val];
    uRef(isnan(xRef)) = 0;
    
    % Calculate A and B matrices at each time step
    A{i} = eye(3) + deltaT*A_1_anon(uRef(i,1),xRef(i,3));
    B{i} = deltaT*B_1_anon(xRef(i,3));
end

%% Compute LQR Controller
P = cell(finalIndex,1);
P{currIndex} = zeros(3);
if (sigmaV == 0 && sigmaW==0) || (remove_K)
    P(:) = {zeros(3)};
else
    for i = currIndex:finalIndex
        P{i+1} = A{i}*(P{i}-P{i}*transpose(C)*inv(C*P{i}*transpose(C)+noiseWcovar)...
            *C*P{i})*transpose(A{i})+noiseVcovar;
    end
end

S = cell(finalIndex,1);
S{finalIndex} = F;
for i = finalIndex-1:-1:currIndex
S{i} = transpose(A{i})*(S{i+1}-S{i+1}*B{i}*...
    inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1})*A{i}+Q;
end

K = cell(finalIndex,1);
if (sigmaV==0 && sigmaW==0) || (remove_K)
    K(:) = {zeros(3)};
else
    for i = currIndex:finalIndex
    K{i} = P{i}*transpose(C)*inv(C*P{i}*transpose(C)+noiseWcovar);
    end
end

L = cell(finalIndex-1,1);
y = L;
for i = currIndex:finalIndex-1
L{i} = inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1}*A{i};
u{i} = -L{i}*x{i};

% % %%%%%%%%%%%% Incorporate constraints on u:
% % if i==1 || isempty(u{i-1})
% %     prev_vs = [0,0];
% % else
% %     prev_vs = u{i-1};
% % end
% % 
% % % Acceleration Constraints
% % v_acc = (u{i}(1) - prev_vs(1))/deltaT;
% % w_acc = (u{i}(2) - prev_vs(2))/deltaT;
% % if abs(v_acc)>v_acc_c
% %     u{i}(1) = prev_vs(1) + sign(v_acc)*v_acc_c*deltaT;
% % end
% % if abs(w_acc)>w_acc_c
% %     u{i}(2) = prev_vs(2) + sign(w_acc)*w_acc_c*deltaT;
% % end

% Velocity Constraints
%u{i}(2) = sign(u{i}(2)) * min(abs(u{i}(2)), w_vel_c);

% Update position
y{i} = C*x{i}+noiseW{i};
x{i+1} = A{i}*x{i}+B{i}*u{i}+noiseV{i};

% % % % Apply position constraints
% % % Vask = (x{i}(3)-x{i+1}(3) ) / deltaT;
% % % angV = sign(Vask) * min(abs(Vask), w_vel_c );
% % % 
% % % if i==1
% % %     idx = 1;
% % % else
% % %     idx = i-1;
% % % end
% % % x{i+1}(3) = xRef(i+1,3) - (xRef(i,3) - x{i}(3)) + angV*deltaT ; 

end

for i = currIndex:finalIndex-2
xhat{i+1} = A{i}*xhat{i}+B{i}*u{i}+K{i+1}*(y{i+1}-C*(A{i}*xhat{i}+B{i}*u{i}));
end


for i = currIndex:finalIndex-1    
    % Calculate New Position
    xTilda(i) = xhat{i}(1);
    xValues(i) = xTilda(i) + xRef(i,1);
    yTilda(i) = xhat{i}(2);
    yValues(i) = yTilda(i) + xRef(i,2);
    err(i) = sqrt(xTilda(i)^2 + yTilda(i)^2);
end


end