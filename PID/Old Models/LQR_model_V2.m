% Initial Values
finalIndex = 100;
deltaT = 0.1;
Q = eye(3);
F = eye(3);
R = eye(2);
x = cell(finalIndex,1);
x{1} = [1.4 0 pi/8]';

% Calculate A and B matrices
A = cell(finalIndex,1);
B = A;
xRef = zeros(finalIndex,3);
uRef = zeros(finalIndex,2);

%% Determine reference path
syms t y
x_func = cos(t);
y_func = sin(t);

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
for i = 1:finalIndex
    % NOTE: First uRef value is NaN
    xRef(i,:) = [x_func_anon(deltaT*(i-1)),  y_func_anon(deltaT*(i-1)), theta_fun(deltaT*(i-1))];
    xRef(isnan(xRef)) = 0;
    uRef(i,:) = [sqrt(x_func_anon(deltaT*(i-1))^2 + y_func_anon(deltaT*(i-1))^2), theta_fun_deriv(deltaT*(i-1))];
    uRef(isnan(xRef)) = 0;
    A{i} = eval(eye(3) + deltaT*subs(A_1, [x1, x2, x3, u1, u2], [xRef(i,1), xRef(i,2), xRef(i,3), uRef(i,1), uRef(i,2)]));
    B{i} = eval(deltaT*subs(B_1, [x1, x2, x3, u1, u2], [xRef(i,1), xRef(i,2), xRef(i,3), uRef(i,1), uRef(i,2)]));
end
fprintf('Finished A and B\n')

%% Compute LQR Controller
S = cell(finalIndex,1);
S{finalIndex} = F;
for i = finalIndex-1:-1:1
S{i} = transpose(A{i})*(S{i+1}-S{i+1}*B{i}*...
    inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1})*A{i}+Q;
end

L = cell(finalIndex-1,1);
u = L;

for i = 1:finalIndex-1
    L{i} = inv(transpose(B{i})*S{i+1}*B{i}+R)*transpose(B{i})*S{i+1}*A{i};
    u{i} = -L{i}*x{i};
    x{i+1} = A{i}*x{i}+B{i}*u{i};
end

% Calculate positions
xTilda = 1:finalIndex;
yTilda = 1:finalIndex;
xValues = zeros(1, finalIndex);
yValues = xValues;
for i = 1:finalIndex
    xTilda(i) = x{i}(1);
    xValues(i) = xTilda(i) + xRef(i,1);
    yTilda(i) = x{i}(2);
    yValues(i) = yTilda(i) + xRef(i,2);
end

err = sqrt(xTilda.^2 + yTilda.^2);

%% PLOT RESULTS
figure
% Plotting positions
subplot(1,2,1)
plot(xRef(:,1),xRef(:,2), 'LineWidth', 1)
hold on
plot(xValues,yValues, 'LineWidth', 1)
legend('Reference Path', 'Robot Path')
title('Robot Path')
% Plotting Error
subplot(1,2,2)
plot((1:finalIndex)*deltaT, err, 'LineWidth', 1)
title('X-Y Error')
xlabel('Time (s)')
ylabel('Error')
ylim([0, max(err)])

