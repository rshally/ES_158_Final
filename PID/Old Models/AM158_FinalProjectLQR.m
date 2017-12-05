% Initial Values
finalIndex = 80;
deltaT = 0.1;
Q = eye(3);
F = eye(3);
R = eye(2);
x = cell(finalIndex,1);
x{1} = [1.4 0 pi/8]';

% Calculate A and B matrices
A = cell(finalIndex,1);
B = A;
xRef = cell(finalIndex,1);
uRef = x_ref;

% Determine Reference 
x_func_anon = @(T) cos(T);
y_func_anon = @(T) sin(T);
syms t y
x_func = cos(t);
y_func = sin(t);
theta_fun_sym = atan2(diff(y_func), diff(x_func));
theta_fun_deriv_sym = diff(theta_fun_sym);
theta_fun = matlabFunction(theta_fun_sym);
theta_fun_deriv = matlabFunction(theta_fun_deriv_sym);

% Determine Reference Values
for i = 1:finalIndex
    % NOTE: First uRef value is NaN
    xRef(i,:) = [x_func_anon(deltaT*(i-1)),  y_func_anon(deltaT*(i-1)), theta_fun(deltaT*(i-1))];
    uRef(i,:) = [sqrt(x_func_anon(deltaT*(i-1))^2 + y_func_anon(deltaT*(i-1))^2), theta_fun_deriv(deltaT*(i-1))];
end
xRef(isnan(xRef)) = 0;
uRef(isnan(uRef)) = 0;


%%%%%%%%%%%%%%% I'm here

for i = 1:finalIndex
    
    
    %%%%% OLD
    A_1 = [0 0 -cos((i-1)*deltaT); 0 0 -sin((i-1)*deltaT); 0 0 0];
    A{i} = eye(3) + deltaT*A_1;
    B_1 = [-sin((i-1)*deltaT) 0; cos((i-1)*deltaT) 0; 0 1];
    B{i} = deltaT*B_1;
end


% Compute LQR Controller
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
for i = 1:finalIndex
    xTilda(i) = x{i}(1);
    yTilda(i) = x{i}(2);
end

%Because, e.g. x tilda = x-xRef
xValues = xTilda+xRef;
yValues = yTilda+yRef;
err = sqrt(xTilda.^2 + yTilda.^2);

% PLOT
figure
% Plotting positions
subplot(1,2,1)
plot(xRef,yRef)
hold on
plot(xValues,yValues)
legend('Reference Path', 'Robot Path')
title('Robot Path')
% Plotting Error
subplot(1,2,2)
plot((1:finalIndex)*deltaT, err)
title('X-Y Error')
xlabel('Time (s)')
ylabel('Error')

