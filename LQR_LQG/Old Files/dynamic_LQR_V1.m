function dynamic_LQR(x_func, y_func)
fprintf('Entering Dynamic LQR Model\n')



%% Initial Values
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
err = 1:finalIndex;
xValues = zeros(1, finalIndex);
yValues = xValues;

% Define bottom left coordinate of axis
global zz_pt
zz_pt = [-3,-3];

% Set up figure with limits
f = figure;
set(f,'WindowButtonDownFcn',@clicker)
ax1 = subplot(1,2,1);
set(ax1,'ButtonDownFcn','disp(''axis callback'')')

ax2 = subplot(1,2,2);
for i = 1:finalIndex
    % Calculate New Position
    xTilda(i) = x{i}(1);
    xValues(i) = xTilda(i) + xRef(i,1);
    yTilda(i) = x{i}(2);
    yValues(i) = yTilda(i) + xRef(i,2);
    err(i) = sqrt(xTilda(i)^2 + yTilda(i)^2);

    % Plot Path
    a = plot(ax1, xRef(1:i,1), xRef(1:i,2),'b-');
    hold(ax1,'on')
    b = plot(ax1, xValues(1:i), yValues(1:i),'r-');
    legend(ax1, 'Reference Path','Robot Path')
    title('Robot Path')
    xlim(ax1, [zz_pt(1), 3])
    ylim(ax1, [zz_pt(2), 3])
    hold(ax1,'off')
    set(a,'HitTest','off')
    set(b,'HitTest','off')
    
    % Make Error Plot
    c = plot(ax2, (1:i)*deltaT, err(1:i), 'LineWidth', 1);
    title('X-Y Error')
    xlabel('Time (s)')
    ylabel('Error')
    xlim(ax2, [0,finalIndex*deltaT])
    ylim_curr = get(gca,'ylim'); 
    ylim_curr(1) = 0;
    set(gca,'ylim',ylim_curr)
    set(c,'HitTest','off')
    pause(0.05)
    
    % Now: Check if reference is updated with a click or something:
end
function clicker(src,eventdata)
    global zz_pt
    
    % Get clicked pixel locations
    P = get(src, 'CurrentPoint');
    
    % Convert pixel locations to x-y coordinates
    R = makerefmat(zz_pt(1), zz_pt(2), .01, .01);
    [x,y] = pix2map(R,P(1),P(2));
    
    
    % NOW WHAT TO DO: MAKE THE NEW REFERENCE TRAJECTOR A LINE TO THIS
    % POINT!!!!!!!!!
    % ALSO: CHECK TO SEE IF THE POINT IS EVEN IN THE BOUNDS
    % ALSO: CHECK TO MAKE SURE THE MAPPING OF PIXELS ACTUALLY WORKS

