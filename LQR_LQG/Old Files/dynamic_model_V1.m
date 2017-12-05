function dynamic_model_V1(x_func, y_func)
% Dynamic LQR function for real-time updating
fprintf('Entering Dynamic LQR Model\n')

% Create global variables
global xRef uRef xhat finalIndex deltaT currIndex

% Run the calculate reference function
currIndex = 1;
finalIndex = 100;
deltaT = 0.1;
[xRef, uRef, xhat] = calc_reference_V2(x_func, y_func, currIndex, finalIndex, deltaT);

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
f = figure('Position', [1600, 20, 1000, 600]);
set(f,'WindowButtonDownFcn',@clicker)
ax1 = subplot(1,2,1);
set(ax1,'ButtonDownFcn','disp(''axis callback'')')

ax2 = subplot(1,2,2);
for i = 1:finalIndex-1    
    % Calculate New Position
    xTilda(i) = xhat{i}(1);
    xValues(i) = xTilda(i) + xRef(i,1);
    yTilda(i) = xhat{i}(2);
    yValues(i) = yTilda(i) + xRef(i,2);
    err(i) = sqrt(xTilda(i)^2 + yTilda(i)^2);
end

for i = 1:finalIndex - 1
    currIndex = i;
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
    ylim_curr = get(ax2,'ylim'); 
    ylim_curr(1) = 0;
    set(ax2,'ylim',ylim_curr)
    set(c,'HitTest','off')
    pause(0.05)
    
    % Now: Check if reference is updated with a click or something:
end

function clicker(src, eventdata)
    global xRef uRef  xhat finalIndex deltaT currIndex
    
    % Get clicked pixel locations
    P = get(gca, 'CurrentPoint');
    x_pt = P(1,1);
    y_pt = P(1,2);
    
    % Create a new reference line that reaches target at the end of time
    slope = (y_pt - xRef(currIndex,2)) / (x_pt - xRef(currIndex,1));
    syms t
    x_func = xRef(currIndex,1) + (t - (currIndex-1)*deltaT) / (((finalIndex-1) - (currIndex-1))*deltaT) * (x_pt - xRef(currIndex,1));
    y_func = slope*(x_func - x_pt) + y_pt + 0*t^3; 
    [xRef, uRef, xhat] = calc_reference_V2(x_func, y_func, currIndex, finalIndex, deltaT, xhat, xRef, uRef);

