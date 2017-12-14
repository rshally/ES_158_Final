function cost = plot_model(x_func, y_func, time, plt_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Entering Calculation Mode\n')
% INPUTS: x_func   --> parameterized representation of the x function
%         y_func   --> parameterized representation of the y function
%         plt_type --> static or dynamic plot specification



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTES:
%
% Creating global variables --> I think they need to be global variables
% because of the feature of click-updating of reference path in real-time
% mode. Otherwise, I'm not sure how to pass in the current index of the
% plotting to the clicker function.....if we think of a way to do this, we
% can improve the efficiency by changing all of these global variables to
% function input-output parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global xRef uRef xhat finalIndex deltaT currIndex xValues yValues err xTilda yTilda u
currIndex = 1;
deltaT = 0.1;
finalIndex = time/deltaT - currIndex;

% Initialize position variables
xTilda = 1:finalIndex-1;
yTilda = 1:finalIndex-1;
err = 1:finalIndex-1;
xValues = zeros(1, finalIndex-1);
yValues = xValues;
u = cell(1, finalIndex-1);

% Run function to calculate reference
[xRef, uRef, xhat] = calc_reference_V3(x_func, y_func, currIndex, finalIndex, deltaT);

% Set up figure with limits
f = figure('units','normalized','outerposition',[0 0 1 1]);
set(f,'WindowButtonDownFcn',@clicker)
ax1 = subplot(1,2,1);
ax2 = subplot(1,2,2);
%ax3 = subplot(1,1,1);
set(ax1,'ButtonDownFcn','disp(''axis callback'')')

% Switch-case for determining if plot method is static or dynamic
switch plt_type
    case 'static'
        plt_start = finalIndex-1;
    case 'dynamic'
        plt_start = 1;
end

% Run loop to do the plotting while keeping the ability for real-time
% updating possible
a1 = -1.5;
b1 = 1.5;
c1 = -1.5;
d1 = 1.5;
for i = plt_start:finalIndex-1
    currIndex = i;
    % Plot Path
    x_circ = cos(0:.01:(2*pi));
    y_circ = sin(0:.01:(2*pi));
    a = plot(ax1, x_circ, y_circ, 'b--', 'LineWidth', 3);
    %a = plot(ax1, xRef(1:i,1), xRef(1:i,2),'b--', 'LineWidth', 3);
    hold(ax1,'on')
    b = plot(ax1, xValues(1:i), yValues(1:i),'r-', 'LineWidth', 1);
    legend(ax1, 'Reference Path','Robot Path')
    title(ax1, 'Robot Trajectory')
    xlim(ax1, [a1, b1])
    ylim(ax1, [c1, d1])
    hold(ax1,'off')
    set(a,'HitTest','off')
    set(b,'HitTest','off')
    
    % Make Error Plot
    c = plot(ax2, (0:i-1)*deltaT, err(1:i), 'LineWidth', 1);
    title(ax2, 'X-Y Position Error')
    xlabel(ax2, 'Time (s)')
    ylabel(ax2, 'Error')
    xlim(ax2, [0,finalIndex*deltaT])
    ylim_curr = get(ax2,'ylim'); 
    ylim_curr(1) = 0;
    set(ax2,'ylim',ylim_curr)
    set(c,'HitTest','off')
    
    cost = calc_cost(xhat, u);

    % Pause before showing next time step
    pause(0.1)
end
if strcmp(plt_type,'static')
    fprintf('Cost:\n')
    disp(cost)
end

function clicker(~, ~)
    % This is the function to update the trajectory by clicking anywhere in
    % the axis to specify a new reference point
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
    [xRef, uRef, xhat] = calc_reference_V3(x_func, y_func, currIndex, finalIndex, deltaT, xhat, xRef, uRef);

