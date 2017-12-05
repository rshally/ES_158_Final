function error_output = plot_model(x_func, y_func, plt_type)
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
global xRef uRef xhat finalIndex deltaT currIndex xValues yValues err xTilda yTilda 
currIndex = 1;
finalIndex = 100;
deltaT = 0.1;

% Initialize position variables
xTilda = 1:finalIndex-1;
yTilda = 1:finalIndex-1;
err = 1:finalIndex-1;
xValues = zeros(1, finalIndex-1);
yValues = xValues;

% Run function to calculate reference
[xRef, uRef, xhat] = calc_reference_V2(x_func, y_func, currIndex, finalIndex, deltaT);

% Define bottom left coordinate of axis
global zz_pt
zz_pt = [-3,-3];

% Set up figure with limits
f = figure('units','normalized','outerposition',[0 0 1 1]);
set(f,'WindowButtonDownFcn',@clicker)
ax1 = subplot(1,3,1);
ax2 = subplot(1,3,2);
ax3 = subplot(1,3,3);
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
for i = plt_start:finalIndex-1
    currIndex = i;
    % Plot Path
    a = plot(ax1, xRef(1:i,1), xRef(1:i,2),'b-');
    hold(ax1,'on')
    b = plot(ax1, xValues(1:i), yValues(1:i),'r-');
    legend(ax1, 'Reference Path','Robot Path')
    title(ax1, 'Robot Path')
    xlim(ax1, [zz_pt(1), 3])
    ylim(ax1, [zz_pt(2), 3])
    hold(ax1,'off')
    set(a,'HitTest','off')
    set(b,'HitTest','off')
    
    % Make Error Plot
    c = plot(ax2, (0:i-1)*deltaT, err(1:i), 'LineWidth', 1);
    title(ax2, 'X-Y Error')
    xlabel(ax2, 'Time (s)')
    ylabel(ax2, 'Error')
    xlim(ax2, [0,finalIndex*deltaT])
    ylim_curr = get(ax2,'ylim'); 
    ylim_curr(1) = 0;
    set(ax2,'ylim',ylim_curr)
    set(c,'HitTest','off')
    
    % Integrate error and return metric
    if i>1
        error_output = cumtrapz((0:i-1)*deltaT, err(1:i));
    else
        error_output = 0;
    end
    plot(ax3, (0:i-1)*deltaT, error_output, 'LineWidth', 1);
    title(ax3, 'Integration of Error')
    xlabel(ax3, 'Time (s)')
    ylabel(ax3, 'Error Accumulation')
    xlim(ax3, [0,finalIndex*deltaT])
    ylim_curr = get(ax3,'ylim'); 
    ylim_curr(1) = 0;
    set(ax3,'ylim',ylim_curr)
    
    % Pause before showing next time step
    pause(0.05)
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
    [xRef, uRef, xhat] = calc_reference_V2(x_func, y_func, currIndex, finalIndex, deltaT, xhat, xRef, uRef);

