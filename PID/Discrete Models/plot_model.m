function plot_model( t_all, x, xy_err, plt_type)
% Plots the PID model staticly or dynamicly

finalIdx = length(t_all);
switch plt_type
    case 'static'
        currIdx = finalIdx;
    case 'dynamic'
        currIdx = 1;
end

% Plot in a loop
% Plotting results
figure('units','normalized','outerposition',[0 0 1 1]);
s1 = subplot(1,2,1);
hold on
s2 = subplot(1,2,2);

for i = currIdx:1:finalIdx
    plot(s1, cos(0:.1:(2*pi)), sin(0:.1:(2*pi)),'b--','LineWidth', 3)
    plot(s1, x(1,1:i), x(2,1:i), 'r-', 'LineWidth', 1)
    title(s1, 'Robot Trajectory')
    legend(s1, 'Reference', 'Robot')
    xlim(s1, [min(x(1,:))-.5, max(x(1,:))+.5])
    ylim(s1, [min(x(2,:))-.5, max(x(2,:))+.5])

    % Plotting error
    subplot(1,2,2)
    plot(s2, t_all(1:i), xy_err(1:i), 'LineWidth', 1)
    title(s2, 'X-Y Position Error')
    xlabel(s2, 'Time (s)')
    ylabel(s2, 'Error')
    xlim(s2,[0, 10])
    ylim(s2, [0, max(xy_err)+.1])
    
    pause(0.01)
end

end

