function static_model_V1(x_func, y_func)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Static LQR function for full stage processing

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Values
currIndex = 1;
finalIndex = 100;
deltaT = 0.1;

[xRef, uRef, xhat] = calc_reference_V2(x_func, y_func, currIndex, finalIndex, deltaT);

% Calculate positions
xTilda = 1:finalIndex-1;
yTilda = 1:finalIndex-1;
xValues = zeros(1, finalIndex-1);
yValues = xValues;
for i = 1:finalIndex-1
    xTilda(i) = xhat{i}(1);
    xValues(i) = xTilda(i) + xRef(i,1);
    yTilda(i) = xhat{i}(2);
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
plot((1:finalIndex-1)*deltaT, err, 'LineWidth', 1)
title('X-Y Error')
xlabel('Time (s)')
ylabel('Error')
ylim([0, max(err)])

end

