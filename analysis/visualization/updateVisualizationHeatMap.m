function handles = updateVisualizationHeatMap(handles, xTrue, xKF, xEKF, xUKF, P_KF, P_EKF, P_UKF, ...
                                             trajTrue, trajKF, trajEKF, trajUKF, iteration, params, ...
                                             heatMap_KF, heatMap_EKF, heatMap_UKF)
% Updates visualization with trajectories, heat maps, and statistics
% VERSION TCC: Includes GPS indicator and covariance trace
%
% INPUTS:
%   handles   - Structure with graphics handles
%   xTrue     - True state
%   xKF/EKF/UKF - Filter estimates
%   P_KF/EKF/UKF - Filter covariances
%   trajTrue/KF/EKF/UKF - Trajectory histories
%   iteration - Current iteration
%   params    - Parameters structure
%   heatMap_KF/EKF/UKF - Heat maps

%% Top Row: Trajectory (left) and Covariance Trace (right)

% Trajectory plot (top left, spanning 2 columns)
subplot(2, 4, [1, 2]);
cla; hold on;
grid on;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Robot Trajectories and Uncertainty', 'Interpreter', 'latex', ...
      'FontSize', 12, 'FontWeight', 'bold');

% Plot trajectories
plot(trajTrue(1, :), trajTrue(2, :), '-', 'Color', params.colorTrue, 'LineWidth', 2.5);
plot(trajKF(1, :), trajKF(2, :), '--', 'Color', params.colorKF, 'LineWidth', 1.8);
plot(trajEKF(1, :), trajEKF(2, :), '--', 'Color', params.colorEKF, 'LineWidth', 1.8);
plot(trajUKF(1, :), trajUKF(2, :), '--', 'Color', params.colorUKF, 'LineWidth', 1.8);

% Plot uncertainty ellipses
plotUncertaintyEllipse(xKF(1:2), P_KF(1:2, 1:2), params.colorKF, 2);
plotUncertaintyEllipse(xEKF(1:2), P_EKF(1:2, 1:2), params.colorEKF, 2);
plotUncertaintyEllipse(xUKF(1:2), P_UKF(1:2, 1:2), params.colorUKF, 2);

% Draw robots
createRobotBody(xTrue(1), xTrue(2), xTrue(3), params.robotBodyLength, ...
                params.robotBodyWidth, params.colorTrue, '-');
createRobotBody(xKF(1), xKF(2), xKF(3), params.robotBodyLength, ...
                params.robotBodyWidth, params.colorKF, '--');
createRobotBody(xEKF(1), xEKF(2), xEKF(3), params.robotBodyLength, ...
                params.robotBodyWidth, params.colorEKF, '--');
createRobotBody(xUKF(1), xUKF(2), xUKF(3), params.robotBodyLength, ...
                params.robotBodyWidth, params.colorUKF, '--');

% Dynamic axis limits
allX = [trajTrue(1,:), trajKF(1,:), trajEKF(1,:), trajUKF(1,:)];
allY = [trajTrue(2,:), trajKF(2,:), trajEKF(2,:), trajUKF(2,:)];
margin = 2;
xlim([min(allX)-margin, max(allX)+margin]);
ylim([min(allY)-margin, max(allY)+margin]);
axis equal;

legend('True', 'KF', 'EKF', 'UKF', 'Location', 'best', 'FontSize', 9);

% GPS Availability Indicator
gpsAvailable = (mod(iteration, params.gpsAvailabilityRate) == 0);
if gpsAvailable
    gpsColor = [0 0.8 0];  % Green
    gpsText = 'GPS: ON';
else
    gpsColor = [0.5 0.5 0.5];  % Gray
    gpsText = 'GPS: OFF';
end

text(0.02, 0.98, sprintf('Iter: %d/%d', iteration, params.maxIterations), ...
     'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 9, ...
     'BackgroundColor', 'white', 'EdgeColor', 'black');

text(0.02, 0.90, gpsText, ...
     'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 9, ...
     'BackgroundColor', 'white', 'EdgeColor', 'black', 'Color', gpsColor, ...
     'FontWeight', 'bold');

%% Covariance Trace (top right, column 3)
subplot(2, 4, 3);
plotCovarianceTrace(handles.trace_KF, handles.trace_EKF, handles.trace_UKF, ...
                    iteration, params);

%% NEES Plot (top right, column 4) - NEW!
subplot(2, 4, 4);
plotNEES(handles.nees_KF, handles.nees_EKF, handles.nees_UKF, iteration, 3);

%% Bottom Row: Heat Maps
% KF Heat Map
subplot(2, 4, 5);
imagesc([params.heatMapXLim(1), params.heatMapXLim(2)], ...
        [params.heatMapYLim(1), params.heatMapYLim(2)], ...
        heatMap_KF / max(heatMap_KF(:)));
set(gca, 'YDir', 'normal');
colormap(gca, hot);
clim([0, 1]);
axis([-30 10 -5 30]);
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
title('KF: Path Certainty', 'Interpreter', 'latex', 'FontSize', 10, ...
      'FontWeight', 'bold', 'Color', params.colorKF);
grid on;

% EKF Heat Map
subplot(2, 4, 6);
imagesc([params.heatMapXLim(1), params.heatMapXLim(2)], ...
        [params.heatMapYLim(1), params.heatMapYLim(2)], ...
        heatMap_EKF / max(heatMap_EKF(:)));
set(gca, 'YDir', 'normal');
colormap(gca, hot);
clim([0, 1]);
axis([-30 10 -5 30]);
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
title('EKF: Path Certainty', 'Interpreter', 'latex', 'FontSize', 10, ...
      'FontWeight', 'bold', 'Color', params.colorEKF);
grid on;

% UKF Heat Map
subplot(2, 4, 7);
imagesc([params.heatMapXLim(1), params.heatMapXLim(2)], ...
        [params.heatMapYLim(1), params.heatMapYLim(2)], ...
        heatMap_UKF / max(heatMap_UKF(:)));
set(gca, 'YDir', 'normal');
colormap(gca, hot);
clim([0, 1]);
axis([-30 10 -5 30]);
colorbar;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 9);
title('UKF: Path Certainty', 'Interpreter', 'latex', 'FontSize', 10, ...
      'FontWeight', 'bold', 'Color', params.colorUKF);
grid on;

% Force rendering
drawnow limitrate;

end