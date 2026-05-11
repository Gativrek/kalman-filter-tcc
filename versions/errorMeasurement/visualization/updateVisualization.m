function handles = updateVisualization(handles, xTrue, xKF, xEKF, xUKF, P_KF, P_EKF, P_UKF, trajTrue, trajKF, trajEKF, trajUKF, iteration, params)
% Updates all visualization elements with DYNAMIC AXES
%
% INPUTS:
%   handles   - Structure with graphics handles
%   xTrue     - True state
%   xKF/EKF/UKF - Filter estimates
%   P_KF/EKF/UKF - Filter covariances
%   trajTrue/KF/EKF/UKF - Trajectory histories
%   iteration - Current iteration
%   params    - Parameters structure

%% Main plot (top view)
nexttile(handles.tiles, 1);
cla; hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Robot Trajectory and Uncertainty', 'FontWeight', 'bold');

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
createRobotBody(xTrue(1), xTrue(2), xTrue(3), params.robotBodyLength, params.robotBodyWidth, params.colorTrue, '-');
createRobotBody(xKF(1), xKF(2), xKF(3), params.robotBodyLength, params.robotBodyWidth, params.colorKF, '--');
createRobotBody(xEKF(1), xEKF(2), xEKF(3), params.robotBodyLength, params.robotBodyWidth, params.colorEKF, '--');
createRobotBody(xUKF(1), xUKF(2), xUKF(3), params.robotBodyLength, params.robotBodyWidth, params.colorUKF, '--');

% Dynamic axis limits
allX = [trajTrue(1,:), trajKF(1,:), trajEKF(1,:), trajUKF(1,:)];
allY = [trajTrue(2,:), trajKF(2,:), trajEKF(2,:), trajUKF(2,:)];

margin = 2;
xlim([min(allX) - margin, max(allX) + margin]);
ylim([min(allY) - margin, max(allY) + margin]);
axis equal;

legend('True', 'KF', 'EKF', 'UKF', 'Location', 'best', 'FontSize', 10);

% Add iteration counter
text(0.02, 0.98, sprintf('Iteration: %d/%d', iteration, params.maxIterations), 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');

%% Position error plots
nexttile(handles.tiles, 2);
cla; hold on; grid on;
plot(1:iteration, handles.errorX_KF(1:iteration), '-', 'Color', params.colorKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorX_EKF(1:iteration), '-', 'Color', params.colorEKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorX_UKF(1:iteration), '-', 'Color', params.colorUKF, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('x Error [m]');
title('X Position Error', 'FontWeight', 'bold');
xlim([1, params.maxIterations]);

nexttile(handles.tiles, 3);
cla; hold on; grid on;
plot(1:iteration, handles.errorY_KF(1:iteration), '-', 'Color', params.colorKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorY_EKF(1:iteration), '-', 'Color', params.colorEKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorY_UKF(1:iteration), '-', 'Color', params.colorUKF, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('y Error [m]');
title('Y Position Error', 'FontWeight', 'bold');
xlim([1, params.maxIterations]);

nexttile(handles.tiles, 4);
cla; hold on; grid on;
plot(1:iteration, handles.errorTheta_KF(1:iteration), '-', 'Color', params.colorKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorTheta_EKF(1:iteration), '-', 'Color', params.colorEKF, 'LineWidth', 1.5);
plot(1:iteration, handles.errorTheta_UKF(1:iteration), '-', 'Color', params.colorUKF, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('$\theta$ Error [deg]', 'Interpreter', 'latex');
title('Heading Error', 'FontWeight', 'bold');
legend('KF', 'EKF', 'UKF', 'Location', 'best', 'FontSize', 9);
xlim([1, params.maxIterations]);

% Force rendering
drawnow limitrate;

end