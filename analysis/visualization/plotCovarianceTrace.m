function plotCovarianceTrace(trace_KF, trace_EKF, trace_UKF, iteration, params)
% Plots trace of covariance matrix over time for all filters
% Shows convergence and uncertainty evolution
%
% INPUTS:
%   trace_KF/EKF/UKF - Trace history vectors
%   iteration        - Current iteration
%   params           - Parameters (for colors)

cla; hold on; grid on;

% Apply moving average for smoother visualization
window = 5;  % 5-point moving average
trace_KF_smooth = movmean(trace_KF(1:iteration), window);
trace_EKF_smooth = movmean(trace_EKF(1:iteration), window);
trace_UKF_smooth = movmean(trace_UKF(1:iteration), window);

plot(1:iteration, trace_KF_smooth, '-', 'Color', params.colorKF, ...
     'LineWidth', 2, 'DisplayName', 'KF');
plot(1:iteration, trace_EKF_smooth, '-', 'Color', params.colorEKF, ...
     'LineWidth', 2, 'DisplayName', 'EKF');
plot(1:iteration, trace_UKF_smooth, '-', 'Color', params.colorUKF, ...
     'LineWidth', 2, 'DisplayName', 'UKF');

xlabel('Iteration', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('trace($P$)', 'Interpreter', 'latex', 'FontSize', 10);
title('Covariance Evolution', 'Interpreter', 'latex', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

% Set Y-axis limits safely
all_traces = [trace_KF(1:iteration), trace_EKF(1:iteration), trace_UKF(1:iteration)];
min_trace = min(all_traces);
max_trace = max(all_traces);

% Only set log scale and limits if data is valid
if min_trace > 0 && max_trace > min_trace
    set(gca, 'YScale', 'log');
    ylim([min_trace * 0.5, max_trace * 2]);
elseif max_trace > min_trace
    % Linear scale if min is zero or negative
    ylim([min_trace - 0.1, max_trace + 0.1]);
end

%% Add statistics box
if iteration > 10  % Only show after some iterations
    % Calculate statistics
    mean_KF = mean(trace_KF(1:iteration));
    mean_EKF = mean(trace_EKF(1:iteration));
    mean_UKF = mean(trace_UKF(1:iteration));
    
    std_KF = std(trace_KF(1:iteration));
    std_EKF = std(trace_EKF(1:iteration));
    std_UKF = std(trace_UKF(1:iteration));
    
    final_KF = trace_KF(iteration);
    final_EKF = trace_EKF(iteration);
    final_UKF = trace_UKF(iteration);
    
    % Create statistics text
    stats_text = sprintf(['\\bf Statistics:\\rm\n' ...
                         '\\color[rgb]{%.2f,%.2f,%.2f}KF:\\color{black}  mean=%.3f  std=%.3f  final=%.3f\n' ...
                         '\\color[rgb]{%.2f,%.2f,%.2f}EKF:\\color{black} mean=%.3f  std=%.3f  final=%.3f\n' ...
                         '\\color[rgb]{%.2f,%.2f,%.2f}UKF:\\color{black} mean=%.3f  std=%.3f  final=%.3f'], ...
                         params.colorKF(1), params.colorKF(2), params.colorKF(3), ...
                         mean_KF, std_KF, final_KF, ...
                         params.colorEKF(1), params.colorEKF(2), params.colorEKF(3), ...
                         mean_EKF, std_EKF, final_EKF, ...
                         params.colorUKF(1), params.colorUKF(2), params.colorUKF(3), ...
                         mean_UKF, std_UKF, final_UKF);
    
    % Add text box
    text(0.98, 0.35, stats_text, ...
         'Units', 'normalized', ...
         'VerticalAlignment', 'top', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 8, ...
         'FontName', 'FixedWidth', ...
         'BackgroundColor', [1 1 1 0.9], ...
         'EdgeColor', 'black', ...
         'Margin', 5);
end

end