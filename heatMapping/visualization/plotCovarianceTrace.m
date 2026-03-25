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
window = 5;
trace_KF_smooth = movmean(trace_KF(1:iteration), window);
trace_EKF_smooth = movmean(trace_EKF(1:iteration), window);
trace_UKF_smooth = movmean(trace_UKF(1:iteration), window);

plot(1:iteration, trace_KF_smooth, '-', 'Color', params.colorKF, 'LineWidth', 2, 'DisplayName', 'KF');
plot(1:iteration, trace_EKF_smooth, '-', 'Color', params.colorEKF, 'LineWidth', 2, 'DisplayName', 'EKF');
plot(1:iteration, trace_UKF_smooth, '-', 'Color', params.colorUKF, 'LineWidth', 2, 'DisplayName', 'UKF');

xlabel('Iteration');
ylabel('trace(P)');
title('Covariance Evolution', 'FontWeight', 'bold');
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
    ylim([min_trace - 0.1, max_trace + 0.1]);
end


end