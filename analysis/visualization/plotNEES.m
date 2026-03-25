function plotNEES(nees_KF, nees_EKF, nees_UKF, iteration, n)
% Plots NEES with confidence bounds
% Shows if filters are statistically consistent
%
% INPUTS:
%   nees_KF/EKF/UKF - NEES history vectors
%   iteration       - Current iteration
%   n               - State dimension (usually 3)

cla; hold on; grid on;

% Chi-squared 95% confidence bounds for n degrees of freedom
% For consistent filter: NEES should be between these bounds ~95% of time
alpha = 0.05;
r1 = chi2inv(alpha/2, n) / 1;      % Lower bound (per sample)
r2 = chi2inv(1-alpha/2, n) / 1;    % Upper bound (per sample)

% Plot confidence bounds
plot([1, iteration], [r1, r1], 'k--', 'LineWidth', 1, 'DisplayName', '95% Bounds');
plot([1, iteration], [r2, r2], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
fill([1, iteration, iteration, 1], [r1, r1, r2, r2], ...
     [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, ...
     'DisplayName', 'Consistency Region');

% Plot NEES
plot(1:iteration, nees_KF(1:iteration), '-', 'Color', [1 0.6 0], ...
     'LineWidth', 1.5, 'DisplayName', 'KF');
plot(1:iteration, nees_EKF(1:iteration), '-', 'Color', [0.8 0 0], ...
     'LineWidth', 1.5, 'DisplayName', 'EKF');
plot(1:iteration, nees_UKF(1:iteration), '-', 'Color', [0 0 0.8], ...
     'LineWidth', 1.5, 'DisplayName', 'UKF');

xlabel('Iteration', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('NEES', 'Interpreter', 'latex', 'FontSize', 10);
title('NEES - Consistency Check', 'Interpreter', 'latex', ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);
ylim([0, max([r2*2, max([nees_KF(1:iteration), nees_EKF(1:iteration), nees_UKF(1:iteration)])])]);

% Add statistics text
if iteration > 10
    inside_KF = sum(nees_KF(1:iteration) >= r1 & nees_KF(1:iteration) <= r2) / iteration * 100;
    inside_EKF = sum(nees_EKF(1:iteration) >= r1 & nees_EKF(1:iteration) <= r2) / iteration * 100;
    inside_UKF = sum(nees_UKF(1:iteration) >= r1 & nees_UKF(1:iteration) <= r2) / iteration * 100;
    
    stats_text = sprintf(['Consistency (should be ~95%%):\n' ...
                         'KF:  %.1f%%\n' ...
                         'EKF: %.1f%%\n' ...
                         'UKF: %.1f%%'], ...
                         inside_KF, inside_EKF, inside_UKF);
    
    text(0.02, 0.98, stats_text, ...
         'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontSize', 9, 'BackgroundColor', [1 1 1 0.9], ...
         'EdgeColor', 'black', 'Margin', 3);
end

end