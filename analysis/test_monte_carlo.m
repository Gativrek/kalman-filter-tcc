%% Monte Carlo Analysis - 100 Runs
% Statistical validation of filter performance

clear; clc; close all;

%% Add paths
addpath('models', 'filters', 'simulation', 'visualization', 'mapping', 'analysis');

%% Parameters
nRuns = 100;  % Number of Monte Carlo runs
fprintf('=== Monte Carlo Analysis: %d runs ===\n', nRuns);
fprintf('Expected time: ~5-10 minutes\n\n');

% Start timer
tic;

%% Initialize storage
results = struct();
results.rmse_KF = zeros(1, nRuns);
results.rmse_EKF = zeros(1, nRuns);
results.rmse_UKF = zeros(1, nRuns);

results.heading_KF = zeros(1, nRuns);
results.heading_EKF = zeros(1, nRuns);
results.heading_UKF = zeros(1, nRuns);

results.final_nees_KF = zeros(1, nRuns);
results.final_nees_EKF = zeros(1, nRuns);
results.final_nees_UKF = zeros(1, nRuns);

%% Run Monte Carlo
for run = 1:nRuns
    % Progress indicator
    if mod(run, 10) == 0
        elapsed = toc;
        fprintf('Run %d/%d... (%.1fs elapsed, ~%.1fs remaining)\n', ...
                run, nRuns, elapsed, elapsed/run * (nRuns-run));
    end
    
    %% Initialize parameters (CRITICAL: different random seed each time!)
    rng(run, 'twister');  % Explicit random seed
    
    params = initParameters();
    params.maxIterations = 100;
    
    % Aggressive scenario (same as test_heat_mapping)
    params.gpsAvailabilityRate = 15;
    params.dt = 0.12;
    params.Q = diag([0.05, 0.05, 0.01]);
    params.R_odom = diag([0.2, 0.1].^2);
    
    %% Initialize states (CRITICAL: use params initial states!)
    xTrue = params.initialTrueState;
    xKF = params.initialEstimateKF;
    xEKF = params.initialEstimateEKF;
    xUKF = params.initialEstimateUKF;
    
    P_KF = params.initialCovariance;
    P_EKF = params.initialCovariance;
    P_UKF = params.initialCovariance;
    
    %% Storage for this run
    errorX_KF = zeros(1, params.maxIterations);
    errorY_KF = zeros(1, params.maxIterations);
    errorTheta_KF = zeros(1, params.maxIterations);
    
    errorX_EKF = zeros(1, params.maxIterations);
    errorY_EKF = zeros(1, params.maxIterations);
    errorTheta_EKF = zeros(1, params.maxIterations);
    
    errorX_UKF = zeros(1, params.maxIterations);
    errorY_UKF = zeros(1, params.maxIterations);
    errorTheta_UKF = zeros(1, params.maxIterations);
    
    nees_KF = zeros(1, params.maxIterations);
    nees_EKF = zeros(1, params.maxIterations);
    nees_UKF = zeros(1, params.maxIterations);
    
    %% Main simulation loop (THIS is where time should be spent!)
    for k = 1:params.maxIterations
        
        %% Control (aggressive figure-8)
        if k < 20
            u = [4.5; 1.2];
        elseif k < 40
            u = [4.5; -1.2];
        elseif k < 60
            u = [4.5; 1.2];
        elseif k < 80
            u = [4.5; -1.2];
        else
            u = [4.5; 1.2];
        end
        
        %% Propagate ground truth
        xTrue = propagateGroundTruth(xTrue, u, params.dt);
        
        %% Generate measurements (WITH NOISE - different each run!)
        zOdom = measurementOdometry(xTrue, u, params.R_odom);
        
        gpsAvailable = (mod(k, params.gpsAvailabilityRate) == 0);
        if gpsAvailable
            zGPS = measurementGPS(xTrue, params.R_gps);
        else
            zGPS = [];
        end
        
        %% Update filters (CRITICAL: This should take most of the time!)
        [xKF, P_KF] = kalmanFilter(xKF, P_KF, zOdom, zGPS, gpsAvailable, ...
                                   params.dt, params.Q, params.R_gps);
        
        [xEKF, P_EKF] = extendedKalmanFilter(xEKF, P_EKF, zOdom, zGPS, gpsAvailable, ...
                                             params.dt, params.Q, params.R_gps);
        
        [xUKF, P_UKF] = unscentedKalmanFilter(xUKF, P_UKF, zOdom, zGPS, gpsAvailable, ...
                                              params.dt, params.Q, params.R_gps, ...
                                              params.ukfAlpha, params.ukfKappa, params.ukfBeta);
        
        %% Compute and store errors
        errorX_KF(k) = xKF(1) - xTrue(1);
        errorY_KF(k) = xKF(2) - xTrue(2);
        errorTheta_KF(k) = rad2deg(angdiff(xKF(3), xTrue(3)));
        
        errorX_EKF(k) = xEKF(1) - xTrue(1);
        errorY_EKF(k) = xEKF(2) - xTrue(2);
        errorTheta_EKF(k) = rad2deg(angdiff(xEKF(3), xTrue(3)));
        
        errorX_UKF(k) = xUKF(1) - xTrue(1);
        errorY_UKF(k) = xUKF(2) - xTrue(2);
        errorTheta_UKF(k) = rad2deg(angdiff(xUKF(3), xTrue(3)));
        
        %% Compute and store NEES
        nees_KF(k) = computeNEES(xTrue, xKF, P_KF);
        nees_EKF(k) = computeNEES(xTrue, xEKF, P_EKF);
        nees_UKF(k) = computeNEES(xTrue, xUKF, P_UKF);
    end
    
    %% Store results for this run
    results.rmse_KF(run) = sqrt(mean(errorX_KF.^2 + errorY_KF.^2));
    results.rmse_EKF(run) = sqrt(mean(errorX_EKF.^2 + errorY_EKF.^2));
    results.rmse_UKF(run) = sqrt(mean(errorX_UKF.^2 + errorY_UKF.^2));
    
    results.heading_KF(run) = sqrt(mean(errorTheta_KF.^2));
    results.heading_EKF(run) = sqrt(mean(errorTheta_EKF.^2));
    results.heading_UKF(run) = sqrt(mean(errorTheta_UKF.^2));
    
    results.final_nees_KF(run) = mean(nees_KF);
    results.final_nees_EKF(run) = mean(nees_EKF);
    results.final_nees_UKF(run) = mean(nees_UKF);
end

totalTime = toc;
fprintf('\nMonte Carlo complete! Total time: %.1f seconds (%.1fs per run)\n', ...
        totalTime, totalTime/nRuns);

%% Sanity check - verify runs are actually different
fprintf('\n=== Sanity Check ===\n');
fprintf('RMSE variance (should be > 0):\n');
fprintf('  KF:  var=%.4f (range: %.3f to %.3f)\n', var(results.rmse_KF), ...
        min(results.rmse_KF), max(results.rmse_KF));
fprintf('  EKF: var=%.4f (range: %.3f to %.3f)\n', var(results.rmse_EKF), ...
        min(results.rmse_EKF), max(results.rmse_EKF));
fprintf('  UKF: var=%.4f (range: %.3f to %.3f)\n', var(results.rmse_UKF), ...
        min(results.rmse_UKF), max(results.rmse_UKF));

%% Compute statistics
fprintf('\n=== Monte Carlo Results (%d runs) ===\n\n', nRuns);

fprintf('RMSE (Position) [m]:\n');
fprintf('  KF:  %.3f ± %.3f  (median: %.3f)\n', ...
        mean(results.rmse_KF), std(results.rmse_KF), median(results.rmse_KF));
fprintf('  EKF: %.3f ± %.3f  (median: %.3f)\n', ...
        mean(results.rmse_EKF), std(results.rmse_EKF), median(results.rmse_EKF));
fprintf('  UKF: %.3f ± %.3f  (median: %.3f)\n', ...
        mean(results.rmse_UKF), std(results.rmse_UKF), median(results.rmse_UKF));

fprintf('\nHeading Error [deg]:\n');
fprintf('  KF:  %.2f ± %.2f  (median: %.2f)\n', ...
        mean(results.heading_KF), std(results.heading_KF), median(results.heading_KF));
fprintf('  EKF: %.2f ± %.2f  (median: %.2f)\n', ...
        mean(results.heading_EKF), std(results.heading_EKF), median(results.heading_EKF));
fprintf('  UKF: %.2f ± %.2f  (median: %.2f)\n', ...
        mean(results.heading_UKF), std(results.heading_UKF), median(results.heading_UKF));

fprintf('\nAverage NEES:\n');
fprintf('  KF:  %.2f ± %.2f\n', mean(results.final_nees_KF), std(results.final_nees_KF));
fprintf('  EKF: %.2f ± %.2f\n', mean(results.final_nees_EKF), std(results.final_nees_EKF));
fprintf('  UKF: %.2f ± %.2f\n', mean(results.final_nees_UKF), std(results.final_nees_UKF));

%% Improvement percentages
improvement_EKF_vs_KF = (mean(results.rmse_KF) - mean(results.rmse_EKF)) / mean(results.rmse_KF) * 100;
improvement_UKF_vs_EKF = (mean(results.rmse_EKF) - mean(results.rmse_UKF)) / mean(results.rmse_EKF) * 100;
improvement_UKF_vs_KF = (mean(results.rmse_KF) - mean(results.rmse_UKF)) / mean(results.rmse_KF) * 100;

fprintf('\nImprovement:\n');
fprintf('  EKF vs KF:  %.1f%% better\n', improvement_EKF_vs_KF);
fprintf('  UKF vs EKF: %.1f%% better\n', improvement_UKF_vs_EKF);
fprintf('  UKF vs KF:  %.1f%% better\n', improvement_UKF_vs_KF);

%% Statistical significance test (t-test)
[~, p_EKF_vs_UKF] = ttest2(results.rmse_EKF, results.rmse_UKF);
[~, p_KF_vs_EKF] = ttest2(results.rmse_KF, results.rmse_EKF);

fprintf('\nStatistical Significance (paired t-test):\n');
fprintf('  EKF vs UKF: p = %.6f %s\n', p_EKF_vs_UKF, ...
        ternary(p_EKF_vs_UKF < 0.05, '(SIGNIFICANT)', '(not significant)'));
fprintf('  KF vs EKF:  p = %.6f %s\n', p_KF_vs_EKF, ...
        ternary(p_KF_vs_EKF < 0.05, '(SIGNIFICANT)', '(not significant)'));

%% Visualization
figure('Name', 'Monte Carlo Results', 'Position', [100, 100, 1200, 800]);

% Box plots
subplot(2, 2, 1);
boxplot([results.rmse_KF', results.rmse_EKF', results.rmse_UKF'], ...
        'Labels', {'KF', 'EKF', 'UKF'}, 'Colors', [1 0.6 0; 0.8 0 0; 0 0 0.8]);
ylabel('RMSE [m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Position Error Distribution', 'Interpreter', 'latex', 'FontWeight', 'bold');
grid on;

subplot(2, 2, 2);
boxplot([results.heading_KF', results.heading_EKF', results.heading_UKF'], ...
        'Labels', {'KF', 'EKF', 'UKF'}, 'Colors', [1 0.6 0; 0.8 0 0; 0 0 0.8]);
ylabel('Heading Error [deg]', 'Interpreter', 'latex', 'FontSize', 11);
title('Heading Error Distribution', 'Interpreter', 'latex', 'FontWeight', 'bold');
grid on;

% Histograms
subplot(2, 2, 3);
hold on; grid on;
histogram(results.rmse_KF, 20, 'FaceColor', [1 0.6 0], 'FaceAlpha', 0.6, 'DisplayName', 'KF');
histogram(results.rmse_EKF, 20, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'DisplayName', 'EKF');
histogram(results.rmse_UKF, 20, 'FaceColor', [0 0 0.8], 'FaceAlpha', 0.6, 'DisplayName', 'UKF');
xlabel('RMSE [m]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Count', 'Interpreter', 'latex', 'FontSize', 11);
title('RMSE Distribution', 'Interpreter', 'latex', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

subplot(2, 2, 4);
hold on; grid on;
histogram(results.final_nees_KF, 20, 'FaceColor', [1 0.6 0], 'FaceAlpha', 0.6, 'DisplayName', 'KF');
histogram(results.final_nees_EKF, 20, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'DisplayName', 'EKF');
histogram(results.final_nees_UKF, 20, 'FaceColor', [0 0 0.8], 'FaceAlpha', 0.6, 'DisplayName', 'UKF');
xlabel('Average NEES', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Count', 'Interpreter', 'latex', 'FontSize', 11);
title('NEES Distribution', 'Interpreter', 'latex', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

fprintf('\nPlots generated!\n');
fprintf('\nDone! Results saved in workspace as ''results'' struct.\n');

%% Helper function
function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end