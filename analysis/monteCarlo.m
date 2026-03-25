%% Monte Carlo Analysis
clear; clc; close all;

%% Add paths
addpath('models', 'filters', 'simulation', 'visualization', 'mapping', 'analysis');

%% Parameters
seed = 22011;
rng(seed)

nRuns = 1000;
%% Initialize storage
results = struct();
results.rmse_KF = zeros(1, nRuns);
results.rmse_EKF = zeros(1, nRuns);
results.rmse_UKF = zeros(1, nRuns);

results.heading_KF = zeros(1, nRuns);
results.heading_EKF = zeros(1, nRuns);
results.heading_UKF = zeros(1, nRuns);

% Metrics
results.final_nees_KF = zeros(1, nRuns);
results.final_nees_EKF = zeros(1, nRuns);
results.final_nees_UKF = zeros(1, nRuns);

results.nees_KF_all = [];
results.nees_EKF_all = [];
results.nees_UKF_all = [];

results.time_KF = zeros(1, nRuns);
results.time_EKF = zeros(1, nRuns);
results.time_UKF = zeros(1, nRuns);

%% Run Monte Carlo
for run = 1:nRuns
    %% Initialize parameters
    rng(run, 'twister');
    params = initParameters();

    %% Initialize states
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
    
    %% Main simulation loop
    for k = 1:params.maxIterations
        
        %% Control (aggressive figure-8)
        if k < 20
            u = [5.0; 1.5];        
        elseif k < 40
            u = [5.0; -1.5];       
        elseif k < 60
            u = [7.5; 2];
        elseif k < 80
            u = [7.5; -2];
        else
            u = [2.5; 1];
        end
            
        %% Propagate ground truth
        xTrue = propagateGroundTruth(xTrue, u, params.dt);
        
        %% Generate measurements
        zOdom = measurementOdometry(u, params.R_odom);
        
        gpsAvailable = (mod(k, params.gpsAvailabilityRate) == 0);
        if gpsAvailable
            zGPS = measurementGPS(xTrue, params.R_gps);
        else
            zGPS = [];
        end
        
        %% Update filters 
        tic;
        [xKF, P_KF] = kalmanFilter(xKF, P_KF, zOdom, zGPS, gpsAvailable, params);
        results.time_KF(run) = results.time_KF(run) + toc;

        tic;
        [xEKF, P_EKF] = extendedKalmanFilter(xEKF, P_EKF, zOdom, zGPS, gpsAvailable, params);
        results.time_EKF(run) = results.time_EKF(run) + toc;
        
        tic;
        [xUKF, P_UKF] = unscentedKalmanFilter(xUKF, P_UKF, zOdom, zGPS, gpsAvailable, params);
        results.time_UKF(run) = results.time_UKF(run) + toc;
        
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

    results.nees_KF_all = [results.nees_KF_all, nees_KF];
    results.nees_EKF_all = [results.nees_EKF_all, nees_EKF];
    results.nees_UKF_all = [results.nees_UKF_all, nees_UKF];
end

%% Compute statistics
fprintf('\nRMSE (Position) [m]:\n');
fprintf('  KF:  %.3f ± %.3f  (median: %.3f)\n', mean(results.rmse_KF), std(results.rmse_KF), median(results.rmse_KF));
fprintf('  EKF: %.3f ± %.3f  (median: %.3f)\n', mean(results.rmse_EKF), std(results.rmse_EKF), median(results.rmse_EKF));
fprintf('  UKF: %.3f ± %.3f  (median: %.3f)\n', mean(results.rmse_UKF), std(results.rmse_UKF), median(results.rmse_UKF));

fprintf('\nHeading Error [deg]:\n');
fprintf('  KF:  %.2f ± %.2f  (median: %.2f)\n', mean(results.heading_KF), std(results.heading_KF), median(results.heading_KF));
fprintf('  EKF: %.2f ± %.2f  (median: %.2f)\n', mean(results.heading_EKF), std(results.heading_EKF), median(results.heading_EKF));
fprintf('  UKF: %.2f ± %.2f  (median: %.2f)\n', mean(results.heading_UKF), std(results.heading_UKF), median(results.heading_UKF));

fprintf('\nAverage NEES:\n');
fprintf('  KF:  %.2f ± %.2f\n', mean(results.final_nees_KF), std(results.final_nees_KF));
fprintf('  EKF: %.2f ± %.2f\n', mean(results.final_nees_EKF), std(results.final_nees_EKF));
fprintf('  UKF: %.2f ± %.2f\n', mean(results.final_nees_UKF), std(results.final_nees_UKF));

chi2_lower = 0.35;
chi2_upper = 7.81;

nees_KF_inside = sum(results.nees_KF_all >= chi2_lower & results.nees_KF_all <= chi2_upper);
nees_EKF_inside = sum(results.nees_EKF_all >= chi2_lower & results.nees_EKF_all <= chi2_upper);
nees_UKF_inside = sum(results.nees_UKF_all >= chi2_lower & results.nees_UKF_all <= chi2_upper);

pct_KF = 100 * nees_KF_inside / length(results.nees_KF_all);
pct_EKF = 100 * nees_EKF_inside / length(results.nees_EKF_all);
pct_UKF = 100 * nees_UKF_inside / length(results.nees_UKF_all);

fprintf('\nNEES within [0.35, 7.81] (95%% confidence):\n');
fprintf('  KF:  %.1f%%  (%d/%d)\n', pct_KF, nees_KF_inside, length(results.nees_KF_all));
fprintf('  EKF: %.1f%%  (%d/%d)\n', pct_EKF, nees_EKF_inside, length(results.nees_EKF_all));
fprintf('  UKF: %.1f%%  (%d/%d)\n', pct_UKF, nees_UKF_inside, length(results.nees_UKF_all));

[~, p_KF_EKF] = ttest2(results.rmse_KF, results.rmse_EKF);
[~, p_KF_UKF] = ttest2(results.rmse_KF, results.rmse_UKF);
[~, p_EKF_UKF] = ttest2(results.rmse_EKF, results.rmse_UKF);

fprintf('\nt-test (two-tailed, alpha=0.05):\n');
fprintf('  KF vs EKF:  p = %.6f\n', p_KF_EKF);
fprintf('  KF vs UKF:  p = %.6f\n', p_KF_UKF);
fprintf('  EKF vs UKF: p = %.6f\n', p_EKF_UKF);

time_KF_ms_per_iter = 1000 * mean(results.time_KF) / params.maxIterations;
time_EKF_ms_per_iter = 1000 * mean(results.time_EKF) / params.maxIterations;
time_UKF_ms_per_iter = 1000 * mean(results.time_UKF) / params.maxIterations;

fprintf('\nAverage time per iteration:\n');
fprintf('  KF:  %.5f ms  (1.0× baseline)\n', time_KF_ms_per_iter);
fprintf('  EKF: %.5f ms  (%.1f× slower)\n', time_EKF_ms_per_iter, time_EKF_ms_per_iter / time_KF_ms_per_iter);
fprintf('  UKF: %.5f ms  (%.1f× slower)\n', time_UKF_ms_per_iter, time_UKF_ms_per_iter / time_KF_ms_per_iter);

fprintf('\nTotal simulation time per run:\n');
fprintf('  KF:  %.5f s\n', mean(results.time_KF));
fprintf('  EKF: %.5f s\n', mean(results.time_EKF));
fprintf('  UKF: %.5f s\n', mean(results.time_UKF));

%% Improvement percentages
improvement_EKF_vs_KF = (mean(results.rmse_KF) - mean(results.rmse_EKF)) / mean(results.rmse_KF) * 100;
improvement_UKF_vs_EKF = (mean(results.rmse_EKF) - mean(results.rmse_UKF)) / mean(results.rmse_EKF) * 100;
improvement_UKF_vs_KF = (mean(results.rmse_KF) - mean(results.rmse_UKF)) / mean(results.rmse_KF) * 100;

fprintf('\nImprovement (RMSE):\n');
fprintf('  EKF vs KF:  %.1f%% better\n', improvement_EKF_vs_KF);
fprintf('  UKF vs KF:  %.1f%% better\n', improvement_UKF_vs_KF);
fprintf('  UKF vs EKF: %.1f%% better\n', improvement_UKF_vs_EKF);

%% Visualization
figure('Name', 'Monte Carlo Results', 'Position', [100, 100, 1200, 800]);

% Box plots
subplot(2, 2, 1);
boxplot([results.rmse_KF', results.rmse_EKF', results.rmse_UKF'], 'Labels', {'KF', 'EKF', 'UKF'}, 'Colors', [1 0.6 0; 0.8 0 0; 0 0 0.8]);
ylabel('RMSE [m]');
title('Position Error Distribution', 'FontWeight', 'bold');
grid on;

subplot(2, 2, 2);
boxplot([results.heading_KF', results.heading_EKF', results.heading_UKF'], 'Labels', {'KF', 'EKF', 'UKF'}, 'Colors', [1 0.6 0; 0.8 0 0; 0 0 0.8]);
ylabel('Heading Error [deg]');
title('Heading Error Distribution', 'FontWeight', 'bold');
grid on;

% Histograms
subplot(2, 2, 3);
hold on; grid on;
histogram(results.rmse_KF, 20, 'FaceColor', [1 0.6 0], 'FaceAlpha', 0.6, 'DisplayName', 'KF');
histogram(results.rmse_EKF, 20, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'DisplayName', 'EKF');
histogram(results.rmse_UKF, 20, 'FaceColor', [0 0 0.8], 'FaceAlpha', 0.6, 'DisplayName', 'UKF');
xlabel('RMSE [m]');
ylabel('Count');
title('RMSE Distribution', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

subplot(2, 2, 4);
hold on; grid on;
histogram(results.final_nees_KF, 20, 'FaceColor', [1 0.6 0], 'FaceAlpha', 0.6, 'DisplayName', 'KF');
histogram(results.final_nees_EKF, 20, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'DisplayName', 'EKF');
histogram(results.final_nees_UKF, 20, 'FaceColor', [0 0 0.8], 'FaceAlpha', 0.6, 'DisplayName', 'UKF');
xlabel('Average NEES');
ylabel('Count');
title('NEES Distribution', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);