%% Unicycle Robot - Heat Map Visualization
% Shows "certainty of having been here" for each filter
% NO OBSTACLES - just trajectory tracking with heat maps

clear; clc; close all;

%% Add paths
addpath('models', 'filters', 'simulation', 'visualization', 'mapping');

%% Initialize parameters
params = initParameters();
params.maxIterations = 100;
% Aggressive scenario for UKF vs EKF comparison
params.gpsAvailabilityRate = 15;         
params.dt = 0.1;                         
params.Q = diag([0.05, 0.05, 0.01]);     
params.R_odom = diag([0.2, 0.1].^2);   

% Heat map parameters
params.heatMapResolution = 0.25;  % 25cm cells
params.heatMapXLim = [-10, 5];
params.heatMapYLim = [0, 30];

fprintf('=== Heat Map Tracking Test ===\n');
fprintf('Heat map resolution: %.2fm (%.0fcm cells)\n', ...
        params.heatMapResolution, params.heatMapResolution*100);
fprintf('Heat map bounds: X[%.0f, %.0f], Y[%.0f, %.0f]\n', ...
        params.heatMapXLim(1), params.heatMapXLim(2), ...
        params.heatMapYLim(1), params.heatMapYLim(2));
fprintf('\n');

%% Initialize states
xTrue = params.initialTrueState;
xKF = params.initialEstimateKF;
xEKF = params.initialEstimateEKF;
xUKF = params.initialEstimateUKF;

P_KF = params.initialCovariance;
P_EKF = params.initialCovariance;
P_UKF = params.initialCovariance;

%% Initialize trajectory storage
trajTrue = xTrue;
trajKF = xKF;
trajEKF = xEKF;
trajUKF = xUKF;

%% Initialize error tracking
errorX_KF = zeros(1, params.maxIterations);
errorY_KF = zeros(1, params.maxIterations);
errorTheta_KF = zeros(1, params.maxIterations);

errorX_EKF = zeros(1, params.maxIterations);
errorY_EKF = zeros(1, params.maxIterations);
errorTheta_EKF = zeros(1, params.maxIterations);

errorX_UKF = zeros(1, params.maxIterations);
errorY_UKF = zeros(1, params.maxIterations);
errorTheta_UKF = zeros(1, params.maxIterations);

%% Initialize covariance trace tracking
trace_KF = zeros(1, params.maxIterations);
trace_EKF = zeros(1, params.maxIterations);
trace_UKF = zeros(1, params.maxIterations);

%% Initialize heat maps
fprintf('Creating heat maps...\n');

heatMap_KF = createHeatMap(params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);
heatMap_EKF = createHeatMap(params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);
heatMap_UKF = createHeatMap(params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);

fprintf('Heat maps initialized: %dx%d cells\n\n', size(heatMap_KF, 2), size(heatMap_KF, 1));

%% Setup visualization
fprintf('Setting up visualization...\n');
fig = figure('Name', 'Heat Map Tracking - Path Certainty', ...
             'Position', [50, 50, 1400, 800]);

handles.errorX_KF = errorX_KF;
handles.errorY_KF = errorY_KF;
handles.errorTheta_KF = errorTheta_KF;

handles.errorX_EKF = errorX_EKF;
handles.errorY_EKF = errorY_EKF;
handles.errorTheta_EKF = errorTheta_EKF;

handles.errorX_UKF = errorX_UKF;
handles.errorY_UKF = errorY_UKF;
handles.errorTheta_UKF = errorTheta_UKF;

handles.trace_KF = trace_KF;
handles.trace_EKF = trace_EKF;
handles.trace_UKF = trace_UKF;

% Initial visualization
handles = updateVisualizationHeatMap(handles, xTrue, xKF, xEKF, xUKF, ...
                                     P_KF, P_EKF, P_UKF, ...
                                     trajTrue, trajKF, trajEKF, trajUKF, ...
                                     1, params, ...
                                     heatMap_KF, heatMap_EKF, heatMap_UKF);

fprintf('Visualization ready!\n');
fprintf('Running simulation with heat map tracking...\n\n');

%% Main loop
for k = 1:params.maxIterations
    
    %% Automatic control - figure-8 trajectory
    if k < 20
        u = [5.0; 1.5];        % MUITO rápido + curva APERTADA
    elseif k < 40
        u = [5.0; -1.5];       % Muda direção bruscamente
    elseif k < 60
        u = [5.0; 1.5];
    elseif k < 80
        u = [5.0; -1.5];
    else
        u = [5.0; 1.5];
    end
    
    %% Propagate ground truth (NO OBSTACLES)
    xTrue = propagateGroundTruth(xTrue, u, params.dt);
    
    %% Generate sensor measurements
    zOdom = measurementOdometry(xTrue, u, params.R_odom);
    
    gpsAvailable = (mod(k, params.gpsAvailabilityRate) == 0);
    if gpsAvailable
        zGPS = measurementGPS(xTrue, params.R_gps);
    else
        zGPS = [];
    end
    
    %% Update filters
    [xKF, P_KF] = kalmanFilter(xKF, P_KF, zOdom, zGPS, gpsAvailable, ...
                               params.dt, params.Q, params.R_gps);
    
    [xEKF, P_EKF] = extendedKalmanFilter(xEKF, P_EKF, zOdom, zGPS, gpsAvailable, ...
                                         params.dt, params.Q, params.R_gps);
    
    [xUKF, P_UKF] = unscentedKalmanFilter(xUKF, P_UKF, zOdom, zGPS, gpsAvailable, ...
                                          params.dt, params.Q, params.R_gps, ...
                                          params.ukfAlpha, params.ukfKappa, params.ukfBeta);
    
    %% Update heat maps (Gaussian spreading - VERSION 2)
    heatMap_KF = updateHeatMap(heatMap_KF, xKF, P_KF, ...
                               params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);
    
    heatMap_EKF = updateHeatMap(heatMap_EKF, xEKF, P_EKF, ...
                                params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);
    
    heatMap_UKF = updateHeatMap(heatMap_UKF, xUKF, P_UKF, ...
                                params.heatMapXLim, params.heatMapYLim, params.heatMapResolution);
    
    %% Store trajectories
    trajTrue = [trajTrue, xTrue];
    trajKF = [trajKF, xKF];
    trajEKF = [trajEKF, xEKF];
    trajUKF = [trajUKF, xUKF];
    
    %% Compute errors
    handles.errorX_KF(k) = xKF(1) - xTrue(1);
    handles.errorY_KF(k) = xKF(2) - xTrue(2);
    handles.errorTheta_KF(k) = rad2deg(angdiff(xKF(3), xTrue(3)));
    
    handles.errorX_EKF(k) = xEKF(1) - xTrue(1);
    handles.errorY_EKF(k) = xEKF(2) - xTrue(2);
    handles.errorTheta_EKF(k) = rad2deg(angdiff(xEKF(3), xTrue(3)));
    
    handles.errorX_UKF(k) = xUKF(1) - xTrue(1);
    handles.errorY_UKF(k) = xUKF(2) - xTrue(2);
    handles.errorTheta_UKF(k) = rad2deg(angdiff(xUKF(3), xTrue(3)));

    %% Store covariance traces
    handles.trace_KF(k) = trace(P_KF(1:2, 1:2));   % Apenas posição
    handles.trace_EKF(k) = trace(P_EKF(1:2, 1:2));
    handles.trace_UKF(k) = trace(P_UKF(1:2, 1:2));
    
    %% Update visualization
    handles = updateVisualizationHeatMap(handles, xTrue, xKF, xEKF, xUKF, ...
                                         P_KF, P_EKF, P_UKF, ...
                                         trajTrue, trajKF, trajEKF, trajUKF, ...
                                         k, params, ...
                                         heatMap_KF, heatMap_EKF, heatMap_UKF);
    
    pause(0.01);  % Small pause for visualization
end

%% Final statistics
fprintf('\n=== Final Statistics ===\n');

rmse_KF = sqrt(mean(handles.errorX_KF.^2 + handles.errorY_KF.^2));
rmse_EKF = sqrt(mean(handles.errorX_EKF.^2 + handles.errorY_EKF.^2));
rmse_UKF = sqrt(mean(handles.errorX_UKF.^2 + handles.errorY_UKF.^2));

heading_KF = sqrt(mean(handles.errorTheta_KF.^2));
heading_EKF = sqrt(mean(handles.errorTheta_EKF.^2));
heading_UKF = sqrt(mean(handles.errorTheta_UKF.^2));

fprintf('KF  - RMSE: %.3f m, Heading: %.2f deg\n', rmse_KF, heading_KF);
fprintf('EKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_EKF, heading_EKF);
fprintf('UKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_UKF, heading_UKF);

%% Heat map statistics
fprintf('\n=== Heat Map Statistics ===\n');
total_visits_KF = sum(heatMap_KF(:));
total_visits_EKF = sum(heatMap_EKF(:));
total_visits_UKF = sum(heatMap_UKF(:));

fprintf('Total visits recorded:\n');
fprintf('  KF:  %d\n', total_visits_KF);
fprintf('  EKF: %d\n', total_visits_EKF);
fprintf('  UKF: %d\n', total_visits_UKF);

max_visits_KF = max(heatMap_KF(:));
max_visits_EKF = max(heatMap_EKF(:));
max_visits_UKF = max(heatMap_UKF(:));

fprintf('Max visits per cell:\n');
fprintf('  KF:  %d (most concentrated cell)\n', max_visits_KF);
fprintf('  EKF: %d\n', max_visits_EKF);
fprintf('  UKF: %d\n', max_visits_UKF);

fprintf('\nSimulation complete!\n');
fprintf('Check the heat maps (bottom row) to see path certainty.\n');
fprintf('Hotter colors = higher certainty of having been there.\n');
