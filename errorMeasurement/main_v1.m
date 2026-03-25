clear; clc; close all;

%% Add paths
addpath('models', 'filters', 'simulation', 'visualization');

%% Initialize parameters
params = initParameters();
seed = 22011;
rng(seed);

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

%% Setup visualization
fig = figure('Name', 'Unicycle Filter Comparison', 'Position', [100, 100, 1400, 800]);

tiles = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

handles.tiles = tiles;
handles.xlim = [-2, 12];
handles.ylim = [-2, 12];
handles.errorX_KF = errorX_KF;
handles.errorY_KF = errorY_KF;
handles.errorTheta_KF = errorTheta_KF;
handles.errorX_EKF = errorX_EKF;
handles.errorY_EKF = errorY_EKF;
handles.errorTheta_EKF = errorTheta_EKF;
handles.errorX_UKF = errorX_UKF;
handles.errorY_UKF = errorY_UKF;
handles.errorTheta_UKF = errorTheta_UKF;

% Initial visualization
handles = updateVisualization(handles, xTrue, xKF, xEKF, xUKF, P_KF, P_EKF, P_UKF, trajTrue, trajKF, trajEKF, trajUKF, 1, params);

%% Main loop
for k = 1:params.maxIterations
    
    if params.scenario == 1
        if k < 20
            u = [3.0; 0.5];
        elseif k < 40
            u = [3.0; -0.5];
        elseif k < 60
            u = [3.0; 0.5];
        elseif k < 80
            u = [3.0; -0.5];
        else
            u = [3.0; 0.5];
        end
    elseif params.scenario == 2
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
    end

    % Propagate ground truth
    xTrue = propagateGroundTruth(xTrue, u, params.dt);
    
    % Generate sensor measurements
    zOdom = measurementOdometry(u, params.R_odom);
    
    gpsAvailable = (mod(k, params.gpsAvailabilityRate) == 0);
    if gpsAvailable
        zGPS = measurementGPS(xTrue, params.R_gps);
    else
        zGPS = [];
    end
    
    % Update filters
    [xKF, P_KF] = kalmanFilter(xKF, P_KF, zOdom, zGPS, gpsAvailable, params);
    [xEKF, P_EKF] = extendedKalmanFilter(xEKF, P_EKF, zOdom, zGPS, gpsAvailable, params);
    [xUKF, P_UKF] = unscentedKalmanFilter(xUKF, P_UKF, zOdom, zGPS, gpsAvailable, params);
    
    % Store trajectories
    trajTrue = [trajTrue, xTrue];
    trajKF = [trajKF, xKF];
    trajEKF = [trajEKF, xEKF];
    trajUKF = [trajUKF, xUKF];
    
    % Compute errors
    handles.errorX_KF(k) = xKF(1) - xTrue(1);
    handles.errorY_KF(k) = xKF(2) - xTrue(2);
    handles.errorTheta_KF(k) = rad2deg(angdiff(xKF(3), xTrue(3)));
    
    handles.errorX_EKF(k) = xEKF(1) - xTrue(1);
    handles.errorY_EKF(k) = xEKF(2) - xTrue(2);
    handles.errorTheta_EKF(k) = rad2deg(angdiff(xEKF(3), xTrue(3)));
    
    handles.errorX_UKF(k) = xUKF(1) - xTrue(1);
    handles.errorY_UKF(k) = xUKF(2) - xTrue(2);
    handles.errorTheta_UKF(k) = rad2deg(angdiff(xUKF(3), xTrue(3)));
    
    % Update visualization
    handles = updateVisualization(handles, xTrue, xKF, xEKF, xUKF, P_KF, P_EKF, P_UKF, trajTrue, trajKF, trajEKF, trajUKF, k, params);
    
    pause(0.01);
end

%% Final statistics
fprintf('Simulation Statistics\n');

rmse_KF = sqrt(mean(handles.errorX_KF.^2 + handles.errorY_KF.^2));
rmse_EKF = sqrt(mean(handles.errorX_EKF.^2 + handles.errorY_EKF.^2));
rmse_UKF = sqrt(mean(handles.errorX_UKF.^2 + handles.errorY_UKF.^2));

heading_KF = sqrt(mean(handles.errorTheta_KF.^2));
heading_EKF = sqrt(mean(handles.errorTheta_EKF.^2));
heading_UKF = sqrt(mean(handles.errorTheta_UKF.^2));

fprintf('KF  - RMSE: %.3f m, Heading: %.2f deg\n', rmse_KF, heading_KF);
fprintf('EKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_EKF, heading_EKF);
fprintf('UKF - RMSE: %.3f m, Heading: %.2f deg\n', rmse_UKF, heading_UKF);