%% Unicycle Robot - Test with Simple Obstacles
% PHASE 2: Adding obstacles incrementally

clear; clc; close all;

%% Add paths
addpath('models', 'filters', 'simulation', 'visualization');

%% Initialize parameters
params = initParameters();
params.maxIterations = 100;

%% Define FIXED obstacles (not random!)
% 3 obstacles in strategic positions along the figure-8 path
obstacles = [
    struct('center', [2; 4], 'radius', 0.6);   % Middle-top
    struct('center', [8; 18], 'radius', 0.5);   % Upper area
    struct('center', [2; 22], 'radius', 0.7);   % Top area
];

fprintf('=== Obstacle Test ===\n');
fprintf('Number of obstacles: %d\n', length(obstacles));
for i = 1:length(obstacles)
    fprintf('  Obstacle %d: center=(%.1f, %.1f), radius=%.2f\n', ...
            i, obstacles(i).center(1), obstacles(i).center(2), obstacles(i).radius);
end
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

%% Track collisions
collisionCount = 0;

%% Setup visualization
fprintf('Setting up visualization...\n');
fig = figure('Name', 'Unicycle Filter Comparison - With Obstacles', ...
             'Position', [100, 100, 1400, 800]);

tiles = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

handles.tiles = tiles;
handles.errorX_KF = errorX_KF;
handles.errorY_KF = errorY_KF;
handles.errorTheta_KF = errorTheta_KF;
handles.errorX_EKF = errorX_EKF;
handles.errorY_EKF = errorY_EKF;
handles.errorTheta_EKF = errorTheta_EKF;
handles.errorX_UKF = errorX_UKF;
handles.errorY_UKF = errorY_UKF;
handles.errorTheta_UKF = errorTheta_UKF;
handles.obstacles = obstacles;

% Initial visualization
handles = updateVisualizationWithObstacles(handles, xTrue, xKF, xEKF, xUKF, ...
                                           P_KF, P_EKF, P_UKF, ...
                                           trajTrue, trajKF, trajEKF, trajUKF, ...
                                           1, params);

fprintf('Visualization ready!\n');
fprintf('Running simulation with obstacles...\n\n');

%% Main loop
for k = 1:params.maxIterations
    
    %% Automatic control - figure-8 trajectory
    if k < 25
        u = [3.0; 0.5];        % Turn right
    elseif k < 50
        u = [3.0; -0.5];       % Turn left
    elseif k < 75
        u = [3.0; 0.5];        % Turn right again
    else
        u = [3.0; -0.5];       % Complete the 8
    end
    
    %% Propagate ground truth WITH collision detection
    xOld = xTrue;
    [xTrue, actualMovement] = propagateGroundTruthWithObstacles(xTrue, u, params.dt, obstacles);
    
    % Check if collision occurred
    if norm(actualMovement(1:2)) < 0.01 && norm(u) > 0.1
        collisionCount = collisionCount + 1;
    end
    
    %% Generate sensor measurements
    % Odometry measures actual movement (not commanded!)
    actualLinearVel = norm(actualMovement(1:2)) / params.dt;
    actualAngularVel = actualMovement(3) / params.dt;
    
    zOdom = [actualLinearVel; actualAngularVel] + mvnrnd([0;0], params.R_odom)';
    zOdom(1) = max(0, zOdom(1));  % Non-negative velocity
    
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
    
    %% Update visualization
    handles = updateVisualizationWithObstacles(handles, xTrue, xKF, xEKF, xUKF, ...
                                               P_KF, P_EKF, P_UKF, ...
                                               trajTrue, trajKF, trajEKF, trajUKF, ...
                                               k, params);
    
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

fprintf('\n=== Collision Statistics ===\n');
fprintf('Total collisions detected: %d\n', collisionCount);
fprintf('Collision rate: %.1f%%\n', collisionCount/params.maxIterations * 100);

fprintf('\nSimulation complete!\n');
