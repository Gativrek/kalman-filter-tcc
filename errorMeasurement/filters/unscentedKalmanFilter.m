function [xEst, P] = unscentedKalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps, alpha, kappa, beta)
% Unscented Kalman Filter - no linearization!
% Uses sigma points to propagate through nonlinear dynamics
%
% INPUTS:
%   xEst         - Current state estimate [x; y; theta]
%   P            - Current covariance matrix 3x3
%   zOdom        - Odometry measurement [v; omega]
%   zGPS         - GPS measurement [x; y] (empty if not available)
%   gpsAvailable - Boolean flag
%   dt           - Time step
%   Q            - Process noise covariance 3x3
%   R_gps        - GPS measurement noise covariance 2x2
%   alpha        - UKF parameter (spread of sigma points)
%   kappa        - UKF parameter
%   beta         - UKF parameter (2 for Gaussian)
%
% OUTPUTS:
%   xEst - Updated state estimate
%   P    - Updated covariance matrix

%% Extract odometry measurements
v = zOdom(1);      % Linear velocity
omega = zOdom(2);  % Angular velocity

%% UKF Parameters
n = length(xEst);  % State dimension (3)
lambda = alpha^2 * (n + kappa) - n;

%% Generate Sigma Points
% Square root of scaled covariance
sqrtP = chol((n + lambda) * P, 'lower');

% Sigma points (2n+1 = 7 points)
sigmaPoints = zeros(n, 2*n+1);
sigmaPoints(:, 1) = xEst;  % Center point

for i = 1:n
    sigmaPoints(:, i+1) = xEst + sqrtP(:, i);
    sigmaPoints(:, n+i+1) = xEst - sqrtP(:, i);
end

%% Propagate Sigma Points through Nonlinear Dynamics
sigmaPoints_pred = zeros(n, 2*n+1);

for i = 1:(2*n+1)
    theta_i = sigmaPoints(3, i);
    
    % Nonlinear state transition
    sigmaPoints_pred(:, i) = sigmaPoints(:, i) + dt * [v * cos(theta_i);
                                                        v * sin(theta_i);
                                                        omega];
end

%% Compute Weights
Wm = zeros(2*n+1, 1);  % Mean weights
Wc = zeros(2*n+1, 1);  % Covariance weights

Wm(1) = lambda / (n + lambda);
Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);

for i = 2:(2*n+1)
    Wm(i) = 1 / (2 * (n + lambda));
    Wc(i) = 1 / (2 * (n + lambda));
end

%% Predicted Mean
xEst = zeros(n, 1);
for i = 1:(2*n+1)
    xEst = xEst + Wm(i) * sigmaPoints_pred(:, i);
end

%% Predicted Covariance
P = Q;  % Start with process noise
for i = 1:(2*n+1)
    diff = sigmaPoints_pred(:, i) - xEst;
    P = P + Wc(i) * (diff * diff');
end

%% CORRECTION STEP (only if GPS available)

if gpsAvailable
    %% Generate new sigma points around predicted state
    sqrtP_pred = chol((n + lambda) * P, 'lower');
    
    sigmaPoints_update = zeros(n, 2*n+1);
    sigmaPoints_update(:, 1) = xEst;
    
    for i = 1:n
        sigmaPoints_update(:, i+1) = xEst + sqrtP_pred(:, i);
        sigmaPoints_update(:, n+i+1) = xEst - sqrtP_pred(:, i);
    end
    
    %% Transform sigma points to measurement space
    nz = 2;  % Measurement dimension (GPS: x, y)
    Z = zeros(nz, 2*n+1);
    
    for i = 1:(2*n+1)
        % Measurement model: GPS measures [x; y]
        Z(:, i) = sigmaPoints_update(1:2, i);
    end
    
    %% Predicted measurement mean
    zPred = zeros(nz, 1);
    for i = 1:(2*n+1)
        zPred = zPred + Wm(i) * Z(:, i);
    end
    
    %% Innovation covariance
    S = R_gps;
    for i = 1:(2*n+1)
        diff = Z(:, i) - zPred;
        S = S + Wc(i) * (diff * diff');
    end
    
    %% Cross-covariance
    Pxz = zeros(n, nz);
    for i = 1:(2*n+1)
        diff_x = sigmaPoints_update(:, i) - xEst;
        diff_z = Z(:, i) - zPred;
        Pxz = Pxz + Wc(i) * (diff_x * diff_z');
    end
    
    %% Kalman gain
    K = Pxz / S;
    
    %% State update
    innovation = zGPS - zPred;
    xEst = xEst + K * innovation;
    
    %% Covariance update
    P = P - K * S * K';
end

end