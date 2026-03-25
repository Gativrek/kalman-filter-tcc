function [xEst, P] = unscentedKalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps, alpha, kappa, beta)
% Unscented Kalman Filter using sigma points
%
% INPUTS:
%   xEst          - Current state estimate [x; y; theta]
%   P             - Current covariance matrix
%   zOdom         - Odometry measurement [v; omega]
%   zGPS          - GPS measurement [x; y] (if available)
%   gpsAvailable  - Boolean flag for GPS availability
%   dt            - Time step
%   Q             - Process noise covariance
%   R_gps         - GPS measurement noise covariance
%   alpha, kappa, beta - UKF tuning parameters
%
% OUTPUTS:
%   xEst - Updated state estimate
%   P    - Updated covariance matrix

n = length(xEst);
lambda = alpha^2 * (n + kappa) - n;

%% Generate sigma points
sqrtP = chol((n + lambda) * P, 'lower');
sigmaPoints = [xEst, xEst + sqrtP, xEst - sqrtP];

%% Weights
wM = [lambda/(n + lambda), repmat(1/(2*(n + lambda)), 1, 2*n)];
wC = wM;
wC(1) = wC(1) + (1 - alpha^2 + beta);

%% Prediction step - propagate sigma points through dynamics
% Use measured odometry
v = zOdom(1);
omega = zOdom(2);

sigmaPointsPred = zeros(n, 2*n + 1);
for i = 1:(2*n + 1)
    theta = sigmaPoints(3, i);
    
    sigmaPointsPred(:, i) = sigmaPoints(:, i) + dt * [v * cos(theta);
                                                        v * sin(theta);
                                                        omega];
end

% Predicted mean
xPred = sigmaPointsPred * wM';

% Predicted covariance
P = Q;
for i = 1:(2*n + 1)
    diff = sigmaPointsPred(:, i) - xPred;
    P = P + wC(i) * (diff * diff');
end

%% Correction step - GPS (if available)
if gpsAvailable
    % Generate new sigma points around prediction
    sqrtP = chol((n + lambda) * P, 'lower');
    sigmaPoints = [xPred, xPred + sqrtP, xPred - sqrtP];
    
    % Propagate through GPS measurement model
    zSigmaGPS = sigmaPoints(1:2, :);  % GPS measures [x; y]
    
    % Predicted measurement
    zPredGPS = zSigmaGPS * wM';
    
    % Innovation covariance
    S_gps = R_gps;
    Pxz_gps = zeros(n, 2);
    for i = 1:(2*n + 1)
        diffZ = zSigmaGPS(:, i) - zPredGPS;
        diffX = sigmaPoints(:, i) - xPred;
        S_gps = S_gps + wC(i) * (diffZ * diffZ');
        Pxz_gps = Pxz_gps + wC(i) * (diffX * diffZ');
    end
    
    % Kalman gain and update
    K_gps = Pxz_gps / S_gps;
    innovation_gps = zGPS - zPredGPS;
    xEst = xPred + K_gps * innovation_gps;
    P = P - K_gps * S_gps * K_gps';
else
    % No GPS correction
    xEst = xPred;
end

end
