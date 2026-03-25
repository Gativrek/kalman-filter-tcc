function [xEst, P] = extendedKalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps)
% Extended Kalman Filter with proper Jacobian computation
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
%
% OUTPUTS:
%   xEst - Updated state estimate
%   P    - Updated covariance matrix

%% Prediction step
% Use measured odometry (noisy)
v = zOdom(1);
omega = zOdom(2);
theta = xEst(3);

% Predicted state (non-linear propagation)
xPred = xEst + dt * [v * cos(theta);
                     v * sin(theta);
                     omega];

% Jacobian of dynamics w.r.t. state (evaluated at current estimate)
F = [1, 0, -dt * v * sin(theta);
     0, 1,  dt * v * cos(theta);
     0, 0,  1];

% Predicted covariance
P = F * P * F' + Q;

%% Correction step - GPS (if available)
if gpsAvailable
    % Measurement model: h(x) = [x; y]
    h_gps = xPred(1:2);
    
    % Jacobian of GPS measurement w.r.t. state
    H_gps = [1, 0, 0;
             0, 1, 0];
    
    % Kalman gain
    S_gps = H_gps * P * H_gps' + R_gps;
    K_gps = P * H_gps' / S_gps;
    
    % Innovation
    innovation_gps = zGPS - h_gps;
    
    % Update state
    xEst = xPred + K_gps * innovation_gps;
    
    % Update covariance
    P = (eye(3) - K_gps * H_gps) * P;
else
    % No GPS correction
    xEst = xPred;
end

end
