function [xEst, P] = kalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps)
% Linear Kalman Filter (with FIXED linearization at theta=0)
% This demonstrates the limitation of using a fixed linearization point
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

% FIXED linearization point (theta_lin = 0 for simplicity)
% This is the KEY difference from EKF: we don't update this!
theta_lin = 0;  

% State transition matrix (FIXED linearization)
F = [1, 0, -dt * v * sin(theta_lin);
     0, 1,  dt * v * cos(theta_lin);
     0, 0,  1];

% Predicted state (use non-linear model with measured velocities)
xPred = xEst + dt * [v * cos(theta);
                     v * sin(theta);
                     omega];

% Predicted covariance (uses FIXED F)
% Q already accounts for odometry noise + model uncertainty
P = F * P * F' + Q;

%% Correction step - GPS (if available)
if gpsAvailable
    % GPS measures [x; y]
    H_gps = [1, 0, 0;
             0, 1, 0];
    
    % Kalman gain
    S_gps = H_gps * P * H_gps' + R_gps;
    K_gps = P * H_gps' / S_gps;
    
    % Innovation
    innovation_gps = zGPS - xPred(1:2);
    
    % Update state
    xEst = xPred + K_gps * innovation_gps;
    
    % Update covariance
    P = (eye(3) - K_gps * H_gps) * P;
else
    % No GPS correction, just use prediction
    xEst = xPred;
end

end
