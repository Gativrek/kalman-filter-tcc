function [xEst, P] = kalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, params)
% Linear Kalman Filter
%
% INPUTS:
%   xEst          - Current state estimate [x; y; theta]
%   P             - Current covariance matrix
%   zOdom         - Odometry measurement [v; omega]
%   zGPS          - GPS measurement [x; y] (if available)
%   gpsAvailable  - Boolean flag for GPS availability
%   params        - Parameters structure
%
% OUTPUTS:
%   xEst - Updated state estimate
%   P    - Updated covariance matrix

%% Params Extraction
dt = params.dt;
Q = params.Q;
R_gps = params.R_gps;

%% Prediction step (odometry)
% Use measured odometry
v = zOdom(1);
omega = zOdom(2);
theta = xEst(3);

% Fixed linearization point
theta_lin = 0;  

% State transition matrix (FIXED linearization)
F = [1, 0, -dt * v * sin(theta_lin);
     0, 1,  dt * v * cos(theta_lin);
     0, 0,  1];

% Predicted state
xPred = xEst + dt * [v * cos(theta);
                     v * sin(theta);
                     omega];

% Predicted covariance
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
