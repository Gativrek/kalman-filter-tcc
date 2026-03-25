function [xEst, P] = extendedKalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps)
% Extended Kalman Filter with adaptive linearization
% Linearizes around CURRENT operating point
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
%
% OUTPUTS:
%   xEst - Updated state estimate
%   P    - Updated covariance matrix

%% PREDICTION STEP

% Extract odometry measurements
v = zOdom(1);      % Linear velocity
omega = zOdom(2);  % Angular velocity

% Current orientation
theta = xEst(3);

% Propagate state using nonlinear model
xEst = xEst + dt * [v * cos(theta);
                     v * sin(theta);
                     omega];

% State transition Jacobian F (linearized at CURRENT theta)
F = [1, 0, -v*dt*sin(theta);
     0, 1,  v*dt*cos(theta);
     0, 0,  1];

% Covariance prediction
P = F * P * F' + Q;

%% CORRECTION STEP (only if GPS available)

if gpsAvailable
    % Measurement model
    H = [1, 0, 0;
         0, 1, 0];
    
    % Innovation
    y = zGPS - H * xEst;
    
    % Innovation covariance
    S = H * P * H' + R_gps;
    
    % Kalman gain
    K = P * H' / S;
    
    % State update
    xEst = xEst + K * y;
    
    % Covariance update (Joseph form)
    I_KH = eye(3) - K * H;
    P = I_KH * P * I_KH' + K * R_gps * K';
end

end