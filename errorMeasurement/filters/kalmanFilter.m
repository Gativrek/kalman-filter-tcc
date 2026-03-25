function [xEst, P] = kalmanFilter(xEst, P, zOdom, zGPS, gpsAvailable, dt, Q, R_gps)
% Linear Kalman Filter with FIXED linearization at theta=0
% WARNING: Inaccurate for large orientation changes!
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

%% PREDICTION STEP (always happens)

% Extract odometry measurements
v = zOdom(1);      % Linear velocity
omega = zOdom(2);  % Angular velocity

% Current orientation
theta = xEst(3);

% Propagate state using ACTUAL nonlinear model
xEst = xEst + dt * [v * cos(theta);
                     v * sin(theta);
                     omega];

% CRITICAL BUG: Linearization FIXED at theta_lin = 0
theta_lin = 0;  % Always linearize at zero orientation!

% State transition Jacobian F (FIXED linearization)
F = [1, 0, -v*dt*sin(theta_lin);
     0, 1,  v*dt*cos(theta_lin);
     0, 0,  1];

% Covariance prediction
P = F * P * F' + Q;

%% CORRECTION STEP (only if GPS available)

if gpsAvailable
    % Measurement model: H maps state to GPS measurement
    H = [1, 0, 0;   % GPS measures x
         0, 1, 0];  % GPS measures y
    
    % Innovation (measurement residual)
    y = zGPS - H * xEst;
    
    % Innovation covariance
    S = H * P * H' + R_gps;
    
    % Kalman gain
    K = P * H' / S;
    
    % State update
    xEst = xEst + K * y;
    
    % Covariance update (Joseph form for numerical stability)
    I_KH = eye(3) - K * H;
    P = I_KH * P * I_KH' + K * R_gps * K';
end

end