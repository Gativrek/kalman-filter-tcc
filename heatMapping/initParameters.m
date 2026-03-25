function params = initParameters()
% Central interface for all simulation parameters

% Simulation settings
params.dt = 0.3;                    % Time step [s]
params.maxIterations = 100;         % Maximum number of iterations
params.gpsAvailabilityRate = 5;     % GPS available every N steps

% Initial states
params.initialTrueState = [0; 0; pi/4];           % [x; y; theta]
params.initialEstimateKF = [0.5; 0.5; pi/3];      % Initial guess for KF
params.initialEstimateEKF = [0.5; 0.5; pi/3];     % Initial guess for EKF
params.initialEstimateUKF = [0.5; 0.5; pi/3];     % Initial guess for UKF

% Initial covariances
params.initialCovariance = diag([0.5, 0.5, 0.1].^2);

% Process noise covariance Q (model uncertainty + odometry noise)
% This is larger now because it includes both model errors and sensor noise
params.Q = diag([0.3, 0.3, 0.15].^2);

% Measurement noise covariances
params.R_odom = diag([0.4, 0.2].^2);   % Odometry: [v, omega] (informational only)
params.R_gps = diag([1.5, 1.5].^2);      % GPS: [x, y]

% UKF parameters [alpha, kappa, beta]
params.ukfAlpha = 0.01;
params.ukfKappa = 3.0;
params.ukfBeta = 2.0;

% Visualization settings
params.robotBodyLength = 0.8;
params.robotBodyWidth = 0.5;
params.colorTrue = [0.2, 0.8, 0.2];      % Green
params.colorKF = [1.0, 0.5, 0.0];        % Orange
params.colorEKF = [0.8, 0.2, 0.2];       % Red
params.colorUKF = [0.2, 0.2, 0.8];       % Blue

% Control input limits
params.maxLinearVelocity = 5.0;          % [m/s]
params.maxAngularVelocity = 1.5*pi;        % [rad/s]

end
