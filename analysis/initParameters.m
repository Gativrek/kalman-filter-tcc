function params = initParameters()
% Central interface for all simulation parameters

% Simulation settings
params.dt = 0.1;                    % Time step [s]
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
params.Q = diag([0.01, 0.01, 0.001]);  % 10x maior!

% Measurement noise covariances
params.R_odom = diag([0.15, 0.08].^2);   % Odometry: [v, omega] (informational only)
params.R_gps = diag([0.8, 0.8].^2);      % GPS: [x, y]

% UKF parameters [alpha, kappa, beta]
params.ukfAlpha = 0.001;
params.ukfKappa = 0.0;
params.ukfBeta = 2.0;

% Visualization settings
params.robotBodyLength = 0.4;
params.robotBodyWidth = 0.25;
params.colorTrue = [0.2, 0.8, 0.2];      % Green
params.colorKF = [1.0, 0.5, 0.0];        % Orange
params.colorEKF = [0.8, 0.2, 0.2];       % Red
params.colorUKF = [0.2, 0.2, 0.8];       % Blue

% Control input limits
params.maxLinearVelocity = 2.0;          % [m/s]
params.maxAngularVelocity = pi/2;        % [rad/s]

% Obstacle generation
params.nObstacles = 7;                   % Number of obstacles
params.obstacleMinRadius = 0.3;          % Minimum obstacle radius [m]
params.obstacleMaxRadius = 0.7;          % Maximum obstacle radius [m]

% Occupancy grid parameters
params.gridResolution = 0.2;             % Grid cell size [m]
params.gridXLim = [-2, 12];              % Grid X bounds [m]
params.gridYLim = [-2, 12];              % Grid Y bounds [m]
params.logOddsOccupied = 0.7;            % Log-odds increase when obstacle detected
params.logOddsFree = -0.3;               % Log-odds decrease when free space
params.logOddsMax = 5.0;                 % Maximum log-odds (saturation)
params.logOddsMin = -5.0;                % Minimum log-odds (saturation)

end
