function params = initParameters()
% Central interface for all simulation parameters

params.scenario = 2;

if params.scenario == 1
    params.gpsAvailabilityRate = 5;
    params.factor = 1;
elseif params.scenario == 2
    params.gpsAvailabilityRate = 15;
    params.factor = 5;
end

% Simulation settings
params.dt = 0.1;                    % Time step [s]
params.maxIterations = 100;         % Maximum number of iterations

% Initial states
params.initialTrueState = [0; 0; pi/4];           % [x; y; theta]
params.initialEstimateKF = [0.5; 0.5; pi/3];      % Initial guess for KF
params.initialEstimateEKF = [0.5; 0.5; pi/3];     % Initial guess for EKF
params.initialEstimateUKF = [0.5; 0.5; pi/3];     % Initial guess for UKF

% Initial covariances
params.initialCovariance = diag([1, 1, 0.1]);

% Process noise covariance Q
params.Q = diag([0.01, 0.01, 0.001].*params.factor);

% Measurement noise covariances
params.R_odom = diag([0.0225, 0.0064].*params.factor);   % Odometry: [v, omega]
params.R_gps = diag([0.64, 0.64].*params.factor);      % GPS: [x, y]

% UKF parameters [alpha, kappa, beta]
params.ukfAlpha = 0.001;
params.ukfKappa = 0.0;
params.ukfBeta = 2.0;

% Visualization settings
params.robotBodyLength = 0.8;
params.robotBodyWidth = 0.5;
params.colorTrue = [0.2, 0.8, 0.2];      % Green
params.colorKF = [1.0, 0.5, 0.0];        % Orange
params.colorEKF = [0.8, 0.2, 0.2];       % Red
params.colorUKF = [0.2, 0.2, 0.8];       % Blue

% Heat map params
params.heatMapResolution = 0.25;
params.heatMapXLim = [-30, 10];
params.heatMapYLim = [-5, 30];

end
