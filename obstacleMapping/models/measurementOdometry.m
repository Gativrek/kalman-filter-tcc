function z = measurementOdometry(u, R)
% Simulates odometry measurement (velocity measurements with noise)
%
% INPUTS:
%   x - True state [x; y; theta]
%   u - True control input [v; omega]
%   R - Measurement noise covariance
%
% OUTPUT:
%   z - Noisy measurement of control [v_measured; omega_measured]

noise = mvnrnd([0; 0], R)';
z = u + noise;

end
