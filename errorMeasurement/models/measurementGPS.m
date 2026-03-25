function z = measurementGPS(x, R)
% Simulates GPS measurement (position with noise)
%
% INPUTS:
%   x - True state [x; y; theta]
%   R - Measurement noise covariance
%
% OUTPUT:
%   z - Noisy position measurement [x_measured; y_measured]

noise = mvnrnd([0; 0], R)';
z = x(1:2) + noise;

end
