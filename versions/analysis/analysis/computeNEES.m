function nees = computeNEES(xTrue, xEst, P)
% Computes Normalized Estimation Error Squared (NEES)
% Tests if filter is statistically consistent
%
% INPUTS:
%   xTrue - True state [x; y; theta]
%   xEst  - Estimated state [x; y; theta]
%   P     - Covariance matrix 3x3
%
% OUTPUT:
%   nees - NEES value (scalar)

% Estimation error
error = xTrue - xEst;

% Normalize angle error to [-pi, pi]
error(3) = atan2(sin(error(3)), cos(error(3)));

% NEES = error' * P^-1 * error
try
    nees = error' * inv(P) * error;
catch
    % If P is singular, use a small regularization
    nees = error' * inv(P + eye(3)*1e-6) * error;
end

end