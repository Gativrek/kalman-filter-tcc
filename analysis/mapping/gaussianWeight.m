function weight = gaussianWeight(distance, P)
% Calculates Gaussian weight based on distance and covariance
% Used for spreading probability in heat map
%
% INPUTS:
%   distance - Distance from center [dx; dy]
%   P        - Covariance matrix 2x2 (position only)
%
% OUTPUT:
%   weight - Probability weight (0 to 1)

% Mahalanobis distance (accounts for covariance shape)
try
    % Use covariance to weight the distance
    mahal_dist = sqrt(distance' * inv(P) * distance);
    
    % Gaussian probability (unnormalized)
    weight = exp(-0.5 * mahal_dist^2);
    
catch
    % If P is singular, use simple Euclidean distance
    dist_norm = norm(distance);
    sigma = sqrt(trace(P) / 2);  % Average uncertainty
    weight = exp(-0.5 * (dist_norm / max(sigma, 0.1))^2);
end

end