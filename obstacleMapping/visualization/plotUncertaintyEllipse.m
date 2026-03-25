function h = plotUncertaintyEllipse(mu, Sigma, color, nSigma)
% Plots uncertainty ellipse at nSigma standard deviations
%
% INPUTS:
%   mu     - Mean [x; y]
%   Sigma  - 2x2 covariance matrix
%   color  - RGB color vector
%   nSigma - Number of standard deviations (default: 2)
%
% OUTPUT:
%   h - Handle to plot object

if nargin < 4
    nSigma = 2;
end

% Eigenvalue decomposition
[V, D] = eig(Sigma);

% Semi-axes lengths
a = nSigma * sqrt(D(1,1));
b = nSigma * sqrt(D(2,2));

% Angle of rotation
angle = atan2(V(2,1), V(1,1));

% Parametric ellipse
theta = linspace(0, 2*pi, 100);
ellipse = [a * cos(theta); b * sin(theta)];

% Rotation matrix
R = [cos(angle), -sin(angle);
     sin(angle),  cos(angle)];

% Transform ellipse
ellipseRot = R * ellipse;
x = ellipseRot(1, :) + mu(1);
y = ellipseRot(2, :) + mu(2);

% Plot
hold on;
h = plot(x, y, '-', 'Color', color, 'LineWidth', 2.5);

end
