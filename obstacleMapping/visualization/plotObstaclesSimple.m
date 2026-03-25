function plotObstaclesSimple(obstacles)
% Plots circular obstacles as filled circles
% SIMPLE VERSION - just filled gray circles
%
% INPUT:
%   obstacles - Struct array with fields: center, radius

if isempty(obstacles)
    return;
end

hold on;

for i = 1:length(obstacles)
    center = obstacles(i).center;
    radius = obstacles(i).radius;
    
    % Draw filled circle
    theta = linspace(0, 2*pi, 50);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    
    % Fill with dark gray
    fill(x, y, [0.3 0.3 0.3], 'EdgeColor', [0 0 0], 'LineWidth', 2, ...
         'FaceAlpha', 0.7);
end

end
