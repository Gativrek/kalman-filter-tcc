function h = createRobotBody(x, y, theta, bodyLength, bodyWidth, color, lineStyle)
% Draws a rectangular robot with direction indicator
%
% INPUTS:
%   x, y        - Robot position
%   theta       - Robot orientation [rad]
%   bodyLength  - Length of robot body
%   bodyWidth   - Width of robot body
%   color       - RGB color vector
%   lineStyle   - Line style string
%
% OUTPUT:
%   h - Handle to patch object

% Robot vertices in body frame
vertices = [-bodyLength/2, -bodyWidth/2;
             bodyLength/2, -bodyWidth/2;
             bodyLength/2,  bodyWidth/2;
            -bodyLength/2,  bodyWidth/2];

% Rotation matrix
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];

% Transform to world frame
verticesWorld = (R * vertices')';
verticesWorld(:, 1) = verticesWorld(:, 1) + x;
verticesWorld(:, 2) = verticesWorld(:, 2) + y;

% Draw robot body
hold on;
h = patch(verticesWorld(:, 1), verticesWorld(:, 2), color, ...
          'EdgeColor', color * 0.6, ...
          'LineWidth', 1.5, ...
          'LineStyle', lineStyle);

% Draw direction indicator (arrow)
arrowLength = bodyLength * 0.7;
xArrow = [x, x + arrowLength * cos(theta)];
yArrow = [y, y + arrowLength * sin(theta)];
plot(xArrow, yArrow, 'k-', 'LineWidth', 2);

end
