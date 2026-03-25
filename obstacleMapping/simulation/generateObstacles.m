function obstacles = generateObstacles(nObstacles, xlim, ylim, minRadius, maxRadius)
% Generates random circular obstacles in the environment
% Ensures no overlap with origin or other obstacles
%
% INPUTS:
%   nObstacles - Number of obstacles to generate
%   xlim       - [xmin, xmax] bounds
%   ylim       - [ymin, ymax] bounds
%   minRadius  - Minimum obstacle radius
%   maxRadius  - Maximum obstacle radius
%
% OUTPUT:
%   obstacles - Struct array with fields: center [x;y], radius

obstacles = struct('center', {}, 'radius', {});

margin = 1.5;
minDistFromOrigin = 2.5;  % Keep away from robot start
minDistBetweenObstacles = 0.5;  % Minimum gap between obstacles

i = 1;
attempts = 0;
maxAttempts = 1000;

while i <= nObstacles && attempts < maxAttempts
    attempts = attempts + 1;
    
    % Random position (avoid edges)
    x = xlim(1) + margin + rand() * (xlim(2) - xlim(1) - 2*margin);
    y = ylim(1) + margin + rand() * (ylim(2) - ylim(1) - 2*margin);
    
    % Random radius
    radius = minRadius + rand() * (maxRadius - minRadius);
    
    % Check distance from origin
    if norm([x; y]) < minDistFromOrigin
        continue;  % Too close to origin
    end
    
    % Check overlap with existing obstacles
    tooClose = false;
    for j = 1:(i-1)
        dist = norm([x; y] - obstacles(j).center);
        if dist < (radius + obstacles(j).radius + minDistBetweenObstacles)
            tooClose = true;
            break;
        end
    end
    
    if tooClose
        continue;  % Overlaps with existing obstacle
    end
    
    % Valid obstacle, store it
    obstacles(i).center = [x; y];
    obstacles(i).radius = radius;
    i = i + 1;
end

% Warn if couldn't generate all obstacles
if i <= nObstacles
    fprintf('Warning: Could only generate %d/%d obstacles\n', i-1, nObstacles);
end

end
