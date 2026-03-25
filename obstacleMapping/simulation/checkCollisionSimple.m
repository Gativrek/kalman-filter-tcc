function collided = checkCollisionSimple(pos, obstacles)
% Ultra-simple collision detection - just checks distance to circles
%
% INPUTS:
%   pos       - Position to check [x; y]
%   obstacles - Struct array with fields: center [x;y], radius
%
% OUTPUT:
%   collided - Boolean, true if inside any obstacle

collided = false;

% Safety margin - stop a bit before actually hitting
safetyMargin = 0.15;  % 15cm margin

for i = 1:length(obstacles)
    center = obstacles(i).center;
    radius = obstacles(i).radius;
    
    % Distance from position to obstacle center
    dist = norm(pos - center);
    
    % Check if too close (inside radius + margin)
    if dist < (radius + safetyMargin)
        collided = true;
        return;
    end
end

end
