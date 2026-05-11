function collided = checkCollisionSimple(pos, obstacles)
% Simple collision detection
%
% INPUTS:
%   pos       - Position to check [x; y]
%   obstacles - Struct array with fields: center [x;y], radius
%
% OUTPUT:
%   collided - Boolean, true if inside any obstacle

collided = false;
safetyMargin = 0.15;

for i = 1:length(obstacles)
    center = obstacles(i).center;
    radius = obstacles(i).radius;
    
    % Distance from position to obstacle center
    dist = norm(pos - center);
    
    % Check if too close
    if dist < (radius + safetyMargin)
        collided = true;
        return;
    end
end

end
