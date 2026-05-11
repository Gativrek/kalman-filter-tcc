function [xActual, actualMovement] = applyCollisionDynamics(xStart, xCommanded, obstacles)
% Applies collision dynamics - robot stops before hitting obstacle
%
% INPUTS:
%   xStart     - Starting state [x; y; theta]
%   xCommanded - Commanded next state [x; y; theta]
%   obstacles  - Struct array of obstacles
%
% OUTPUTS:
%   xActual        - Actual state after collision [x; y; theta]
%   actualMovement - Actual displacement [dx; dy; dtheta]

% Check collision along path
[collided, distToCollision] = checkCollision(xStart(1:2), xCommanded(1:2), obstacles);

if collided
    % Calculate how far we can actually move
    safetyMargin = 0.1;
    
    pathVec = xCommanded(1:2) - xStart(1:2);
    pathLength = norm(pathVec);
    
    if pathLength > 1e-6
        % Move to safe distance
        safeDist = max(0, distToCollision - safetyMargin);
        fraction = safeDist / pathLength;
        
        % Actual position
        actualPos = xStart(1:2) + fraction * pathVec;
        
        % Keep orientation change proportional to movement
        actualTheta = xStart(3) + fraction * (xCommanded(3) - xStart(3));
        
        xActual = [actualPos; actualTheta];
    else
        % No movement commanded
        xActual = xStart;
    end
else
    % No collision, move as commanded
    xActual = xCommanded;
end

% Compute actual movement
actualMovement = xActual - xStart;

end
