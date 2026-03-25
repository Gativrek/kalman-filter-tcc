function [xTrue, actualMovement] = propagateGroundTruthWithObstacles(xTrue, u, dt, obstacles)
% Propagates true state with simple obstacle collision
%
% INPUTS:
%   xTrue     - Current true state [x; y; theta]
%   u         - Control input [v; omega]
%   dt        - Time step
%   obstacles - Struct array of obstacles (can be empty)
%
% OUTPUTS:
%   xTrue          - Next true state (stopped if collision)
%   actualMovement - Actual displacement [dx; dy; dtheta]

v = u(1);
omega = u(2);
theta = xTrue(3);

% Calculate next position
xNext = xTrue + dt * [v * cos(theta);
                       v * sin(theta);
                       omega];

% Check if next position would collide
if ~isempty(obstacles) && checkCollisionSimple(xNext(1:2), obstacles)
    xActual = xTrue;
    actualMovement = [0; 0; 0];
else
    xActual = xNext;
    actualMovement = xActual - xTrue;
end

xTrue = xActual;

end
