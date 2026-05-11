function xTrue = propagateGroundTruth(xTrue, u, dt)
% Propagates true state without noise
%
% INPUTS:
%   xTrue - Current true state [x; y; theta]
%   u     - Control input [v; omega]
%   dt    - Time step
%
% OUTPUT:
%   xTrue - Next true state

v = u(1);
omega = u(2);
theta = xTrue(3);

xTrue = xTrue + dt * [v * cos(theta);
                       v * sin(theta);
                       omega];

end
