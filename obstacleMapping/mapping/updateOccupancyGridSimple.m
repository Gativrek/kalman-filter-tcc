function grid = updateOccupancyGridSimple(grid, xRobot, commandedMovement, actualMovement, xlim, ylim, resolution)
% Updates occupancy grid based on movement discrepancy
% ULTRA-SIMPLE VERSION - binary marking
%
% INPUTS:
%   grid               - Current occupancy grid
%   xRobot             - Robot position BEFORE movement [x; y; theta]
%   commandedMovement  - What we tried to do [dx; dy; dtheta]
%   actualMovement     - What actually happened [dx; dy; dtheta]
%   xlim, ylim         - Grid bounds
%   resolution         - Grid cell size
%
% OUTPUT:
%   grid - Updated occupancy grid

% Calculate discrepancy
commandedDist = norm(commandedMovement(1:2));
actualDist = norm(actualMovement(1:2));
discrepancy = commandedDist - actualDist;

% Threshold for obstacle detection (in meters)
threshold = 0.15;  % 15cm difference suggests obstacle

% Only process if there was significant commanded movement
if commandedDist > 0.05
    
    if discrepancy > threshold
        %% OBSTACLE DETECTED!
        
        % Where we are
        startPos = xRobot(1:2);
        
        % Where we tried to go
        expectedEnd = startPos + commandedMovement(1:2);
        
        % Where we actually went
        actualEnd = startPos + actualMovement(1:2);
        
        % Mark cells between actual end and expected end as OCCUPIED
        % (obstacle is somewhere in this region)
        
        direction = expectedEnd - actualEnd;
        distance = norm(direction);
        
        if distance > 0.01
            direction = direction / distance;
            
            % Sample points along the ray
            nSamples = max(1, ceil(distance / resolution));
            nSamples = min(nSamples, 20);  % Limit samples
            
            for i = 1:nSamples
                t = i / nSamples;
                point = actualEnd + t * distance * direction;
                
                % Convert to grid coordinates
                ix = round((point(1) - xlim(1)) / resolution) + 1;
                iy = round((point(2) - ylim(1)) / resolution) + 1;
                
                % Check bounds
                if ix >= 1 && ix <= size(grid, 2) && iy >= 1 && iy <= size(grid, 1)
                    % Mark as occupied
                    grid(iy, ix) = 1;
                end
            end
        end
    end
    % Note: We could mark free space too, but keeping it simple for now
end

end
