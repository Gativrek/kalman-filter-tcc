function heatMap = updateHeatMap(heatMap, xRobot, P, xlim, ylim, resolution)
% Updates heat map with Gaussian spreading based on uncertainty
% VERSION 2 - Gaussian accumulation
%
% INPUTS:
%   heatMap    - Current heat map
%   xRobot     - Robot position estimate [x; y; theta]
%   P          - Covariance matrix (at least 3x3, uses first 2x2 for position)
%   xlim, ylim - Grid bounds
%   resolution - Grid cell size
%
% OUTPUT:
%   heatMap - Updated heat map

% Extract position and position covariance
pos = xRobot(1:2);
P_pos = P(1:2, 1:2);

% Convert center position to grid coordinates
ix_center = round((pos(1) - xlim(1)) / resolution) + 1;
iy_center = round((pos(2) - ylim(1)) / resolution) + 1;

% Define neighborhood size (5x5 cells around center)
neighborhood = 2;  % cells in each direction

% Loop through neighborhood
for di = -neighborhood:neighborhood
    for dj = -neighborhood:neighborhood
        
        ix = ix_center + di;
        iy = iy_center + dj;
        
        % Check bounds
        if ix >= 1 && ix <= size(heatMap, 2) && iy >= 1 && iy <= size(heatMap, 1)
            
            % Cell center position in world coordinates
            cell_x = xlim(1) + (ix - 0.5) * resolution;
            cell_y = ylim(1) + (iy - 0.5) * resolution;
            cell_pos = [cell_x; cell_y];
            
            % Distance from robot position to cell center
            distance = cell_pos - pos;
            
            % Calculate Gaussian weight
            weight = gaussianWeight(distance, P_pos);
            
            % Accumulate weighted probability
            heatMap(iy, ix) = heatMap(iy, ix) + weight;
        end
    end
end

end