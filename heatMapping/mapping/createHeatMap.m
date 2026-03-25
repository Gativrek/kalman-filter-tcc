function heatMap = createHeatMap(xlim, ylim, resolution)
% Creates an empty heat map for trajectory tracking
% Simple counter-based approach
%
% INPUTS:
%   xlim       - [xmin, xmax] in meters
%   ylim       - [ymin, ymax] in meters
%   resolution - Grid cell size in meters
%
% OUTPUT:
%   heatMap - Matrix initialized with zeros (visit counts)

% Calculate grid size
nx = ceil((xlim(2) - xlim(1)) / resolution);
ny = ceil((ylim(2) - ylim(1)) / resolution);

% Initialize with zeros
heatMap = zeros(ny, nx);

end
