function grid = createOccupancyGrid(xlim, ylim, resolution)
% Creates an empty occupancy grid
% SIMPLE VERSION - binary grid (0 = unknown/free, 1 = occupied)
%
% INPUTS:
%   xlim       - [xmin, xmax] in meters
%   ylim       - [ymin, ymax] in meters  
%   resolution - Grid cell size in meters
%
% OUTPUT:
%   grid - Binary occupancy grid (0 or 1)

% Calculate grid size
nx = ceil((xlim(2) - xlim(1)) / resolution);
ny = ceil((ylim(2) - ylim(1)) / resolution);

% Initialize grid with zeros (unknown/free)
grid = zeros(ny, nx);

end
