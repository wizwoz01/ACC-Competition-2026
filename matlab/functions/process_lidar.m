function [filtered_points, obstacles] = process_lidar(lidar_data, config)
%PROCESS_LIDAR Process LIDAR point cloud data
%
%   [filtered_points, obstacles] = process_lidar(lidar_data, config)
%
%   Inputs:
%       lidar_data - Nx3 matrix of LIDAR points [x, y, z] or Nx2 [x, y]
%       config     - Configuration struct (optional)
%                    .min_range     - Minimum valid range (default: 0.1)
%                    .max_range     - Maximum valid range (default: 10.0)
%                    .ground_height - Ground plane height (default: -0.1)
%                    .cluster_dist  - Clustering distance (default: 0.3)
%
%   Outputs:
%       filtered_points - Filtered point cloud
%       obstacles       - Struct array of detected obstacles
%
%   Example:
%       [points, obs] = process_lidar(qcar.read_lidar());

    % Default configuration
    if nargin < 2 || isempty(config)
        config = struct();
    end
    if ~isfield(config, 'min_range'), config.min_range = 0.1; end
    if ~isfield(config, 'max_range'), config.max_range = 10.0; end
    if ~isfield(config, 'ground_height'), config.ground_height = -0.1; end
    if ~isfield(config, 'cluster_dist'), config.cluster_dist = 0.3; end
    
    % Handle 2D or 3D data
    if size(lidar_data, 2) == 2
        lidar_data = [lidar_data, zeros(size(lidar_data, 1), 1)];
    end
    
    % Calculate ranges
    ranges = sqrt(lidar_data(:,1).^2 + lidar_data(:,2).^2);
    
    % Filter by range
    valid_range = ranges >= config.min_range & ranges <= config.max_range;
    
    % Filter ground points
    above_ground = lidar_data(:,3) > config.ground_height;
    
    % Combined filter
    valid_idx = valid_range & above_ground;
    filtered_points = lidar_data(valid_idx, :);
    
    % Simple obstacle clustering (basic implementation)
    obstacles = cluster_obstacles(filtered_points, config.cluster_dist);
end

function obstacles = cluster_obstacles(points, cluster_dist)
%CLUSTER_OBSTACLES Simple clustering of obstacle points
    
    obstacles = struct('center', {}, 'points', {}, 'distance', {});
    
    if isempty(points)
        return;
    end
    
    % Simple grid-based clustering
    grid_size = cluster_dist;
    grid_x = floor(points(:,1) / grid_size);
    grid_y = floor(points(:,2) / grid_size);
    
    % Find unique grid cells
    grid_cells = unique([grid_x, grid_y], 'rows');
    
    for i = 1:size(grid_cells, 1)
        cell = grid_cells(i, :);
        idx = grid_x == cell(1) & grid_y == cell(2);
        cluster_points = points(idx, :);
        
        if size(cluster_points, 1) >= 3  % Minimum points for obstacle
            obs.center = mean(cluster_points, 1);
            obs.points = cluster_points;
            obs.distance = sqrt(obs.center(1)^2 + obs.center(2)^2);
            obstacles(end+1) = obs;
        end
    end
end

