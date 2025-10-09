function waypoints = load_waypoints(filename)
% LOAD_WAYPOINTS - Load waypoint data from JSON .wpt file
%
% Parses a JSON waypoint file and returns a structured waypoint definition.
%
% SYNTAX:
%   waypoints = load_waypoints(filename)
%
% INPUTS:
%   filename - Path to .wpt file (JSON format)
%
% OUTPUTS:
%   waypoints - Structure containing:
%               .metadata - Optional metadata (name, description, etc.)
%               .time     - Time vector (Nx1) [s]
%               .position - Position matrix (Nx3) [x, y, z] in meters
%               .yaw      - Yaw angles (Nx1) [rad], NaN for auto-calculate
%               .labels   - Cell array of waypoint labels (Nx1)
%
% WAYPOINT FILE FORMAT (.wpt as JSON):
%   {
%     "metadata": {
%       "name": "Trajectory Name",
%       "description": "Description",
%       "created": "YYYY-MM-DD",
%       "vehicle": "vehicle_id"
%     },
%     "waypoints": [
%       {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
%       {"label": "climb", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
%       ...
%     ]
%   }
%
% YAW HANDLING:
%   - Explicit value (e.g., 1.57): UAV orients to specified heading
%   - null (in JSON) / NaN (in MATLAB): Auto-calculate from velocity direction
%
% EXAMPLE:
%   wpt = load_waypoints('./trajectories/basic_maneuver.wpt');
%   fprintf('Loaded: %s\n', wpt.metadata.name);
%   fprintf('Waypoints: %d\n', length(wpt.time));
%
% See also: jsondecode, generate_trajectory

% Author: Trey Copeland
% Date: 2025-10-09

%% Validate input
assert(ischar(filename) || isstring(filename), 'Filename must be a string');
assert(exist(filename, 'file') == 2, 'File not found: %s', filename);

%% Read and parse JSON file
try
    % Read file as text
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    json_text = fread(fid, '*char')';
    fclose(fid);
    
    % Parse JSON
    data = jsondecode(json_text);
catch ME
    error('Failed to parse JSON file: %s\nError: %s', filename, ME.message);
end

%% Validate JSON structure
assert(isfield(data, 'waypoints'), 'JSON must contain "waypoints" field');
assert(~isempty(data.waypoints), 'Waypoints array cannot be empty');

%% Extract metadata (optional)
if isfield(data, 'metadata')
    waypoints.metadata = data.metadata;
else
    waypoints.metadata = struct('name', 'Unnamed', 'description', '');
end

%% Extract waypoint data
n_waypoints = length(data.waypoints);

% Pre-allocate arrays
waypoints.time = zeros(n_waypoints, 1);
waypoints.position = zeros(n_waypoints, 3);
waypoints.yaw = nan(n_waypoints, 1);  % NaN = auto-calculate
waypoints.labels = cell(n_waypoints, 1);

% Parse each waypoint
for i = 1:n_waypoints
    wp = data.waypoints(i);
    
    % Required fields
    assert(isfield(wp, 'time'), 'Waypoint %d missing "time" field', i);
    assert(isfield(wp, 'x'), 'Waypoint %d missing "x" field', i);
    assert(isfield(wp, 'y'), 'Waypoint %d missing "y" field', i);
    assert(isfield(wp, 'z'), 'Waypoint %d missing "z" field', i);
    
    waypoints.time(i) = wp.time;
    waypoints.position(i, :) = [wp.x, wp.y, wp.z];
    
    % Optional label
    if isfield(wp, 'label')
        waypoints.labels{i} = wp.label;
    else
        waypoints.labels{i} = sprintf('wp_%d', i);
    end
    
    % Yaw: explicit value or null (NaN for auto)
    if isfield(wp, 'yaw')
        if isempty(wp.yaw) || (isnumeric(wp.yaw) && isnan(wp.yaw))
            waypoints.yaw(i) = NaN;  % Auto-calculate
        else
            waypoints.yaw(i) = wp.yaw;
        end
    else
        waypoints.yaw(i) = NaN;  % Default to auto
    end
end

%% Validate waypoint data
% Check time monotonicity
assert(all(diff(waypoints.time) > 0), 'Waypoint times must be strictly increasing');

% Check for NaN/Inf in positions
assert(all(isfinite(waypoints.position(:))), 'Waypoint positions contain NaN or Inf');

% Warn if all yaw values are NaN
if all(isnan(waypoints.yaw))
    fprintf('Note: All yaw angles set to auto-calculate from velocity\n');
end

%% Display summary
fprintf('Loaded waypoint file: %s\n', filename);
if isfield(waypoints.metadata, 'name')
    fprintf('  Name: %s\n', waypoints.metadata.name);
end
fprintf('  Waypoints: %d\n', n_waypoints);
fprintf('  Duration: %.1f seconds\n', waypoints.time(end) - waypoints.time(1));
fprintf('  Trajectory extent: [%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f] m\n', ...
        min(waypoints.position(:,1)), max(waypoints.position(:,1)), ...
        min(waypoints.position(:,2)), max(waypoints.position(:,2)), ...
        min(waypoints.position(:,3)), max(waypoints.position(:,3)));

end
