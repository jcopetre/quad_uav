function trajectory = generate_trajectory_auto(waypoints, params, dt)
% GENERATE_TRAJECTORY_AUTO - Intelligently select trajectory generation method
%
% Automatically chooses between minimum snap and interpolation based on
% segment duration to ensure physically reasonable trajectories.
%
% SELECTION CRITERIA (based on empirical testing):
%   Short segments (< 3s):  Use interpolation (minsnap too aggressive)
%   Long segments (≥ 3s):   Use minimum snap (optimal smoothness)
%
% RATIONALE:
%   Minimum snap optimization minimizes the 4th derivative (snap) but does
%   not directly constrain acceleration. For short time segments, this can
%   produce very high accelerations (e.g., 30 m/s² for 0.5s segments).
%   
%   Test results show:
%     - 0.5s segments: 30 m/s² acceleration (30° pitch) - UNSAFE
%     - 1.0s segments:  7 m/s² acceleration (30° pitch) - TOO HIGH
%     - 2.0s segments:  2 m/s² acceleration (11° pitch) - MARGINAL
%     - 3.0s segments:  1 m/s² acceleration ( 5° pitch) - REASONABLE
%
% SYNTAX:
%   trajectory = generate_trajectory_auto(waypoints, params)
%   trajectory = generate_trajectory_auto(waypoints, params, dt)
%
% INPUTS:
%   waypoints - Structure from load_waypoints() or matrix format
%   params    - Parameter structure from quadrotor_linear_6dof
%   dt        - (optional) Time step [s], default 0.01
%
% OUTPUTS:
%   trajectory - Generated trajectory with additional fields:
%                .method        - 'minsnap' or 'interpolation'
%                .method_reason - Human-readable explanation
%
% EXAMPLES:
%   % Automatic selection
%   wpt = load_waypoints('basic_maneuver.wpt');
%   traj = generate_trajectory_auto(wpt, params);
%   fprintf('Method: %s\n', traj.method);
%
%   % For manual control, use specific generators:
%   traj = generate_trajectory_minsnap(wpt, params);  % Force minsnap
%   traj = generate_trajectory_interp(wpt, params);   % Force interpolation
%
% See also: generate_trajectory_minsnap, generate_trajectory_interp

% Author: Trey Copeland
% Date: 2025-10-13

    if nargin < 3 || isempty(dt)
        dt = 0.01;
    end
    
    %% Extract waypoint timing information
    if isstruct(waypoints)
        wpt_time = waypoints.time;
        N_waypoints = length(wpt_time);
    else
        % Matrix format: [time, x, y, z, yaw]
        wpt_time = waypoints(:, 1);
        N_waypoints = size(waypoints, 1);
    end
    
    %% Analyze segment durations
    segment_durations = diff(wpt_time);
    min_segment = min(segment_durations);
    max_segment = max(segment_durations);
    avg_segment = mean(segment_durations);
    total_duration = wpt_time(end) - wpt_time(1);
    
    %% Decision threshold (based on empirical testing)
    SAFE_SEGMENT_THRESHOLD = 3.0;  % seconds
    
    %% Make decision
    if min_segment < SAFE_SEGMENT_THRESHOLD
        % Use interpolation (safer for short segments)
        method_reason = sprintf('Auto-selected: Short segment detected (%.2fs < %.2fs threshold)', ...
                           min_segment, SAFE_SEGMENT_THRESHOLD);
        fprintf('Auto-selecting: INTERPOLATION trajectory generation\n');
        fprintf('  Reason: %s\n', method_reason);
        fprintf('  Segment durations: min=%.2fs, avg=%.2fs, max=%.2fs\n', ...
                min_segment, avg_segment, max_segment);
        fprintf('  (Minimum snap would produce excessive accelerations)\n');
        
        trajectory = generate_trajectory_interp(waypoints, params, dt);
    else
        % Use minimum snap (optimal for long segments)
        method_reason = sprintf('Auto-selected: All segments >= %.2fs threshold', SAFE_SEGMENT_THRESHOLD);
        fprintf('  Reason: %s\n', method_reason);
        fprintf('Auto-selecting: MINIMUM SNAP trajectory generation\n');
        fprintf('  Segment durations: min=%.2fs, avg=%.2fs, max=%.2fs\n', ...
                min_segment, avg_segment, max_segment);
        fprintf('  (Segments are long enough for smooth optimization)\n');
        
        trajectory = generate_trajectory_minsnap(waypoints, params, dt);
    end
    
    %% Add metadata about selection
    trajectory.method_reason = method_reason;
    trajectory.selection_criteria = struct(...
        'min_segment', min_segment, ...
        'avg_segment', mean(segment_durations), ...
        'max_segment', max(segment_durations), ...
        'threshold', SAFE_SEGMENT_THRESHOLD, ...
        'auto_selected', true);
    
    %% Post-generation validation
    max_acc = max(vecnorm(trajectory.acceleration, 2, 2));
    expected_pitch = rad2deg(asin(min(max_acc / params.g, 0.9)));
    
    if expected_pitch > 15
        warning('Generated trajectory has high accelerations (%.2f m/s², pitch: %.1f°)', ...
                max_acc, expected_pitch);
        fprintf('  Consider: Increasing time between waypoints\n');
    end
    
end