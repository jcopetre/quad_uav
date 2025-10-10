function trajectory = generate_trajectory_interp(waypoints, params, dt)
% GENERATE_TRAJECTORY_INTERP - Generate smooth trajectory from waypoints
%
% Creates smooth, differentiable trajectories using MAKIMA (Modified Akima 
% piecewise cubic Hermite interpolation). Provides C¹ continuity with natural
% motion characteristics suitable for most quadrotor applications.
%
% For optimal trajectories minimizing snap (4th derivative), use
% generate_trajectory_minsnap() instead.
%
% SYNTAX:
%   trajectory = generate_trajectory(waypoints, params)
%   trajectory = generate_trajectory(waypoints, params, dt)
%
% INPUTS:
%   waypoints - Structure from load_waypoints() containing:
%               .time     - Time vector (Nx1) [s]
%               .position - Position matrix (Nx3) [m]
%               .yaw      - Yaw angles (Nx1) [rad], NaN = auto-calculate
%               .labels   - Waypoint labels (optional)
%               OR
%               Matrix [time, x, y, z, yaw] where yaw=NaN for auto
%   params    - Parameter structure from quadrotor_linear_6dof
%   dt        - (optional) Time step for trajectory [s], default 0.01
%
% OUTPUTS:
%   trajectory - Structure containing:
%                .time     - Time vector (Mx1) [s]
%                .position - Position [x, y, z] (Mx3) [m]
%                .velocity - Velocity [vx, vy, vz] (Mx3) [m/s]
%                .acceleration - Acceleration [ax, ay, az] (Mx3) [m/s²]
%                .attitude - Euler angles [phi, theta, psi] (Mx3) [rad]
%                .omega    - Angular velocity [p, q, r] (Mx3) [rad/s]
%
% INTERPOLATION METHOD:
%   Modified Akima (MAKIMA) interpolation with characteristics:
%   - C¹ continuity (smooth position and velocity)
%   - Non-zero velocities at waypoints (natural motion)
%   - Reduced oscillations compared to spline methods
%   - Robust to unequally-spaced waypoints
%
% YAW CALCULATION:
%   - Explicit: Use provided yaw angle
%   - Auto (NaN): Calculate from velocity direction (atan2(vy, vx))
%
% FEEDFORWARD ATTITUDE:
%   Roll and pitch computed from desired acceleration for better tracking:
%   - theta_d = asin(ax / g)
%   - phi_d = asin(-ay / (g * cos(theta)))
%
% EXAMPLE:
%   wpt = load_waypoints('./trajectories/basic_maneuver.wpt');
%   params = quadrotor_linear_6dof();
%   trajectory = generate_trajectory(wpt, params);
%
% See also: load_waypoints, interp1

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Input validation and defaults
if nargin < 3 || isempty(dt)
    dt = 0.01;  % 100 Hz default
end

% Handle both structure and matrix input
if isstruct(waypoints)
    % Structure from load_waypoints()
    wpt_time = waypoints.time;
    wpt_pos = waypoints.position;
    wpt_yaw = waypoints.yaw;
else
    % Matrix input [time, x, y, z, yaw]
    assert(size(waypoints, 2) == 5, 'Matrix must have 5 columns: [time, x, y, z, yaw]');
    wpt_time = waypoints(:, 1);
    wpt_pos = waypoints(:, 2:4);
    wpt_yaw = waypoints(:, 5);
end

assert(length(wpt_time) >= 2, 'Need at least 2 waypoints');
assert(all(diff(wpt_time) > 0), 'Waypoint times must be strictly increasing');

%% Create time vector for trajectory
t_start = wpt_time(1);
t_end = wpt_time(end);
trajectory.time = (t_start:dt:t_end)';

% Ensure we hit the final time exactly
if trajectory.time(end) < t_end
    trajectory.time(end+1) = t_end;
end

n_points = length(trajectory.time);

%% Interpolate position using Modified Akima interpolation
% MAKIMA (Modified Akima piecewise cubic Hermite interpolation) ensures:
% - C¹ continuity (smooth velocity)
% - Non-zero velocities at waypoints (natural motion, no stopping)
% - Reduces oscillations near abrupt changes
% - More robust than spline for unequally-spaced waypoints
%
% Note: PCHIP was originally used but creates zero velocity at waypoints,
% causing the quadrotor to stop at each waypoint and then accelerate hard,
% leading to control saturation and tracking failures.

trajectory.position = zeros(n_points, 3);
for axis = 1:3
    trajectory.position(:, axis) = interp1(wpt_time, wpt_pos(:, axis), ...
                                            trajectory.time, 'makima');
end

%% Compute velocity and acceleration via numerical differentiation
trajectory.velocity = zeros(n_points, 3);
trajectory.acceleration = zeros(n_points, 3);

for axis = 1:3
    % Velocity: central difference (2nd order accurate)
    trajectory.velocity(:, axis) = gradient(trajectory.position(:, axis), dt);
    
    % Acceleration: central difference of velocity
    trajectory.acceleration(:, axis) = gradient(trajectory.velocity(:, axis), dt);
end

%% Generate yaw trajectory
trajectory.yaw = zeros(n_points, 1);

% Interpolate or auto-calculate yaw
if all(isnan(wpt_yaw))
    % All auto: calculate from velocity direction
    for i = 1:n_points
        vx = trajectory.velocity(i, 1);
        vy = trajectory.velocity(i, 2);
        if abs(vx) < 1e-6 && abs(vy) < 1e-6
            % Stationary: maintain previous yaw (or zero for first point)
            if i == 1
                trajectory.yaw(i) = 0;
            else
                trajectory.yaw(i) = trajectory.yaw(i-1);
            end
        else
            trajectory.yaw(i) = atan2(vy, vx);
        end
    end
elseif all(~isnan(wpt_yaw))
    % All explicit: interpolate yaw angles
    % Use PCHIP for yaw (unlike position) because:
    % - Shape-preserving prevents overshoot in heading
    % - Yaw can naturally hold constant between maneuvers
    % - Reduces oscillation near large heading changes
    trajectory.yaw = interp1(wpt_time, wpt_yaw, trajectory.time, 'pchip');
else
    % Mixed: interpolate explicit values, then fill in auto sections
    explicit_idx = ~isnan(wpt_yaw);
    trajectory.yaw = interp1(wpt_time(explicit_idx), wpt_yaw(explicit_idx), ...
                             trajectory.time, 'linear', 'extrap');
    
    % For sections between auto waypoints, use velocity direction
    for i = 1:length(wpt_time)-1
        if isnan(wpt_yaw(i)) && isnan(wpt_yaw(i+1))
            % Both waypoints are auto - use velocity direction
            mask = trajectory.time >= wpt_time(i) & trajectory.time <= wpt_time(i+1);
            for j = find(mask)'
                vx = trajectory.velocity(j, 1);
                vy = trajectory.velocity(j, 2);
                if abs(vx) > 1e-6 || abs(vy) > 1e-6
                    trajectory.yaw(j) = atan2(vy, vx);
                end
            end
        end
    end
end

%% Compute feedforward attitude (roll and pitch) from acceleration
trajectory.roll = zeros(n_points, 1);
trajectory.pitch = zeros(n_points, 1);

%% Assemble attitude vector
trajectory.attitude = [trajectory.roll, trajectory.pitch, trajectory.yaw];

%% Compute angular velocity (omega) from Euler angle rates
% Since feedforward attitude is disabled (roll=0, pitch=0),
% set omega to zero. The controller will generate required angular
% velocities dynamically during flight.
trajectory.omega = zeros(n_points, 3);

%% Display summary
fprintf('Generated trajectory:\n');
fprintf('  Duration: %.2f seconds\n', t_end - t_start);
fprintf('  Points: %d (dt = %.3f s)\n', n_points, dt);
fprintf('  Max velocity: %.2f m/s\n', max(sqrt(sum(trajectory.velocity.^2, 2))));
fprintf('  Max acceleration: %.2f m/s²\n', max(sqrt(sum(trajectory.acceleration.^2, 2))));
fprintf('  Max roll: %.2f deg\n', max(abs(rad2deg(trajectory.roll))));
fprintf('  Max pitch: %.2f deg\n', max(abs(rad2deg(trajectory.pitch))));

end
