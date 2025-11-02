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
%% Generate yaw trajectory
trajectory.yaw = zeros(n_points, 1);

% Handle first waypoint if NaN (default to facing north)
if isnan(wpt_yaw(1)) % Check for Constants.AUTO_YAW
    wpt_yaw(1) = 0;
end

% Find all defined (non-NaN) waypoint indices
defined_idx = find(~isnan(wpt_yaw)); % Not Constants.AUTO_YAW

if ~isempty(defined_idx)
    if length(defined_idx) == 1
        % Only one defined yaw - hold it constant
        trajectory.yaw(:) = wpt_yaw(defined_idx);
    else
        % Multiple defined yaws - interpolate between them
        trajectory.yaw = interp1(wpt_time(defined_idx), wpt_yaw(defined_idx), ...
                                 trajectory.time, 'pchip', 'extrap');
        
        % Clamp extrapolation: hold first/last values beyond boundaries
        trajectory.yaw(trajectory.time < wpt_time(defined_idx(1))) = wpt_yaw(defined_idx(1));
        trajectory.yaw(trajectory.time > wpt_time(defined_idx(end))) = wpt_yaw(defined_idx(end));
    end
else
    % All NaN (shouldn't happen after defaulting first to 0, but be safe)
    % Calculate from velocity direction
    for i = 1:n_points
        vx = trajectory.velocity(i, 1);
        vy = trajectory.velocity(i, 2);
        if sqrt(vx^2 + vy^2) > 1e-6
            trajectory.yaw(i) = atan2(vy, vx);
        else
            if i == 1
                trajectory.yaw(i) = 0;
            else
                trajectory.yaw(i) = trajectory.yaw(i-1);
            end
        end
    end
end

%% Compute feedforward attitude (roll and pitch) from acceleration
% For a quadrotor, the thrust vector must oppose the net force vector:
%   F_net = m*a_desired + m*g*[0; 0; 1]
%
% In the small-angle approximation:
%   pitch ≈ asin(ax / g)           (positive pitch for forward accel)
%   roll  ≈ asin(-ay / (g*cos(θ))) (negative roll for rightward accel)
%
% These feedforward attitudes help the LQR controller by providing the
% reference orientation needed to achieve desired accelerations.

trajectory.roll = zeros(n_points, 1);
trajectory.pitch = zeros(n_points, 1);

for i = 1:n_points
    ax = trajectory.acceleration(i, 1);
    ay = trajectory.acceleration(i, 2);
    
    % Compute required pitch from forward acceleration
    % Clamp argument to prevent numerical issues (limit to ±45°)
    pitch_arg = ax / params.g;
    pitch_arg = max(-0.707, min(0.707, pitch_arg));  % sin(45°) ≈ 0.707
    trajectory.pitch(i) = asin(pitch_arg);
    
    % Compute required roll from lateral acceleration
    cos_theta = cos(trajectory.pitch(i));
    if abs(cos_theta) > 0.1  % Avoid division by zero near 90° pitch
        roll_arg = -ay / (params.g * cos_theta);
        roll_arg = max(-0.707, min(0.707, roll_arg));
        trajectory.roll(i) = asin(roll_arg);
    else
        % Near vertical flight - no meaningful roll correction
        trajectory.roll(i) = 0;
    end
end

fprintf('  Max roll: %.2f deg\n', max(abs(rad2deg(trajectory.roll))));
fprintf('  Max pitch: %.2f deg\n', max(abs(rad2deg(trajectory.pitch))));

%% Assemble attitude vector
trajectory.attitude = [trajectory.roll, trajectory.pitch, trajectory.yaw];

%% Compute angular velocity for trajectory tracking
% CRITICAL: Angular velocity should be minimal for feedforward trajectories.
% The feedforward attitudes are kinematic (orientation needed for acceleration)
% but don't imply specific angular velocities.
%
% Setting omega from Euler angle derivatives causes massive numerical errors:
%   - Original approach: omega = W^-1 * euler_dot → 1800+ deg/s (impossible!)
%   - New approach: Set p=q=0, compute r from yaw changes
%
% This allows the LQR controller to determine how fast to rotate based on
% tracking error and gains, rather than demanding impossible angular velocities.

trajectory.omega = zeros(n_points, 3);

% Compute yaw rate for coordinated turns
yaw_rate = gradient(trajectory.yaw, dt);

% Limit to reasonable values (avoid wrap-around issues)
max_yaw_rate = deg2rad(90);  % 90 deg/s maximum
yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate));

trajectory.omega(:, 3) = yaw_rate;  % r (yaw rate only)
% p and q remain zero - let controller handle roll/pitch changes

% Store method information for reproducibility
trajectory.method = 'interpolation';
trajectory.method_reason = 'User requested interpolation directly';

%% Display summary
fprintf('Generated trajectory:\n');
fprintf('  Duration: %.2f seconds\n', t_end - t_start);
fprintf('  Points: %d (dt = %.3f s)\n', n_points, dt);
fprintf('  Max velocity: %.2f m/s\n', max(sqrt(sum(trajectory.velocity.^2, 2))));
fprintf('  Max acceleration: %.2f m/s²\n', max(sqrt(sum(trajectory.acceleration.^2, 2))));
fprintf('  Max roll: %.2f deg\n', max(abs(rad2deg(trajectory.roll))));
fprintf('  Max pitch: %.2f deg\n', max(abs(rad2deg(trajectory.pitch))));

end
