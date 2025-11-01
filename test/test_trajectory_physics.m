function test_trajectory_physics()
% TEST_TRAJECTORY_PHYSICS - Validate physical consistency of trajectory outputs
%
% Tests that trajectory generators produce physically valid outputs where:
%   1. Velocity is the numerical derivative of position
%   2. Acceleration is the numerical derivative of velocity
%   3. Jerk (if present) is the derivative of acceleration
%   4. Attitude angles are computed correctly from thrust vector
%   5. Angular velocities produce correct attitude rates
%   6. Boundary conditions match implementation (zero vel at start/end only)
%   7. Velocity is continuous at interior waypoints
%
% This validates the PHYSICS, not just structure.
%
% TEST STYLE: Style 1 (simple assert-based)
%   - Stops on first failure when run directly
%   - Continues through all tests when run via run_tests.m
%
% TODO: Consider standardizing all unit tests to Style 2 (internal try-catch)
%       for consistent behavior when run directly vs. via test runner.

fprintf('Running Physics Validation Tests: Trajectory Generators\n');
fprintf('========================================================\n\n');

setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

%% Test 1: Interpolation - Velocity is derivative of position
fprintf('Test 1: Interpolation velocity = d(position)/dt... ');

wpt.time = [0; 3; 6]';
wpt.position = [0 0 1; 3 0 1; 6 3 1];
wpt.yaw = zeros(3, 1);

traj = generate_trajectory_interp(wpt, params, 0.01);

% Compute numerical derivative of position
dt = diff(traj.time);
vel_numeric = diff(traj.position) ./ dt;

% Compare with reported velocity (skip first point for derivative)
vel_error = vel_numeric - traj.velocity(2:end, :);
max_vel_error = max(abs(vel_error(:)));

% Should match within numerical tolerance
assert(max_vel_error < 0.05, ...
       'Velocity error too large: %.3f m/s', max_vel_error);

fprintf('PASS (max error: %.4f m/s)\n', max_vel_error);

%% Test 2: Interpolation - Acceleration is derivative of velocity
fprintf('Test 2: Interpolation acceleration = d(velocity)/dt... ');

% Compute numerical derivative of velocity
dt = diff(traj.time);
acc_numeric = diff(traj.velocity) ./ dt;

% Compare with reported acceleration (skip first point)
acc_error = acc_numeric - traj.acceleration(2:end, :);
max_acc_error = max(abs(acc_error(:)));

% Should match within numerical tolerance
assert(max_acc_error < 0.5, ...
       'Acceleration error too large: %.3f m/s²', max_acc_error);

fprintf('PASS (max error: %.4f m/s²)\n', max_acc_error);

%% Test 3: Minimum Snap - All derivatives consistent
fprintf('Test 3: Minimum snap derivative consistency... ');

wpt_long.time = [0; 4; 8]';
wpt_long.position = [0 0 1; 2 0 1; 4 2 1];
wpt_long.yaw = zeros(3, 1);

traj_ms = generate_trajectory_minsnap(wpt_long, params, 0.01);

% Check velocity = d(position)/dt
dt = diff(traj_ms.time);
vel_numeric = diff(traj_ms.position) ./ dt;
vel_error = max(abs(vel_numeric - traj_ms.velocity(2:end, :)), [], 'all');

% Check acceleration = d(velocity)/dt
acc_numeric = diff(traj_ms.velocity) ./ dt;
acc_error = max(abs(acc_numeric - traj_ms.acceleration(2:end, :)), [], 'all');

% Check jerk = d(acceleration)/dt
jerk_numeric = diff(traj_ms.acceleration) ./ dt;
jerk_error = max(abs(jerk_numeric - traj_ms.jerk(2:end, :)), [], 'all');

% All should be consistent
assert(vel_error < 0.05, 'Velocity inconsistent: %.3f', vel_error);
assert(acc_error < 0.5, 'Acceleration inconsistent: %.3f', acc_error);
assert(jerk_error < 5.0, 'Jerk inconsistent: %.3f', jerk_error);

fprintf('PASS\n');
fprintf('  Velocity error: %.4f m/s\n', vel_error);
fprintf('  Acceleration error: %.4f m/s²\n', acc_error);
fprintf('  Jerk error: %.4f m/s³\n', jerk_error);

%% Test 4: Attitude angles from thrust vector
fprintf('Test 4: Attitude computation from acceleration... ');

% For hover+forward: thrust must counteract gravity and provide forward accel
% Expected roll ≈ 0, pitch = atan2(ax, az+g)
wpt_fwd.time = [0; 5]';
wpt_fwd.position = [0 0 1; 5 0 1];
wpt_fwd.yaw = zeros(2, 1);

traj_fwd = generate_trajectory_interp(wpt_fwd, params, 0.01);

% Find mid-trajectory point (constant velocity region)
mid_idx = round(length(traj_fwd.time) / 2);

% Compute expected pitch from acceleration
ax = traj_fwd.acceleration(mid_idx, 1);
az = traj_fwd.acceleration(mid_idx, 3);
expected_pitch = atan2(ax, az + params.g);

% Check computed attitude
actual_pitch = traj_fwd.attitude(mid_idx, 2);
pitch_error = abs(expected_pitch - actual_pitch);

assert(pitch_error < deg2rad(1), ...
       'Pitch angle error: %.2f deg', rad2deg(pitch_error));

fprintf('PASS (pitch error: %.3f deg)\n', rad2deg(pitch_error));

%% Test 5: Roll angle from lateral acceleration
fprintf('Test 5: Roll computation from lateral acceleration... ');

% Trajectory with lateral motion (y-direction)
wpt_lat.time = [0; 5]';
wpt_lat.position = [0 0 1; 0 5 1];
wpt_lat.yaw = zeros(2, 1);

traj_lat = generate_trajectory_interp(wpt_lat, params, 0.01);

% Find mid-trajectory point
mid_idx = round(length(traj_lat.time) / 2);

% Compute expected roll from acceleration
ay = traj_lat.acceleration(mid_idx, 2);
az = traj_lat.acceleration(mid_idx, 3);
expected_roll = atan2(-ay, az + params.g);

% Check computed attitude
actual_roll = traj_lat.attitude(mid_idx, 1);
roll_error = abs(expected_roll - actual_roll);

assert(roll_error < deg2rad(1), ...
       'Roll angle error: %.2f deg', rad2deg(roll_error));

fprintf('PASS (roll error: %.3f deg)\n', rad2deg(roll_error));

%% Test 6: Yaw rate consistency with yaw changes
fprintf('Test 6: Yaw rate = d(yaw)/dt... ');

% Trajectory with yaw rotation
wpt_yaw.time = [0; 5]';
wpt_yaw.position = [0 0 1; 0 0 1];
wpt_yaw.yaw = [0; pi/2];

traj_yaw = generate_trajectory_interp(wpt_yaw, params, 0.01);

% Compute numerical derivative of yaw
dt = diff(traj_yaw.time);
yaw_rate_numeric = diff(traj_yaw.attitude(:, 3)) ./ dt;

% Compare with omega_z (yaw rate)
yaw_rate_error = yaw_rate_numeric - traj_yaw.omega(2:end, 3);
max_yaw_error = max(abs(yaw_rate_error));

% Should match within tolerance
assert(max_yaw_error < 0.05, ...
       'Yaw rate error: %.3f rad/s', max_yaw_error);

fprintf('PASS (max error: %.4f rad/s)\n', max_yaw_error);

%% Test 7: Minimum snap boundary conditions (start/end zero velocity)
fprintf('Test 7: Minimum snap zero velocity at start/end... ');

wpt_stop.time = [0; 3; 6]';
wpt_stop.position = [0 0 1; 2 0 1; 4 0 1];
wpt_stop.yaw = zeros(3, 1);

traj_stop = generate_trajectory_minsnap(wpt_stop, params, 0.01);

% Check velocity is near zero at START and END only
% (Interior waypoints can have non-zero velocity for smoother motion)
tolerance = 0.1;  % m/s

vel_start = norm(traj_stop.velocity(1, :));
vel_end = norm(traj_stop.velocity(end, :));

assert(vel_start < tolerance, ...
       'Start velocity too high: %.3f m/s', vel_start);
assert(vel_end < tolerance, ...
       'End velocity too high: %.3f m/s', vel_end);

fprintf('PASS (start: %.3f m/s, end: %.3f m/s)\n', vel_start, vel_end);

%% Test 7b: Interior waypoints have continuous (but non-zero) velocity
fprintf('Test 7b: Velocity continuity at interior waypoints... ');

% Check that velocity is continuous at the middle waypoint
% Find indices near the middle waypoint
mid_time = wpt_stop.time(2);
[~, idx_mid] = min(abs(traj_stop.time - mid_time));

% Velocity should be continuous (similar before/after waypoint)
vel_before = traj_stop.velocity(max(1, idx_mid-5), :);
vel_at = traj_stop.velocity(idx_mid, :);
vel_after = traj_stop.velocity(min(length(traj_stop.time), idx_mid+5), :);

% Check continuity (velocity shouldn't jump)
vel_jump_before = norm(vel_at - vel_before);
vel_jump_after = norm(vel_after - vel_at);

assert(vel_jump_before < 0.5, ...
       'Velocity discontinuity before waypoint: %.3f m/s', vel_jump_before);
assert(vel_jump_after < 0.5, ...
       'Velocity discontinuity after waypoint: %.3f m/s', vel_jump_after);

fprintf('PASS (continuous at interior waypoints)\n');

%% Test 7c: Explicit zero velocity specification
fprintf('Test 7c: Explicitly specified zero velocity at all waypoints... ');

% If you want zero velocity everywhere, explicitly specify it
wpt_explicit.time = [0; 3; 6]';
wpt_explicit.position = [0 0 1; 2 0 1; 4 0 1];
wpt_explicit.velocity = zeros(3, 3);  % Explicitly specify zero velocity
wpt_explicit.yaw = zeros(3, 1);

traj_explicit = generate_trajectory_minsnap(wpt_explicit, params, 0.01);

% Now ALL waypoints should have near-zero velocity
for i = 1:length(wpt_explicit.time)
    [~, idx] = min(abs(traj_explicit.time - wpt_explicit.time(i)));
    vel_mag = norm(traj_explicit.velocity(idx, :));
    
    assert(vel_mag < 0.1, ...
           'Waypoint %d velocity: %.3f m/s (expected ~0)', i, vel_mag);
end

fprintf('PASS (all waypoints < 0.1 m/s when explicitly specified)\n');

%% Test 8: Acceleration magnitude consistency
fprintf('Test 8: Acceleration magnitude = norm(acc vector)... ');

% Use 3+ waypoints to create curvature (2 waypoints = straight line = zero accel)
wpt_3d.time = [0; 2; 4]';
wpt_3d.position = [0 0 1; 1 0 1; 1 1 1];  % L-shaped path
wpt_3d.yaw = zeros(3, 1);

traj_3d = generate_trajectory_interp(wpt_3d, params, 0.01);

% Compute acceleration magnitude from components
acc_mag_computed = vecnorm(traj_3d.acceleration, 2, 2);

% All values should be positive (magnitude is always >= 0)
assert(all(acc_mag_computed >= 0), 'Acceleration magnitude negative');

% Should vary during trajectory (not constant due to corner)
acc_mag_range = max(acc_mag_computed) - min(acc_mag_computed);
assert(acc_mag_range > 0.1, 'Acceleration should vary during maneuver');

fprintf('PASS (magnitude range: %.2f m/s²)\n', acc_mag_range);

%% Test 9: Position continuity at waypoints
fprintf('Test 9: Position continuous through all waypoints... ');

wpt_multi.time = [0; 2; 4; 6; 8]';
wpt_multi.position = [0 0 1; 1 0 1; 1 1 1; 0 1 1; 0 0 1];
wpt_multi.yaw = zeros(5, 1);

traj_multi = generate_trajectory_interp(wpt_multi, params, 0.01);

% Check no discontinuities in position
pos_diff = diff(traj_multi.position);
max_pos_jump = max(abs(pos_diff(:)));

% With dt=0.01, position changes should be small
assert(max_pos_jump < 0.05, ...
       'Position discontinuity detected: %.3f m', max_pos_jump);

fprintf('PASS (max step: %.4f m)\n', max_pos_jump);

%% Test 10: Minimum snap produces bounded, smooth derivatives
fprintf('Test 10: Minimum snap smoothness (bounded snap)... ');

wpt_comp.time = [0; 5; 10]';
wpt_comp.position = [0 0 1; 3 0 1; 6 3 1];
wpt_comp.yaw = zeros(3, 1);

traj_minsnap = generate_trajectory_minsnap(wpt_comp, params, 0.01);

% Check that snap is finite and bounded
assert(all(isfinite(traj_minsnap.snap(:))), 'Snap should be finite');

snap_magnitude = vecnorm(traj_minsnap.snap, 2, 2);
max_snap = max(snap_magnitude);

% Snap should be reasonable (not excessive)
% For smooth trajectories, snap typically < 100 m/s^4
assert(max_snap < 100, 'Snap magnitude too large: %.2f m/s^4', max_snap);

% Check jerk is also bounded (should be smoother than interpolation typically)
jerk_magnitude = vecnorm(traj_minsnap.jerk, 2, 2);
max_jerk = max(jerk_magnitude);

assert(max_jerk < 50, 'Jerk magnitude too large: %.2f m/s^3', max_jerk);

fprintf('PASS\n');
fprintf('  Max jerk: %.2f m/s³\n', max_jerk);
fprintf('  Max snap: %.2f m/s⁴\n', max_snap);

fprintf('\n========================================================\n');
fprintf('All Physics Validation Tests Passed! ✓\n\n');

end