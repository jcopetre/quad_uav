function test_trajectory_auto()
% TEST_TRAJECTORY_AUTO - Verify automatic trajectory method selection

fprintf('Running Unit Tests: Auto Trajectory Selector\n');
fprintf('============================================\n\n');

setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

%% Test 1: Short segments → Interpolation
fprintf('Test 1: Short segments (should select interpolation)... ');

wpt1.time = [0; 1; 2]';  % 1s segments
wpt1.position = [0 0 1; 2 0 1; 4 0 1];
wpt1.yaw = [0; 0; 0];

traj1 = generate_trajectory_auto(wpt1, params, 0.01);

assert(strcmp(traj1.method, 'interpolation'), ...
       'Should select interpolation for 1s segments');
assert(traj1.selection_criteria.min_segment == 1.0, ...
       'Should detect 1s minimum segment');

fprintf('PASS\n');

%% Test 2: Long segments → Minimum Snap
fprintf('Test 2: Long segments (should select minsnap)... ');

wpt2.time = [0; 4; 8]';  % 4s segments
wpt2.position = [0 0 1; 2 0 1; 4 0 1];
wpt2.yaw = [0; 0; 0];

traj2 = generate_trajectory_auto(wpt2, params, 0.01);

assert(strcmp(traj2.method, 'minsnap'), ...
       'Should select minsnap for 4s segments');
assert(traj2.selection_criteria.min_segment == 4.0, ...
       'Should detect 4s minimum segment');

fprintf('PASS\n');

%% Test 3: Mixed segments → Use minimum (conservative)
fprintf('Test 3: Mixed segment durations... ');

wpt3.time = [0; 2; 6; 8]';  % 2s, 4s, 2s segments
wpt3.position = [0 0 1; 2 0 1; 4 0 1; 6 0 1];
wpt3.yaw = [0; 0; 0; 0];

traj3 = generate_trajectory_auto(wpt3, params, 0.01);

assert(strcmp(traj3.method, 'interpolation'), ...
       'Should select interpolation when ANY segment is short');
assert(traj3.selection_criteria.min_segment == 2.0, ...
       'Should detect 2s minimum segment');

fprintf('PASS\n');

%% Test 4: Boundary case (exactly 3s)
fprintf('Test 4: Boundary case (3s segments)... ');

wpt4.time = [0; 3; 6]';  % Exactly 3s segments
wpt4.position = [0 0 1; 2 0 1; 4 0 1];
wpt4.yaw = [0; 0; 0];

traj4 = generate_trajectory_auto(wpt4, params, 0.01);

assert(strcmp(traj4.method, 'minsnap'), ...
       'Should select minsnap at threshold (3s ≥ 3s)');

fprintf('PASS\n');

%% Test 5: Real trajectory file
fprintf('Test 5: Real trajectory (basic_maneuver.wpt)... ');

wpt5 = load_waypoints('basic_maneuver.wpt');
traj5 = generate_trajectory_auto(wpt5, params, 0.01);

% Should have metadata
assert(isfield(traj5, 'method'), 'Should have method field');
assert(isfield(traj5, 'method_reason'), 'Should have reason field');
assert(isfield(traj5, 'selection_criteria'), 'Should have criteria field');

% Check acceleration is reasonable
max_acc = max(vecnorm(traj5.acceleration, 2, 2));
assert(max_acc < 10, 'Acceleration should be reasonable (< 10 m/s²)');

fprintf('PASS (method: %s, max accel: %.2f m/s²)\n', traj5.method, max_acc);

fprintf('\n============================================\n');
fprintf('All Auto-Selector Tests Passed! ✓\n\n');

end