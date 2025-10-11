% test_trajectory.m
% Unit tests for generate_trajectory_interp function
%
% Tests trajectory generation from waypoints

function test_trajectory()
    fprintf('Running Unit Tests: generate_trajectory_interp\n');
    fprintf('========================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_simple_trajectory();
    test_trajectory_structure();
    test_waypoint_satisfaction();
    test_continuity();
    test_yaw_auto_calculation();
    test_yaw_explicit();
    test_matrix_input();
    test_time_sampling();
    
    fprintf('\n========================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: Simple Trajectory Generation
function test_simple_trajectory()
    fprintf('Test 1: Simple trajectory generation... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Create simple waypoint structure
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 1 0 1; 2 0 1];
    wpt.yaw = [0; NaN; 0];
    
    % Generate trajectory
    trajectory = generate_trajectory_interp(wpt, params);
    
    % Check structure exists
    assert(isstruct(trajectory), 'Should return structure');
    
    % Check all required fields
    required_fields = {'time', 'position', 'velocity', 'acceleration', 'attitude', 'omega'};
    for i = 1:length(required_fields)
        assert(isfield(trajectory, required_fields{i}), ...
               'Missing field: %s', required_fields{i});
    end
    
    fprintf('PASS\n');
end

%% Test 2: Trajectory Structure Validation
function test_trajectory_structure()
    fprintf('Test 2: Trajectory structure validation... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    wpt.time = [0; 2]';
    wpt.position = [0 0 0; 1 0 0];
    wpt.yaw = [0; 0];
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    n = length(trajectory.time);
    
    % Check dimensions
    assert(size(trajectory.position, 1) == n, 'Position length mismatch');
    assert(size(trajectory.position, 2) == 3, 'Position should have 3 columns');
    assert(size(trajectory.velocity, 1) == n, 'Velocity length mismatch');
    assert(size(trajectory.acceleration, 1) == n, 'Acceleration length mismatch');
    assert(size(trajectory.attitude, 1) == n, 'Attitude length mismatch');
    assert(size(trajectory.omega, 1) == n, 'Omega length mismatch');
    
    % Check all values are finite
    assert(all(isfinite(trajectory.position(:))), 'Position contains non-finite values');
    assert(all(isfinite(trajectory.velocity(:))), 'Velocity contains non-finite values');
    assert(all(isfinite(trajectory.attitude(:))), 'Attitude contains non-finite values');
    
    fprintf('PASS\n');
end

%% Test 3: Waypoint Satisfaction
function test_waypoint_satisfaction()
    fprintf('Test 3: Waypoint satisfaction... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    wpt.time = [0; 1; 2]';
    wpt.position = [0 0 0; 1 1 1; 2 0 1];
    wpt.yaw = [0; pi/4; pi/2];
    
    trajectory = generate_trajectory_interp(wpt, params, 0.01);
    
    % Check that trajectory passes through waypoints
    for i = 1:length(wpt.time)
        % Find closest time point
        [~, idx] = min(abs(trajectory.time - wpt.time(i)));
        
        % Position should match (within interpolation tolerance)
        pos_error = norm(trajectory.position(idx, :) - wpt.position(i, :));
        assert(pos_error < 0.01, ...
               'Waypoint %d position error: %.3f m', i, pos_error);
        
        % Yaw should match (within interpolation tolerance)
        if ~isnan(wpt.yaw(i))
            yaw_error = abs(trajectory.attitude(idx, 3) - wpt.yaw(i));
            assert(yaw_error < 0.1, ...
                   'Waypoint %d yaw error: %.3f rad', i, yaw_error);
        end
    end
    
    fprintf('PASS\n');
end

%% Test 4: Continuity Check
function test_continuity()
    fprintf('Test 4: Continuity (smoothness)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 1 0 1; 2 1 1];
    wpt.yaw = [0; 0; 0];
    
    trajectory = generate_trajectory_interp(wpt, params, 0.01);
    
    % Check velocity is continuous (no jumps)
    vel_diff = diff(trajectory.velocity);
    max_vel_jump = max(abs(vel_diff(:)));
    
    % With dt=0.01, velocity changes should be small
    assert(max_vel_jump < 1.0, ...
           'Velocity has large discontinuity: %.3f m/s', max_vel_jump);
    
    % Check acceleration doesn't have extreme values
    max_accel = max(abs(trajectory.acceleration(:)));
    assert(max_accel < 50, ...
           'Acceleration too high: %.2f m/s²', max_accel);
    
    fprintf('PASS\n');
end

%% Test 5: Auto Yaw Calculation
function test_yaw_auto_calculation()
    fprintf('Test 5: Auto yaw calculation... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Trajectory moving in +X direction
    wpt.time = [0; 2]';
    wpt.position = [0 0 0; 2 0 0];
    wpt.yaw = [NaN; NaN];  % Auto-calculate both
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    % Find section where moving
    moving = sqrt(sum(trajectory.velocity.^2, 2)) > 0.1;
    
    if any(moving)
        % Yaw should point in direction of motion (approximately 0 for +X)
        avg_yaw = mean(trajectory.attitude(moving, 3));
        assert(abs(avg_yaw) < 0.2, ...
               'Auto yaw should point forward, got %.2f rad', avg_yaw);
    end
    
    fprintf('PASS\n');
end

%% Test 6: Explicit Yaw
function test_yaw_explicit()
    fprintf('Test 6: Explicit yaw handling... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Fixed yaw despite motion
    wpt.time = [0; 2]';
    wpt.position = [0 0 0; 1 1 0];
    wpt.yaw = [0; 0];  % Keep yaw at 0
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    % Yaw should remain near 0 throughout
    max_yaw = max(abs(trajectory.attitude(:, 3)));
    assert(max_yaw < 0.2, ...
           'Explicit yaw not maintained, max deviation: %.2f rad', max_yaw);
    
    fprintf('PASS\n');
end

%% Test 7: Matrix Input Format
function test_matrix_input()
    fprintf('Test 7: Matrix input format... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Matrix format: [time, x, y, z, yaw]
    waypoints = [
        0,  0,  0,  0,  0;
        2,  1,  0,  1,  NaN;
        4,  2,  0,  1,  0
    ];
    
    trajectory = generate_trajectory_interp(waypoints, params, 0.1);
    
    % Should work identically to structure input
    assert(isstruct(trajectory), 'Should return structure');
    assert(isfield(trajectory, 'position'), 'Should have position field');
    
    % Check waypoint satisfaction
    pos_start = trajectory.position(1, :);
    assert(norm(pos_start - [0 0 0]) < 0.01, 'Start position incorrect');
    
    fprintf('PASS\n');
end

%% Test 8: Time Sampling
function test_time_sampling()
    fprintf('Test 8: Time sampling... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    wpt.time = [0; 1]';
    wpt.position = [0 0 0; 1 0 0];
    wpt.yaw = [0; 0];
    
    % Test different time steps
    dt_values = [0.01, 0.05, 0.1];
    
    for dt = dt_values
        trajectory = generate_trajectory_interp(wpt, params, dt);
        
        % Check time step is approximately dt
        actual_dt = mean(diff(trajectory.time));
        assert(abs(actual_dt - dt) < 0.001, ...
               'Time step incorrect: expected %.3f, got %.3f', dt, actual_dt);
        
        % Duration should be correct
        duration = trajectory.time(end) - trajectory.time(1);
        assert(abs(duration - 1.0) < 0.01, 'Duration incorrect');
    end
    
    fprintf('PASS\n');
end

%% Test 9: First Yaw NaN (defaults to 0)
function test_first_yaw_nan()
    fprintf('Test 9: First yaw NaN handling... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % First yaw is NaN, should default to 0
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 1 0 0; 2 0 0];
    wpt.yaw = [NaN; pi/4; pi/2];
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    % First yaw should be 0 (defaulted from NaN)
    assert(abs(trajectory.yaw(1)) < 0.01, 'First yaw should default to 0');
    
    % Should interpolate to subsequent values
    assert(all(isfinite(trajectory.yaw)), 'All yaw values should be finite');
    
    fprintf('PASS\n');
end

%% Test 10: All NaN except first (hold constant)
function test_single_defined_yaw()
    fprintf('Test 10: Single defined yaw (hold constant)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Only first has value (after NaN→0 conversion), rest NaN
    wpt.time = [0; 2]';
    wpt.position = [0 0 0; 2 0 0];
    wpt.yaw = [0; NaN];  % After conversion: [0; NaN] → only one defined
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    % Should hold yaw=0 throughout
    assert(all(abs(trajectory.yaw) < 0.01), 'Should hold constant yaw=0');
    
    fprintf('PASS\n');
end

%% Test 11: Mixed NaN pattern
function test_mixed_nan_pattern()
    fprintf('Test 11: Mixed NaN pattern... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Pattern: defined, NaN, defined, NaN, defined
    wpt.time = [0; 1; 2; 3; 4]';
    wpt.position = [0 0 0; 1 0 0; 2 0 0; 3 0 0; 4 0 0];
    wpt.yaw = [0; NaN; pi/2; NaN; pi];
    
    trajectory = generate_trajectory_interp(wpt, params, 0.1);
    
    % Should interpolate between defined values
    assert(all(isfinite(trajectory.yaw)), 'All yaw should be finite');
    
    % Check interpolation at waypoints
    [~, idx1] = min(abs(trajectory.time - 0));
    [~, idx3] = min(abs(trajectory.time - 2));
    [~, idx5] = min(abs(trajectory.time - 4));
    
    assert(abs(trajectory.yaw(idx1) - 0) < 0.1, 'Waypoint 1 yaw incorrect');
    assert(abs(trajectory.yaw(idx3) - pi/2) < 0.1, 'Waypoint 3 yaw incorrect');
    assert(abs(trajectory.yaw(idx5) - pi) < 0.1, 'Waypoint 5 yaw incorrect');
    
    fprintf('PASS\n');
end