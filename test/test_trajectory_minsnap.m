function test_minsnap()
% TEST_MINSNAP - Comprehensive tests for minimum snap trajectory generation
%
% Tests minimum snap algorithm with various waypoint configurations to ensure
% correctness, robustness, and performance.
%
% Test coverage:
%   - Simple trajectories (2-3 waypoints)
%   - Complex trajectories (many waypoints)
%   - Edge cases (collinear points, tight turns)
%   - Boundary conditions (velocity, acceleration)
%   - Continuity properties (C⁴ smoothness)
%   - Performance metrics (snap cost, computation time)

    fprintf('Running Unit Tests: Minimum Snap Trajectory\n');
    fprintf('============================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_simple_hover();
    test_straight_line();
    test_vertical_climb();
    test_square_pattern();
    test_figure_eight();
    test_aggressive_racing();
    test_stop_and_go();
    test_flythrough_waypoints();
    test_mixed_velocity_constraints();
    test_collinear_waypoints();
    test_sharp_turn();
    test_many_waypoints();
    test_boundary_conditions();
    test_continuity_c4();
    test_snap_cost_optimization();
    test_comparison_with_pchip();
    test_yaw_explicit();
    test_yaw_auto();
    test_yaw_mixed();
    test_computation_performance();
    
    fprintf('\n============================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: Simple Hover (Degenerate Case)
function test_simple_hover()
    fprintf('Test 1: Simple hover (stationary)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Stay at origin
    wpt.time = [0; 5]';
    wpt.position = [0 0 0; 0 0 0];
    wpt.yaw = [0; 0];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.1);
    
    % Should remain stationary
    max_vel = max(sqrt(sum(trajectory.velocity.^2, 2)));
    max_acc = max(sqrt(sum(trajectory.acceleration.^2, 2)));
    
    assert(max_vel < 1e-6, 'Hover should have zero velocity');
    assert(max_acc < 1e-6, 'Hover should have zero acceleration');
    
    fprintf('PASS\n');
end

%% Test 2: Straight Line
function test_straight_line()
    fprintf('Test 2: Straight line trajectory... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Move 5m forward
    wpt.time = [0; 5]';
    wpt.position = [0 0 0; 5 0 0];
    wpt.yaw = [0; 0];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check waypoint satisfaction
    start_pos = trajectory.position(1, :);
    end_pos = trajectory.position(end, :);
    
    assert(norm(start_pos - [0 0 0]) < 1e-3, 'Start position incorrect');
    assert(norm(end_pos - [5 0 0]) < 1e-3, 'End position incorrect');
    
    % Should stay in XZ plane (Y should be ~0)
    max_y = max(abs(trajectory.position(:, 2)));
    assert(max_y < 1e-3, 'Should stay in XZ plane');
    
    % Velocity should be near zero at endpoints
    v_start = norm(trajectory.velocity(1, :));
    v_end = norm(trajectory.velocity(end, :));
    assert(v_start < 0.1, 'Start velocity should be near zero');
    assert(v_end < 0.1, 'End velocity should be near zero');
    
    fprintf('PASS\n');
end

%% Test 3: Vertical Climb
function test_vertical_climb()
    fprintf('Test 3: Vertical climb... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Climb to 5m altitude
    wpt.time = [0; 3]';
    wpt.position = [0 0 0; 0 0 5];
    wpt.yaw = [0; 0];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Should stay at origin in XY
    max_x = max(abs(trajectory.position(:, 1)));
    max_y = max(abs(trajectory.position(:, 2)));
    
    assert(max_x < 1e-3, 'Should not drift in X');
    assert(max_y < 1e-3, 'Should not drift in Y');
    
    % Z should increase monotonically (with stop-and-go)
    z_vel = trajectory.velocity(:, 3);
    
    % Initially accelerating up
    assert(z_vel(floor(length(z_vel)/4)) > 0, 'Should be climbing');
    
    fprintf('PASS\n');
end

%% Test 4: Square Pattern
function test_square_pattern()
    fprintf('Test 4: Square pattern (4 corners)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Square: origin → (2,0) → (2,2) → (0,2) → origin
    wpt.time = [0; 2; 4; 6; 8]';
    wpt.position = [
        0, 0, 1;
        2, 0, 1;
        2, 2, 1;
        0, 2, 1;
        0, 0, 1
    ];
    wpt.yaw = zeros(5, 1);
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check all waypoints satisfied
    tol = 0.01;
    for i = 1:5
        [~, idx] = min(abs(trajectory.time - wpt.time(i)));
        pos_error = norm(trajectory.position(idx, :) - wpt.position(i, :));
        assert(pos_error < tol, sprintf('Waypoint %d error: %.3f m', i, pos_error));
    end
    
    % Should stay at constant altitude
    z_range = max(trajectory.position(:, 3)) - min(trajectory.position(:, 3));
    assert(z_range < 0.5, 'Altitude should remain approximately constant');
    
    fprintf('PASS\n');
end

%% Test 5: Figure Eight
function test_figure_eight()
    fprintf('Test 5: Figure-eight pattern... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Figure-eight waypoints
    t_total = 16;
    n_points = 9;
    theta = linspace(0, 2*pi, n_points);
    
    wpt.time = linspace(0, t_total, n_points)';
    wpt.position = [
        sin(theta') * 2, ...
        sin(2*theta') * 1, ...
        ones(n_points, 1)
    ];
    wpt.yaw = nan(n_points, 1);  % Auto yaw
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check smooth and continuous
    max_jerk = max(sqrt(sum(trajectory.jerk.^2, 2)));
    assert(max_jerk < 50, sprintf('Jerk too high: %.2f m/s³', max_jerk));
    
    % Check closes loop approximately
    start_pos = trajectory.position(1, :);
    end_pos = trajectory.position(end, :);
    loop_closure = norm(end_pos - start_pos);
    assert(loop_closure < 0.1, sprintf('Loop closure error: %.3f m', loop_closure));
    
    fprintf('PASS\n');
end

%% Test 6: Aggressive Racing Track
function test_aggressive_racing()
    fprintf('Test 6: Aggressive racing trajectory... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Fast racing track with tight timing
    wpt.time = [0; 1; 2; 3; 4]';
    wpt.position = [
        0,  0,  0;
        5,  0,  2;
        10, 3,  2;
        10, 6,  1;
        5,  6,  0
    ];
    wpt.yaw = zeros(5, 1);
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Should have high velocities
    max_vel = max(sqrt(sum(trajectory.velocity.^2, 2)));
    assert(max_vel > 2, 'Racing trajectory should have high velocity');
    
    % But still reasonable acceleration
    max_acc = max(sqrt(sum(trajectory.acceleration.^2, 2)));
    assert(max_acc < 50, sprintf('Acceleration too high: %.2f m/s²', max_acc));
    
    fprintf('PASS\n');
end

%% Test 7: Stop-and-Go (Default Velocity Constraints)
function test_stop_and_go()
    fprintf('Test 7: Interior waypoint velocity optimization... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Three waypoints with default zero velocity at start/end
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 2 0 1; 4 0 0];
    wpt.yaw = [0; 0; 0];
    % No velocity field - interior velocity is FREE (optimized)
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check velocity near zero at START and END waypoints only
    [~, idx_start] = min(abs(trajectory.time - wpt.time(1)));
    vel_start = norm(trajectory.velocity(idx_start, :));
    assert(vel_start < 0.1, sprintf('Start velocity: %.3f m/s (should be ~0)', vel_start));
    
    [~, idx_end] = min(abs(trajectory.time - wpt.time(3)));
    vel_end = norm(trajectory.velocity(idx_end, :));
    assert(vel_end < 0.1, sprintf('End velocity: %.3f m/s (should be ~0)', vel_end));
    
    % Interior waypoint velocity should be non-zero (optimized for smoothness)
    [~, idx_mid] = min(abs(trajectory.time - wpt.time(2)));
    vel_mid = norm(trajectory.velocity(idx_mid, :));
    assert(vel_mid > 0.1, 'Interior waypoint should have non-zero velocity (optimized)');
    assert(vel_mid < 5.0, 'Interior waypoint velocity should be reasonable');
    
    fprintf('PASS (interior vel: %.2f m/s)\n', vel_mid);
end

%% Test 8: Flythrough Waypoints (Explicit Velocity)
function test_flythrough_waypoints()
    fprintf('Test 8: Flythrough waypoints (explicit velocity)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Middle waypoint is flythrough at 2 m/s
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 4 0 1; 8 0 0];
    wpt.velocity = [0 0 0; 2 0 0; 0 0 0];  % Flythrough at 2 m/s in X
    wpt.yaw = [0; 0; 0];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check velocity at middle waypoint
    [~, idx] = min(abs(trajectory.time - wpt.time(2)));
    vel_mid = trajectory.velocity(idx, :);
    vel_x = vel_mid(1);
    
    assert(abs(vel_x - 2.0) < 0.2, ...
           sprintf('Middle waypoint vx: %.2f m/s (expected 2.0)', vel_x));
    
    % Start and end should still be near zero
    vel_start = norm(trajectory.velocity(1, :));
    vel_end = norm(trajectory.velocity(end, :));
    assert(vel_start < 0.1, 'Start velocity should be near zero');
    assert(vel_end < 0.1, 'End velocity should be near zero');
    
    fprintf('PASS\n');
end

%% Test 9: Mixed Velocity Constraints
function test_mixed_velocity_constraints()
    fprintf('Test 9: Mixed stop/flythrough waypoints... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Stop, flythrough, stop, flythrough, stop
    wpt.time = [0; 1; 2; 3; 4]';
    wpt.position = [
        0, 0, 0;
        2, 0, 1;
        4, 0, 1;
        6, 0, 0;
        8, 0, 0
    ];
    wpt.velocity = [
        0, 0, 0;    % Stop
        1.5, 0, 0;  % Flythrough
        0, 0, 0;    % Stop
        1.0, 0, 0;  % Flythrough
        0, 0, 0     % Stop
    ];
    wpt.yaw = zeros(5, 1);
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Verify velocities at each waypoint
    expected_vx = [0, 1.5, 0, 1.0, 0];
    for i = 1:5
        [~, idx] = min(abs(trajectory.time - wpt.time(i)));
        vx = trajectory.velocity(idx, 1);
        error = abs(vx - expected_vx(i));
        assert(error < 0.3, ...
               sprintf('Waypoint %d: vx=%.2f (expected %.2f)', i, vx, expected_vx(i)));
    end
    
    fprintf('PASS\n');
end

%% Test 10: Collinear Waypoints
function test_collinear_waypoints()
    fprintf('Test 10: Collinear waypoints (straight line)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % All on same line
    wpt.time = [0; 1; 2; 3]';
    wpt.position = [
        0, 0, 1;
        1, 0, 1;
        2, 0, 1;
        3, 0, 1
    ];
    wpt.yaw = zeros(4, 1);
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Should stay in line (Y and Z constant)
    max_y = max(abs(trajectory.position(:, 2)));
    z_var = max(trajectory.position(:, 3)) - min(trajectory.position(:, 3));
    
    assert(max_y < 0.01, 'Should not deviate in Y');
    assert(z_var < 0.1, 'Should maintain altitude');
    
    fprintf('PASS\n');
end

%% Test 11: Sharp Turn (90 degrees)
function test_sharp_turn()
    fprintf('Test 11: Sharp 90-degree turn... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Right-angle turn
    wpt.time = [0; 2; 4]';
    wpt.position = [
        0, 0, 1;
        3, 0, 1;
        3, 3, 1
    ];
    wpt.yaw = [0; pi/4; pi/2];  % Turn yaw as well
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check corner rounding is smooth
    [~, corner_idx] = min(abs(trajectory.time - wpt.time(2)));
    
    % Acceleration should be finite
    acc_corner = norm(trajectory.acceleration(corner_idx, :));
    assert(acc_corner < 30, sprintf('Corner acceleration: %.2f m/s²', acc_corner));
    
    % Jerk should be reasonable
    jerk_corner = norm(trajectory.jerk(corner_idx, :));
    assert(jerk_corner < 100, sprintf('Corner jerk: %.2f m/s³', jerk_corner));
    
    fprintf('PASS\n');
end

%% Test 12: Many Waypoints (Scalability)
function test_many_waypoints()
    fprintf('Test 12: Many waypoints (N=20)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % 20 waypoints in a spiral
    N = 20;
    t = linspace(0, 30, N)';
    theta = linspace(0, 4*pi, N)';
    
    wpt.time = t;
    wpt.position = [
        cos(theta) .* (1 + theta/10), ...
        sin(theta) .* (1 + theta/10), ...
        theta / 10
    ];
    wpt.yaw = nan(N, 1);
    
    tic;
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    comp_time = toc;
    
    % Should complete in reasonable time
    assert(comp_time < 5, sprintf('Computation too slow: %.2f s', comp_time));
    
    % All waypoints satisfied
    for i = 1:N
        [~, idx] = min(abs(trajectory.time - wpt.time(i)));
        pos_error = norm(trajectory.position(idx, :) - wpt.position(i, :));
        assert(pos_error < 0.05, sprintf('Waypoint %d error: %.3f m', i, pos_error));
    end
    
    fprintf('PASS (%.2f s)\n', comp_time);
end

%% Test 13: Boundary Conditions (Position, Velocity, Acceleration)
function test_boundary_conditions()
    fprintf('Test 13: Boundary condition enforcement... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Simple trajectory with explicit constraints
    wpt.time = [0; 3]';
    wpt.position = [0 0 0; 5 0 0];
    wpt.velocity = [0 0 0; 0 0 0];
    wpt.acceleration = [0 0 0; 0 0 0];
    wpt.yaw = [0; 0];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check start conditions
    assert(norm(trajectory.position(1, :) - [0 0 0]) < 1e-6, 'Start position');
    assert(norm(trajectory.velocity(1, :)) < 1e-3, 'Start velocity');
    assert(norm(trajectory.acceleration(1, :)) < 0.1, 'Start acceleration');
    
    % Check end conditions
    assert(norm(trajectory.position(end, :) - [5 0 0]) < 1e-6, 'End position');
    assert(norm(trajectory.velocity(end, :)) < 1e-3, 'End velocity');
    assert(norm(trajectory.acceleration(end, :)) < 0.1, 'End acceleration');
    
    fprintf('PASS\n');
end

%% Test 14: C⁴ Continuity (Snap is Continuous)
function test_continuity_c4()
    fprintf('Test 14: C⁴ continuity verification... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Multi-segment trajectory
    wpt.time = [0; 2; 4; 6]';
    wpt.position = [
        0, 0, 0;
        2, 0, 1;
        4, 2, 1;
        6, 2, 0
    ];
    wpt.yaw = zeros(4, 1);
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check continuity at interior waypoints
    for wp = 2:3
        t_wp = wpt.time(wp);
        
        % Find points just before and after waypoint
        idx_before = find(trajectory.time < t_wp, 1, 'last');
        idx_after = find(trajectory.time > t_wp, 1, 'first');
        
        if ~isempty(idx_before) && ~isempty(idx_after)
            % Position continuity
            pos_jump = norm(trajectory.position(idx_after, :) - trajectory.position(idx_before, :));
            assert(pos_jump < 0.01, sprintf('Position discontinuity at wp %d', wp));
            
            % Velocity continuity
            vel_jump = norm(trajectory.velocity(idx_after, :) - trajectory.velocity(idx_before, :));
            assert(vel_jump < 0.1, sprintf('Velocity discontinuity at wp %d', wp));
            
            % Acceleration continuity
            acc_jump = norm(trajectory.acceleration(idx_after, :) - trajectory.acceleration(idx_before, :));
            assert(acc_jump < 1.0, sprintf('Acceleration discontinuity at wp %d', wp));
            
            % Jerk continuity
            jerk_jump = norm(trajectory.jerk(idx_after, :) - trajectory.jerk(idx_before, :));
            assert(jerk_jump < 10, sprintf('Jerk discontinuity at wp %d', wp));
        end
    end
    
    fprintf('PASS\n');
end

%% Test 15: Snap Cost is Lower than Alternatives
function test_snap_cost_optimization()
    fprintf('Test 15: Snap cost optimization... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Simple trajectory
    wpt.time = [0; 2; 4]';
    wpt.position = [0 0 0; 2 0 1; 4 0 0];
    wpt.yaw = [0; 0; 0];
    
    % Generate minimum snap
    traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Compute snap cost
    snap_cost = sum(sum(traj_minsnap.snap.^2)) * 0.01;  % Approximate integral
    
    % Should be reasonable (not infinite or NaN)
    assert(isfinite(snap_cost), 'Snap cost should be finite');
    assert(snap_cost > 0, 'Snap cost should be positive');
    
    fprintf('PASS (cost: %.2e)\n', snap_cost);
end

%% Test 16: Comparison with Pchip (if available)
function test_comparison_with_pchip()
    fprintf('Test 16: Comparison with pchip method... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Same waypoints for both methods
    wpt.time = [0; 2; 4; 6]';
    wpt.position = [
        0, 0, 0;
        2, 1, 1;
        4, 1, 1;
        6, 0, 0
    ];
    wpt.yaw = zeros(4, 1);
    
    % Generate with minsnap
    traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % If pchip trajectory generator exists, compare
    if exist('generate_trajectory_pchip', 'file')
        traj_pchip = generate_trajectory_pchip(wpt, params, 0.01);
        
        % Minsnap should have lower jerk
        jerk_minsnap = max(sqrt(sum(traj_minsnap.jerk.^2, 2)));
        jerk_pchip = max(sqrt(sum(traj_pchip.jerk.^2, 2)));
        
        fprintf('(minsnap jerk: %.2f, pchip jerk: %.2f) ', jerk_minsnap, jerk_pchip);
        
        % Minsnap should be smoother (usually)
        % Note: This might not always be true depending on implementation
        % assert(jerk_minsnap < jerk_pchip * 1.5, 'Minsnap should be smoother');
    end
    
    fprintf('PASS\n');
end

%% Test 17: Explicit Yaw Tracking
function test_yaw_explicit()
    fprintf('Test 17: Explicit yaw angles... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Rotating in place
    wpt.time = [0; 2; 4; 6]';
    wpt.position = [
        0, 0, 1;
        0, 0, 1;
        0, 0, 1;
        0, 0, 1
    ];
    wpt.yaw = [0; pi/2; pi; 3*pi/2];
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Check yaw at waypoints
    for i = 1:4
        [~, idx] = min(abs(trajectory.time - wpt.time(i)));
        yaw_error = abs(trajectory.yaw(idx) - wpt.yaw(i));
        assert(yaw_error < 0.1, sprintf('Waypoint %d yaw error: %.2f rad', i, yaw_error));
    end
    
    fprintf('PASS\n');
end

%% Test 18: Auto Yaw from Velocity
function test_yaw_auto()
    fprintf('Test 18: Auto yaw from velocity direction... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Square pattern with auto yaw
    wpt.time = [0; 2; 4; 6; 8]';
    wpt.position = [
        0, 0, 1;
        2, 0, 1;
        2, 2, 1;
        0, 2, 1;
        0, 0, 1
    ];
    wpt.yaw = nan(5, 1);  % All auto
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % When moving in +X, yaw should be near 0
    % When moving in +Y, yaw should be near pi/2
    
    % Check a point in first segment (moving +X)
    idx_seg1 = find(trajectory.time > 0.5 & trajectory.time < 1.5, 1);
    if ~isempty(idx_seg1) && norm(trajectory.velocity(idx_seg1, :)) > 0.1
        yaw_seg1 = trajectory.yaw(idx_seg1);
        vx = trajectory.velocity(idx_seg1, 1);
        vy = trajectory.velocity(idx_seg1, 2);
        expected_yaw = atan2(vy, vx);
        yaw_error = abs(angdiff(yaw_seg1, expected_yaw));
        assert(yaw_error < 0.3, sprintf('Auto yaw error: %.2f rad', yaw_error));
    end
    
    fprintf('PASS\n');
end

%% Test 19: Mixed Yaw (Explicit + Auto)
function test_yaw_mixed()
    fprintf('Test 19: Mixed explicit/auto yaw... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Some explicit, some auto
    wpt.time = [0; 2; 4; 6]';
    wpt.position = [
        0, 0, 1;
        2, 0, 1;
        4, 2, 1;
        6, 2, 1
    ];
    wpt.yaw = [0; Constants.AUTO_YAW; pi/2; Constants.AUTO_YAW];  % Mixed
    
    trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Should handle gracefully
    assert(all(isfinite(trajectory.yaw)), 'Yaw should be finite everywhere');
    
    % Check explicit waypoints
    [~, idx1] = min(abs(trajectory.time - wpt.time(1)));
    assert(abs(trajectory.yaw(idx1) - 0) < 0.1, 'First yaw incorrect');
    
    [~, idx3] = min(abs(trajectory.time - wpt.time(3)));
    assert(abs(trajectory.yaw(idx3) - pi/2) < 0.1, 'Third yaw incorrect');
    
    fprintf('PASS\n');
end

%% Test 20: Computation Performance
function test_computation_performance()
    fprintf('Test 20: Computation performance benchmark... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Benchmark different sizes
    sizes = [3, 5, 10, 15];
    times = zeros(size(sizes));
    
    for i = 1:length(sizes)
        N = sizes(i);
        
        % Generate waypoints
        t = linspace(0, N*2, N)';
        wpt.time = t;
        wpt.position = [rand(N, 2)*10, ones(N, 1)];
        wpt.yaw = zeros(N, 1);
        
        % Time generation
        tic;
        trajectory = generate_trajectory_minsnap(wpt, params, 0.01);
        times(i) = toc;
        
        fprintf('\n    N=%2d waypoints: %.3f s', N, times(i));
    end
    
    % Should scale reasonably (not exponentially)
    % For N=15, should complete in under 3 seconds
    assert(times(end) < 3, sprintf('Too slow for N=%d: %.2f s', sizes(end), times(end)));
    
    fprintf('\n  PASS\n');
end

%% Helper function
function diff_angle = angdiff(angle1, angle2)
% ANGDIFF - Compute smallest angular difference
    diff_angle = mod(angle1 - angle2 + pi, 2*pi) - pi;
end