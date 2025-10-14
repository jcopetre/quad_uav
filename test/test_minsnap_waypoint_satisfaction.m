function test_minsnap_waypoint_satisfaction()
% TEST_MINSNAP_WAYPOINT_SATISFACTION - Verify waypoints are hit exactly
%
% This test exposed the bug where interior waypoints were not properly
% constrained, causing the trajectory to "miss" waypoints while maintaining
% continuity, leading to high accelerations.
%
% The bug was: interior waypoints only constrained the END of the arriving
% segment, not the START of the departing segment. This allowed the
% departing segment to start at a different position while maintaining
% continuity (since continuity only ensures derivatives match, not that
% they match the waypoint).

fprintf('Running Test: MinSnap Waypoint Satisfaction\n');
fprintf('=============================================\n\n');

setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

%% Test 1: Two waypoints (simple case)
fprintf('Test 1: Two waypoints (baseline)... ');
test_two_waypoints(params);
fprintf('PASS\n');

%% Test 2: Three waypoints (exposes the bug)
fprintf('Test 2: Three waypoints (interior point)... ');
test_three_waypoints(params);
fprintf('PASS\n');

%% Test 3: Seven waypoints (realistic trajectory)
fprintf('Test 3: Seven waypoints (realistic)... ');
test_seven_waypoints(params);
fprintf('PASS\n');

%% Test 4: Straight line with many waypoints
fprintf('Test 4: Straight line (5 waypoints)... ');
test_straight_line(params);
fprintf('PASS\n');

%% Test 5: L-shape trajectory
fprintf('Test 5: L-shape trajectory... ');
test_l_shape(params);
fprintf('PASS\n');

fprintf('\n=============================================\n');
fprintf('All Waypoint Satisfaction Tests Passed! ✓\n\n');

end

%% Test Case 1: Two Waypoints
function test_two_waypoints(params)
    wpt.time = [0; 2]';
    wpt.position = [0 0 1; 2 0 1];
    wpt.velocity = [0 0 0; 0 0 0];
    wpt.acceleration = [0 0 0; 0 0 0];
    wpt.yaw = [0; 0];
    
    traj = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check both waypoints
    for i = 1:2
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_error = norm(traj.position(idx, :) - wpt.position(i, :));
        
        assert(pos_error < 1e-3, ...
               sprintf('Waypoint %d error: %.2e m (should be < 1e-3)', i, pos_error));
    end
end

%% Test Case 2: Three Waypoints (Critical Test)
function test_three_waypoints(params)
    % This test FAILS with the buggy version because waypoint 2
    % is only constrained on segment 1's end, not segment 2's start
    
    wpt.time = [0; 2; 4]';
    wpt.position = [
        0, 0, 1;    % Start
        2, 0, 1;    % Middle (CRITICAL: this is where bug appears)
        4, 0, 1     % End
    ];
    wpt.velocity = [0 0 0; 0 0 0; 0 0 0];
    wpt.acceleration = [0 0 0; 0 0 0; 0 0 0];
    wpt.yaw = [0; 0; 0];
    
    traj = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check all three waypoints
    max_error = 0;
    for i = 1:3
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_error = norm(traj.position(idx, :) - wpt.position(i, :));
        max_error = max(max_error, pos_error);
        
        if pos_error >= 1e-3
            fprintf('\n  ⚠ Waypoint %d error: %.2e m\n', i, pos_error);
            fprintf('    Target: [%.3f, %.3f, %.3f]\n', wpt.position(i, :));
            fprintf('    Actual: [%.3f, %.3f, %.3f]\n', traj.position(idx, :));
        end
        
        assert(pos_error < 1e-3, ...
               sprintf('Waypoint %d error: %.2e m (INTERIOR WAYPOINT BUG!)', i, pos_error));
    end
end

%% Test Case 3: Seven Waypoints (Realistic Trajectory)
function test_seven_waypoints(params)
    % Load actual trajectory file
    wpt = load_waypoints('basic_maneuver.wpt');
    
    traj = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check all waypoints
    max_error = 0;
    for i = 1:length(wpt.time)
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_error = norm(traj.position(idx, :) - wpt.position(i, :));
        max_error = max(max_error, pos_error);
        
        assert(pos_error < 1e-3, ...
               sprintf('Waypoint %d error: %.2e m', i, pos_error));
    end
end

%% Test Case 4: Straight Line (Multiple Waypoints)
function test_straight_line(params)
    % Five collinear waypoints - tests if trajectory takes shortcuts
    wpt.time = [0; 1; 2; 3; 4]';
    wpt.position = [
        0, 0, 1;
        1, 0, 1;
        2, 0, 1;
        3, 0, 1;
        4, 0, 1
    ];
    wpt.yaw = zeros(5, 1);
    
    traj = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check all waypoints
    for i = 1:5
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_error = norm(traj.position(idx, :) - wpt.position(i, :));
        
        assert(pos_error < 1e-3, ...
               sprintf('Waypoint %d error: %.2e m', i, pos_error));
    end
    
    % Additional check: trajectory should stay in line (Y, Z constant)
    max_y_deviation = max(abs(traj.position(:, 2)));
    max_z_deviation = max(abs(traj.position(:, 3) - 1));
    
    assert(max_y_deviation < 0.01, 'Trajectory deviates from line in Y');
    assert(max_z_deviation < 0.01, 'Trajectory deviates from line in Z');
end

%% Test Case 5: L-Shape
function test_l_shape(params)
    % Right-angle turn - classic test case
    wpt.time = [0; 2; 4]';
    wpt.position = [
        0, 0, 1;
        3, 0, 1;
        3, 3, 1
    ];
    wpt.yaw = [0; 0; pi/2];
    
    traj = generate_trajectory_minsnap(wpt, params, 0.001);
    
    % Check corner waypoint especially carefully
    [~, idx_corner] = min(abs(traj.time - wpt.time(2)));
    corner_error = norm(traj.position(idx_corner, :) - wpt.position(2, :));
    
    if corner_error >= 1e-3
        fprintf('\n  ⚠ Corner waypoint error: %.2e m\n', corner_error);
        fprintf('    This is where the bug typically shows up!\n');
    end
    
    assert(corner_error < 1e-3, ...
           sprintf('Corner waypoint error: %.2e m (BUG!)', corner_error));
    
    % Check all waypoints
    for i = 1:3
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_error = norm(traj.position(idx, :) - wpt.position(i, :));
        
        assert(pos_error < 1e-3, ...
               sprintf('Waypoint %d error: %.2e m', i, pos_error));
    end
end