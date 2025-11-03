function test_trajectory_feasibility()
% TEST_TRAJECTORY_FEASIBILITY - Unit tests for trajectory feasibility checking
%
% Streamlined test suite covering essential integration points:
%   - Feasibility checking called during simulation
%   - Results structure contains feasibility data
%   - Warnings generated for infeasible trajectories
%   - Yaw rate/acceleration limits detected
%   - Report output includes feasibility section
%
% Author: Trey Copeland
% Date: 2025-11-02

    fprintf('Running Unit Tests: Trajectory Feasibility Checking\n');
    fprintf('========================================\n\n');
    
    try
        % Run all test cases
        test_feasibility_checked_for_good_trajectory();
        test_feasibility_detects_bad_auto_yaw();
        test_violations_structure_complete();
        test_yaw_motion_edge_cases();
        test_report_contains_feasibility_section();
        test_programmatic_trajectory_checked();
        test_fixed_yaw_makes_figure_eight_feasible();
        test_yaw_rate_limit_calculation_correct();
        test_rms_vs_max_yaw_rate_relationship();
        test_critical_vs_warning_thresholds();
        
        fprintf('\n========================================\n');
        fprintf('All Tests Passed! ✓\n\n');
        
    catch ME
        rethrow(ME);
    end
    
end

%% Test 1: Feasibility checked for good trajectory
function test_feasibility_checked_for_good_trajectory()
    fprintf('Test 1: Feasibility checked for good trajectory... ');
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    assert(isfield(results.trajectory, 'feasibility'), ...
        'Trajectory should have feasibility field');
    assert(results.trajectory.feasibility.feasible, ...
        'simple_square.wpt should be feasible');
    assert(isempty(results.trajectory.feasibility.warnings), ...
        'Feasible trajectory should have no warnings');
    
    fprintf('PASS\n');
end

%% Test 2: Feasibility detects bad auto yaw
function test_feasibility_detects_bad_auto_yaw()
    fprintf('Test 2: Feasibility detects bad auto yaw... ');
    
    wpt = load_waypoints('figure_eight_long.wpt');
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory(wpt, [], [], [], opts);
    
    assert(~results.trajectory.feasibility.feasible, ...
        'figure_eight_long with auto yaw should be infeasible');
    assert(~isempty(results.trajectory.feasibility.warnings), ...
        'Infeasible trajectory should have warnings');
    
    % Check for yaw warning
    has_yaw_warning = false;
    for i = 1:length(results.trajectory.feasibility.warnings)
        if contains(results.trajectory.feasibility.warnings{i}, 'yaw', 'IgnoreCase', true)
            has_yaw_warning = true;
            break;
        end
    end
    assert(has_yaw_warning, 'Should have warning about yaw rate/acceleration');
    
    fprintf('PASS\n');
end

%% Test 3: Violations structure complete
function test_violations_structure_complete()
    fprintf('Test 3: Violations structure complete... ');
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    v = results.trajectory.feasibility.violations;
    
    % Check main violation fields
    assert(isfield(v, 'max_attitude'), 'Missing max_attitude');
    assert(isfield(v, 'max_velocity'), 'Missing max_velocity');
    assert(isfield(v, 'max_horiz_accel'), 'Missing max_horiz_accel');
    assert(isfield(v, 'max_yaw_rate'), 'Missing max_yaw_rate');
    assert(isfield(v, 'rms_yaw_rate'), 'Missing rms_yaw_rate');
    assert(isfield(v, 'max_yaw_accel'), 'Missing max_yaw_accel');
    assert(isfield(v, 'limits'), 'Missing limits structure');
    
    % Check limits substructure
    assert(isfield(v.limits, 'yaw_rate_warning'), 'Missing yaw_rate_warning limit');
    assert(isfield(v.limits, 'yaw_accel_physical'), 'Missing yaw_accel_physical limit');
    
    % Verify values are reasonable
    assert(v.max_yaw_rate >= 0, 'max_yaw_rate should be non-negative');
    assert(v.rms_yaw_rate >= 0, 'rms_yaw_rate should be non-negative');
    assert(v.limits.yaw_rate_warning > 0, 'yaw_rate_warning should be positive');
    assert(v.limits.yaw_accel_physical > 0, 'yaw_accel_physical should be positive');
    
    fprintf('PASS\n');
end

%% Test 4: Yaw motion edge cases
function test_yaw_motion_edge_cases()
    fprintf('Test 4: Yaw motion edge cases (zero/constant/slow)... ');
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    
    % Test 4a: Zero yaw motion
    wpt_zero.time = [0; 10; 20]';
    wpt_zero.position = [0 0 1; 0 0 1; 0 0 1];
    wpt_zero.yaw = [0; 0; 0]';
    
    results_zero = simulate_trajectory(wpt_zero, [], [], [], opts);
    
    assert(results_zero.trajectory.feasibility.feasible, ...
        'Zero yaw motion should be feasible');
    assert(results_zero.trajectory.feasibility.violations.max_yaw_rate < 1.0, ...
        'Zero yaw: max rate should be near zero');
    
    % Test 4b: Constant nonzero yaw
    wpt_const.time = [0; 5; 10]';
    wpt_const.position = [0 0 1; 2 0 1; 2 2 1];
    wpt_const.yaw = [pi/4; pi/4; pi/4]';
    
    results_const = simulate_trajectory(wpt_const, [], [], [], opts);
    
    assert(results_const.trajectory.feasibility.feasible, ...
        'Constant yaw should be feasible');
    assert(results_const.trajectory.feasibility.violations.max_yaw_rate < 5.0, ...
        'Constant yaw: max rate should be minimal');
    
    % Test 4c: Slow yaw change
    wpt_slow.time = [0; 10; 20]';
    wpt_slow.position = [0 0 1; 0 0 1; 0 0 1];
    wpt_slow.yaw = [0; pi/4; pi/2]';
    
    results_slow = simulate_trajectory(wpt_slow, [], [], [], opts);
    
    assert(results_slow.trajectory.feasibility.feasible, ...
        'Slow yaw change should be feasible');
    assert(results_slow.trajectory.feasibility.violations.max_yaw_rate < 50, ...
        'Slow yaw: should be under warning threshold');
    
    fprintf('PASS\n');
end

%% Test 5: Report contains feasibility section
function test_report_contains_feasibility_section()
    fprintf('Test 5: Report contains feasibility section... ');
    
    opts = struct('verbose', false, 'plot', false, 'output_dir', './temp_test_files', ...
                  'save_results', true);
    results = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    % Check report exists and has correct content
    try
        report_path = fullfile(results.output_dir, 'analysis_report.txt');
        assert(isfile(report_path), 'analysis_report.txt should exist');
        
        report_text = fileread(report_path);
        assert(contains(report_text, 'TRAJECTORY FEASIBILITY'), ...
            'Report should contain TRAJECTORY FEASIBILITY section');
        assert(contains(report_text, 'Max Yaw Rate'), ...
            'Report should contain yaw rate metrics');
        assert(contains(report_text, 'Max Yaw Accel'), ...
            'Report should contain yaw acceleration metrics');
        
        fprintf('PASS\n');
    catch ME
        % Clean up before rethrowing
        if exist(results.output_dir, 'dir')
            rmdir(results.output_dir, 's');
        end
        rethrow(ME);
    end
    
    % Clean up after test
    if exist(results.output_dir, 'dir')
        rmdir(results.output_dir, 's');
    end
    
    % Clean up parent directory if empty
    if exist('./temp_test_files', 'dir')
        contents = dir('./temp_test_files');
        contents = contents(~ismember({contents.name}, {'.', '..'}));
        if isempty(contents)
            rmdir('./temp_test_files');
        end
    end
end

%% Test 6: Programmatic trajectory checked
function test_programmatic_trajectory_checked()
    fprintf('Test 6: Programmatic trajectory checked... ');
    
    wpt_prog.time = [0; 3; 6]';
    wpt_prog.position = [0 0 1; 1 0 1; 1 1 1];
    wpt_prog.yaw = [0; 0; 0]';
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory(wpt_prog, [], [], [], opts);
    
    assert(isfield(results.trajectory, 'feasibility'), ...
        'Programmatic trajectory should be checked');
    assert(results.trajectory.feasibility.feasible, ...
        'Simple programmatic trajectory should be feasible');
    
    fprintf('PASS\n');
end

%% Test 7: Fixed yaw makes figure_eight feasible
function test_fixed_yaw_makes_figure_eight_feasible()
    fprintf('Test 7: Fixed yaw makes figure_eight feasible... ');
    
    wpt = load_waypoints('figure_eight_long.wpt');
    wpt.yaw = zeros(size(wpt.yaw));  % Force constant yaw
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory(wpt, [], [], [], opts);
    
    assert(results.trajectory.feasibility.feasible, ...
        'figure_eight_long with constant yaw should be feasible');
    assert(results.trajectory.feasibility.violations.max_yaw_rate < 10, ...
        'Constant yaw should have minimal rate');
    
    fprintf('PASS\n');
end

%% Test 8: Yaw rate limit calculation correct
function test_yaw_rate_limit_calculation_correct()
    fprintf('Test 8: Yaw rate limit calculation correct... ');
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    results = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    params = quadrotor_linear_6dof();
    expected_limit = rad2deg(params.u_max(4) / params.Izz);
    actual_limit = results.trajectory.feasibility.violations.limits.yaw_accel_physical;
    
    rel_error = abs(actual_limit - expected_limit) / expected_limit;
    assert(rel_error < 0.01, ...
        'Physical yaw acceleration limit should match τ_max / I_zz');
    
    fprintf('PASS\n');
end

%% Test 9: RMS vs max yaw rate relationship
function test_rms_vs_max_yaw_rate_relationship()
    fprintf('Test 9: RMS vs max yaw rate relationship... ');
    
    test_files = {'simple_square.wpt', 'hover_test.wpt'};
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    
    for i = 1:length(test_files)
        results = simulate_trajectory(test_files{i}, [], [], [], opts);
        
        v = results.trajectory.feasibility.violations;
        assert(v.rms_yaw_rate <= v.max_yaw_rate, ...
            sprintf('RMS yaw rate should not exceed max for %s', test_files{i}));
    end
    
    fprintf('PASS\n');
end

%% Test 10: Critical vs warning thresholds
function test_critical_vs_warning_thresholds()
    fprintf('Test 10: Critical vs warning thresholds... ');
    
    opts = struct('verbose', false, 'plot', false, 'save_results', false);
    
    % Moderately bad
    wpt_warn.time = [0; 2; 4]';
    wpt_warn.position = [0 0 1; 0 0 1; 0 0 1];
    wpt_warn.yaw = [0; deg2rad(120); deg2rad(240)]';  % 60°/s avg
    
    results_warn = simulate_trajectory(wpt_warn, [], [], [], opts);
    
    assert(~results_warn.trajectory.feasibility.feasible, ...
        'Warning-level trajectory should be infeasible');
    
    % Critically bad
    wpt_crit.time = [0; 0.5; 1]';
    wpt_crit.position = [0 0 1; 0 0 1; 0 0 1];
    wpt_crit.yaw = [0; 2*pi; 4*pi]';  % 360°/s
    
    results_crit = simulate_trajectory(wpt_crit, [], [], [], opts);
    
    assert(results_crit.trajectory.feasibility.violations.max_yaw_rate > ...
           results_warn.trajectory.feasibility.violations.max_yaw_rate, ...
           'Critical case should have higher yaw rate than warning case');
    
    fprintf('PASS\n');
end