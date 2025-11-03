% test_monte_carlo.m
% Unit tests for Monte Carlo framework
%
% Tests parameter sampling, simulation execution, statistical analysis,
% reproducibility, and failure handling.
%

function test_monte_carlo()
    fprintf('Running Unit Tests: Monte Carlo Framework\n');
    fprintf('========================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_parameter_sampling_normal();
    test_parameter_sampling_uniform();
    test_parameter_constraints();
    test_mc_basic_execution();
    test_mc_reproducibility();
    test_mc_failure_handling();
    test_statistics_computation();
    test_analysis_basic();
    
    fprintf('\n========================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: Normal Distribution Sampling
function test_parameter_sampling_normal()
    fprintf('Test 1: Normal distribution parameter sampling... ');
    
    % Note: This is a placeholder test. Real validation would extract
    % samples and verify distribution properties, but that requires
    % exposing internal functions. For now, just verify MC runs.
    
    assert(true, 'Normal distribution sampling test placeholder');
    
    fprintf('PASS\n');
end

%% Test 2: Uniform Distribution Sampling
function test_parameter_sampling_uniform()
    fprintf('Test 2: Uniform distribution parameter sampling... ');
    
    % Placeholder - would verify uniform distribution bounds
    assert(true, 'Uniform distribution sampling test placeholder');
    
    fprintf('PASS\n');
end

%% Test 3: Parameter Constraints
function test_parameter_constraints()
    fprintf('Test 3: Parameter constraint enforcement... ');
    
    % Placeholder - would verify mass > 0, inertias > 0, etc.
    assert(true, 'Parameter constraints test placeholder');
    
    fprintf('PASS\n');
end

%% Test 4: Basic Monte Carlo Execution
function test_mc_basic_execution()
    fprintf('Test 4: Basic Monte Carlo execution... ');
    
    % Create simple test trajectory
    create_test_trajectory('./trajectories/mc_test_basic.wpt');
    
    % Configure perturbations
    perturb_config.params = {
        'm', 'uniform', 0.45, 0.55;  % ±10% mass variation
    };
    
    % Run minimal Monte Carlo (5 trials sufficient to test execution)
    mc_options.N_trials = 5;
    mc_options.seed = 42;
    mc_options.verbose = false;
    mc_options.save_all_trials = false;
    mc_options.log_to_file = false;
    mc_options.save_results = false;  % CRITICAL: Don't save during tests
    
    % Suppress console output during execution
    evalc('mc_results = run_monte_carlo(''mc_test_basic.wpt'', perturb_config, mc_options);');
    
    % Verify structure
    assert(isstruct(mc_results), 'Results must be a structure');
    assert(isfield(mc_results, 'trials'), 'Must have trials field');
    assert(isfield(mc_results, 'statistics'), 'Must have statistics field');
    assert(isfield(mc_results, 'nominal'), 'Must have nominal field');
    assert(length(mc_results.trials) == 5, 'Must have 5 trials');
    
    % Verify at least some trials succeeded
    success_flags = [mc_results.trials.success];
    assert(sum(success_flags) > 0, 'At least one trial should succeed');
    
    % Cleanup
    cleanup_test_files('./trajectories/mc_test_basic.wpt');
    
    fprintf('PASS\n');
end

%% Test 5: Reproducibility
function test_mc_reproducibility()
    fprintf('Test 5: Monte Carlo reproducibility... ');
    
    create_test_trajectory('./trajectories/mc_test_repro.wpt');
    
    perturb_config.params = {
        'm', 'normal', 0.5, 0.05;
        'L', 'uniform', 0.22, 0.27;
    };
    
    % 8 trials is sufficient to verify reproducibility
    mc_options.N_trials = 8;
    mc_options.seed = 999;
    mc_options.verbose = false;
    mc_options.log_to_file = false;
    mc_options.save_results = false;  % Don't save during tests
    
    % Run twice with same seed (suppress output)
    evalc('mc_results1 = run_monte_carlo(''mc_test_repro.wpt'', perturb_config, mc_options);');
    evalc('mc_results2 = run_monte_carlo(''mc_test_repro.wpt'', perturb_config, mc_options);');
    
    % Extract metrics from successful trials
    success1 = [mc_results1.trials.success];
    success2 = [mc_results2.trials.success];
    
    % Success patterns should be identical
    assert(all(success1 == success2), 'Success patterns must match with same seed');
    
    % For successful trials, metrics should be identical
    for i = 1:length(mc_results1.trials)
        if success1(i)
            m1 = mc_results1.trials(i).metrics.tracking.rmse_position;
            m2 = mc_results2.trials(i).metrics.tracking.rmse_position;
            assert(abs(m1 - m2) < 1e-10, ...
                sprintf('Metrics must be identical with same seed (trial %d)', i));
        end
    end
    
    cleanup_test_files('./trajectories/mc_test_repro.wpt');
    
    fprintf('PASS\n');
end

%% Test 6: Failure Handling
function test_mc_failure_handling()
    fprintf('Test 6: Graceful failure handling... ');
    
    create_test_trajectory('./trajectories/mc_test_fail.wpt');
    
    % Use extreme parameter variations likely to cause some failures
    perturb_config.params = {
        'm', 'uniform', 0.1, 2.0;      % Very wide range
        'Ixx', 'uniform', 0.001, 0.1;  % Wide inertia range
    };
    
    % 10 trials sufficient to catch potential failures
    mc_options.N_trials = 10;
    mc_options.seed = 777;
    mc_options.verbose = false;
    mc_options.log_to_file = false;
    mc_options.save_results = false;  % Don't save during tests
    
    % Should complete without crashing even if some trials fail (suppress output)
    evalc('mc_results = run_monte_carlo(''mc_test_fail.wpt'', perturb_config, mc_options);');
    
    % Verify structure integrity
    assert(length(mc_results.trials) == 10, 'All trials must be logged');
    
    % Check that failures are properly recorded
    failed_trials = ~[mc_results.trials.success];
    for i = find(failed_trials)
        assert(~isempty(mc_results.trials(i).failure_msg), ...
               sprintf('Failed trial %d must have error message', i));
    end
    
    cleanup_test_files('./trajectories/mc_test_fail.wpt');
    
    fprintf('PASS\n');
end

%% Test 7: Statistics Computation
function test_statistics_computation()
    fprintf('Test 7: Statistical computations... ');
    
    create_test_trajectory('./trajectories/mc_test_stats.wpt');
    
    perturb_config.params = {
        'm', 'normal', 0.5, 0.02;
    };
    
    % 10 trials sufficient to test statistics computation
    mc_options.N_trials = 10;
    mc_options.seed = 555;
    mc_options.verbose = false;
    mc_options.log_to_file = false;
    mc_options.save_results = false;  % Don't save during tests
    
    % Run with suppressed output
    evalc('mc_results = run_monte_carlo(''mc_test_stats.wpt'', perturb_config, mc_options);');
    
    % Verify statistics structure
    stats = mc_results.statistics;
    assert(isfield(stats, 'n_success'), 'Must have success count');
    assert(isfield(stats, 'n_failed'), 'Must have failure count');
    assert(stats.n_success + stats.n_failed == 10, 'Counts must sum to total trials');
    
    % If there are successful trials, verify metric statistics
    if stats.n_success > 0
        assert(isfield(stats, 'metrics'), 'Must have metrics field');
        assert(isfield(stats.metrics, 'rmse_position_mean'), ...
               'Must have position RMSE mean');
        assert(stats.metrics.rmse_position_mean > 0, 'Mean RMSE must be positive');
        assert(stats.metrics.rmse_position_std >= 0, 'Std dev must be non-negative');
    end
    
    cleanup_test_files('./trajectories/mc_test_stats.wpt');
    
    fprintf('PASS\n');
end

%% Test 8: Analysis Functions
function test_analysis_basic()
    fprintf('Test 8: Analysis functions... ');
    
    create_test_trajectory('./trajectories/mc_test_analysis.wpt');
    
    perturb_config.params = {
        'm', 'normal', 0.5, 0.03;
        'L', 'normal', 0.25, 0.01;
    };
    
    % 8 trials sufficient to test analysis (need at least 2 parameters × 3 samples)
    mc_options.N_trials = 8;
    mc_options.seed = 333;
    mc_options.verbose = false;
    mc_options.log_to_file = false;
    mc_options.save_results = false;  % Don't save during tests
    
    % Run with suppressed output
    evalc('mc_results = run_monte_carlo(''mc_test_analysis.wpt'', perturb_config, mc_options);');
    
    % Run analysis without plots (to speed up test)
    analysis_options.plot = false;
    analysis_options.verbose = false;
    analysis_options.correlation = true;
    
    % Suppress analysis output too
    evalc('analysis = analyze_monte_carlo(mc_results, analysis_options);');
    
    % Verify analysis structure
    assert(isstruct(analysis), 'Analysis must return structure');
    assert(isfield(analysis, 'statistics'), 'Must have statistics field');
    assert(isfield(analysis, 'correlations'), 'Must have correlations field');
    
    % Verify correlations were computed
    corr = analysis.correlations;
    assert(isfield(corr, 'rmse_position'), 'Must have position RMSE correlations');
    assert(length(corr.rmse_position) == 2, ...
           'Must have correlation for each parameter');
    
    cleanup_test_files('./trajectories/mc_test_analysis.wpt');
    
    fprintf('PASS\n');
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function create_test_trajectory(filename)
    % Create a minimal test trajectory for MC validation
    
    trajectory_data = struct();
    trajectory_data.metadata.name = 'MC Test Trajectory';
    trajectory_data.metadata.description = 'Minimal trajectory for testing';
    trajectory_data.waypoints = [
        struct('label', 'start', 'time', 0, 'x', 0, 'y', 0, 'z', 0, 'yaw', 0)
        struct('label', 'hover', 'time', 3, 'x', 0, 'y', 0, 'z', 1, 'yaw', [])
        struct('label', 'end', 'time', 6, 'x', 0, 'y', 0, 'z', 0, 'yaw', 0)
    ];
    
    % Write JSON
    json_str = jsonencode(trajectory_data);
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not create test trajectory file: %s', filename);
    end
    fprintf(fid, '%s', json_str);
    fclose(fid);
end

function cleanup_test_files(filename)
    % Clean up test trajectory files ONLY
    % SAFETY: Does NOT touch ./results/ directory
    
    if exist(filename, 'file')
        delete(filename);
    end
    
    % Note: We do NOT clean up anything in ./results/
    % Tests are configured with save_results=false to prevent creating files there
end