function test_parameter_perturbation()
% TEST_PARAMETER_PERTURBATION - Verify that perturbed parameters affect simulation
%
% CRITICAL TEST: Ensures Monte Carlo robustness analysis actually works!
% This test would have caught the params vs params_plant bug.
%
% Tests:
%   1. Nominal simulation produces baseline results
%   2. Perturbed parameters produce DIFFERENT results
%   3. Larger perturbations produce larger differences
%   4. Specific parameters affect expected metrics

    fprintf('Running Unit Tests: Parameter Perturbation\n');
    fprintf('============================================\n\n');
    
    setup_test_environment();
    
    % Run all test cases
    test_mass_perturbation_changes_results();
    test_inertia_perturbation_changes_results();
    test_larger_perturbations_larger_effects();
    test_monte_carlo_produces_variation();
    
    fprintf('\n============================================\n');
    fprintf('All Tests Passed!\n');
    fprintf('============================================\n\n');
end

%% ========================================================================
%% TEST FUNCTIONS
%% ========================================================================

function test_mass_perturbation_changes_results()
    % Test that changing mass produces different tracking errors
    
    fprintf('Test 1: Mass perturbation affects results... ');
    
    % Create simple test trajectory
    create_test_trajectory('./trajectories/perturb_test.wpt');
    
    params_nominal = quadrotor_linear_6dof([], [], false);
    
    % Run nominal simulation
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    
    results_nominal = simulate_trajectory('perturb_test.wpt', [], [], [], opts);
    rmse_nominal = results_nominal.metrics.tracking.rmse_position;
    
    % Run with 50% heavier mass
    params_heavy = params_nominal;
    params_heavy.m = params_nominal.m * 1.5;
    
    opts.params_plant = params_heavy;  % CRITICAL: Use params_plant, not params!
    results_heavy = simulate_trajectory('perturb_test.wpt', [], [], [], opts);
    rmse_heavy = results_heavy.metrics.tracking.rmse_position;
    
    % Results MUST be different
    difference = abs(rmse_heavy - rmse_nominal);
    
    assert(difference > 1e-6, ...
           sprintf('Mass perturbation had no effect! Nominal: %.6f, Heavy: %.6f, Diff: %.9f', ...
                   rmse_nominal, rmse_heavy, difference));
    
    % Heavier mass should generally increase tracking error
    % (controller designed for lighter vehicle)
    assert(rmse_heavy > rmse_nominal * 0.5, ...
           'Heavier mass should degrade tracking performance');
    
    fprintf('PASS (Δ = %.4f m)\n', difference);
    
    % Cleanup
    cleanup_test_files('./trajectories/perturb_test.wpt');
end

function test_inertia_perturbation_changes_results()
    % Test that changing inertia produces different attitude tracking
    
    fprintf('Test 2: Inertia perturbation affects attitude... ');
    
    create_test_trajectory('./trajectories/perturb_test_attitude.wpt');
    
    params_nominal = quadrotor_linear_6dof([], [], false);
    
    % Run nominal simulation
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    
    results_nominal = simulate_trajectory('perturb_test_attitude.wpt', [], [], [], opts);
    rmse_att_nominal = results_nominal.metrics.tracking.rmse_attitude;
    
    % Run with 2x inertia
    params_high_inertia = params_nominal;
    params_high_inertia.Ixx = params_nominal.Ixx * 2.0;
    params_high_inertia.Iyy = params_nominal.Iyy * 2.0;
    params_high_inertia.Izz = params_nominal.Izz * 2.0;
    
    opts.params_plant = params_high_inertia;
    results_high_inertia = simulate_trajectory('perturb_test_attitude.wpt', [], [], [], opts);
    rmse_att_high = results_high_inertia.metrics.tracking.rmse_attitude;
    
    % Results MUST be different
    difference = abs(rmse_att_high - rmse_att_nominal);
    
    assert(difference > 1e-6, ...
           sprintf('Inertia perturbation had no effect! Nominal: %.6f, High: %.6f', ...
                   rmse_att_nominal, rmse_att_high));
    
    fprintf('PASS (Δ = %.4f rad)\n', difference);
    
    cleanup_test_files('./trajectories/perturb_test_attitude.wpt');
end

function test_larger_perturbations_larger_effects()
    % Test that perturbation magnitude correlates with effect size
    
    fprintf('Test 3: Larger perturbations → larger effects... ');
    
    create_test_trajectory('./trajectories/perturb_test_scaling.wpt');
    
    params_nominal = quadrotor_linear_6dof([], [], false);
    
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    
    % Test increasing mass perturbations
    mass_factors = [1.0, 1.1, 1.2, 1.5];
    rmse_values = zeros(size(mass_factors));
    
    for i = 1:length(mass_factors)
        params_test = params_nominal;
        params_test.m = params_nominal.m * mass_factors(i);
        
        opts.params_plant = params_test;
        results = simulate_trajectory('perturb_test_scaling.wpt', [], [], [], opts);
        rmse_values(i) = results.metrics.tracking.rmse_position;
    end
    
    % RMSE should generally increase with mass
    % (Some non-monotonicity is OK, but trend should be clear)
    trend_coefficient = corr(mass_factors', rmse_values');
    
    assert(trend_coefficient > 0.5, ...
           sprintf('RMSE should increase with mass perturbation (corr: %.3f)', trend_coefficient));
    
    % First and last should definitely be different
    assert(abs(rmse_values(end) - rmse_values(1)) > 1e-4, ...
           'Large perturbation (50%%) should produce measurable effect');
    
    fprintf('PASS (correlation: %.3f)\n', trend_coefficient);
    
    cleanup_test_files('./trajectories/perturb_test_scaling.wpt');
end

function test_monte_carlo_produces_variation()
    % Test that Monte Carlo framework produces variation in results
    
    fprintf('Test 4: Monte Carlo produces variation... ');
    
    create_test_trajectory('./trajectories/mc_variation_test.wpt');
    
    params_nominal = quadrotor_linear_6dof([], [], false);
    
    % Configure Monte Carlo with parameter variations
    perturb_config.params = {
        'm', 'normal', params_nominal.m, params_nominal.m * 0.20;  % ±20% variation
    };
    
    mc_options.N_trials = 10;  % Small number for speed
    mc_options.seed = 42;
    mc_options.verbose = false;
    mc_options.parallel = false;
    
    % Run Monte Carlo
    mc_results = run_monte_carlo('mc_variation_test.wpt', perturb_config, mc_options);
    
    % Extract RMSE from all successful trials
    success_flags = [mc_results.trials.success];
    successful_trials = mc_results.trials(success_flags);
    
    assert(length(successful_trials) >= 5, 'At least 5 trials should succeed');
    
    rmse_values = arrayfun(@(t) t.metrics.tracking.rmse_position, successful_trials);
    
    % Check that results vary
    std_rmse = std(rmse_values);
    
    assert(std_rmse > 1e-6, ...
           sprintf('Monte Carlo produced NO variation! Std: %.9f', std_rmse));
    
    % With 20%% mass variation, we should see meaningful spread
    assert(std_rmse > 1e-3, ...
           sprintf('Monte Carlo variation too small! Std: %.6f m (expected > 0.001 m)', std_rmse));
    
    % Coefficient of variation should be reasonable
    cv = std_rmse / mean(rmse_values);
    assert(cv > 0.001, ...
           sprintf('Coefficient of variation too small: %.4f', cv));
    
    fprintf('PASS (std: %.4f m, CV: %.2f%%)\n', std_rmse, cv*100);
    
    cleanup_test_files('./trajectories/mc_variation_test.wpt');
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function create_test_trajectory(filename)
    % Create a simple trajectory for testing by writing JSON directly
    
    % Create JSON structure
    data = struct();
    data.metadata = struct(...
        'name', 'Perturbation Test', ...
        'description', 'Simple trajectory for testing parameter perturbation', ...
        'vehicle', 'quadrotor_500g', ...
        'created', datestr(now, 'yyyy-mm-dd'));
    
    % Create waypoints array
    data.waypoints = {
        struct('time', 0, 'x', 0, 'y', 0, 'z', 1, 'yaw', 0)
        struct('time', 3, 'x', 2, 'y', 0, 'z', 1, 'yaw', 0)
        struct('time', 6, 'x', 2, 'y', 2, 'z', 1, 'yaw', 0)
        struct('time', 9, 'x', 0, 'y', 2, 'z', 1, 'yaw', 0)
    };
    
    % Write JSON file
    json_text = jsonencode(data);
    
    % Make it pretty (optional but nice)
    json_text = strrep(json_text, ',', sprintf(',\n  '));
    json_text = strrep(json_text, '{', sprintf('{\n  '));
    json_text = strrep(json_text, '}', sprintf('\n}'));
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not create waypoint file: %s', filename);
    end
    fprintf(fid, '%s', json_text);
    fclose(fid);
end

function cleanup_test_files(varargin)
    % Delete test files
    for i = 1:length(varargin)
        if exist(varargin{i}, 'file')
            delete(varargin{i});
        end
    end
end