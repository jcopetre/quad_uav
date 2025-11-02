function mc_results = run_monte_carlo(trajectory_file, perturb_config, mc_options)
% RUN_MONTE_CARLO - Monte Carlo analysis for quadrotor robustness
%
% Performs Monte Carlo simulation with parameter uncertainties to assess
% controller robustness. Supports parallel processing and reproducible results.
%
% SYNTAX:
%   mc_results = run_monte_carlo(trajectory_file, perturb_config)
%   mc_results = run_monte_carlo(trajectory_file, perturb_config, mc_options)
%
% INPUTS:
%   trajectory_file - Waypoint file in ./trajectories/ (e.g., 'basic_maneuver.wpt')
%   
%   perturb_config  - Structure defining parameter variations:
%                     .params - Cell array {param_name, dist_type, param1, param2}
%                               dist_type: 'normal', 'uniform', or 'fixed'
%                     .x0     - (optional) Initial state perturbations
%                     
%                     Example:
%                     perturb_config.params = {
%                         'm',   'normal',  0.5,  0.05;      % mean, std
%                         'Ixx', 'normal',  0.01, 0.002;
%                         'L',   'uniform', 0.22, 0.27;      % min, max
%                     };
%   
%   mc_options      - (optional) Monte Carlo configuration:
%                     .N_trials     - Number of trials (default: 100)
%                     .seed         - Random seed for reproducibility (default: 42)
%                     .parallel     - Use parfor (default: false)
%                     .nominal_Q    - LQR state weights (default: from quadrotor_linear_6dof)
%                     .nominal_R    - LQR control weights (default: from quadrotor_linear_6dof)
%                     .save_file    - Save results to file (default: auto-generated)
%                     .verbose      - Progress reporting (default: true)
%                     .save_all_trials - Save full state history (default: false, saves memory)
%
% OUTPUTS:
%   mc_results - Structure containing:
%                .config          - Configuration used (perturb_config, mc_options)
%                .nominal         - Nominal (unperturbed) simulation results
%                .trials          - Array of structures with trial data:
%                                   .params      - Perturbed parameters
%                                   .metrics     - Performance metrics
%                                   .success     - Boolean flag
%                                   .failure_msg - Error message if failed
%                                   .t, .x, .u   - (optional) Full histories
%                .statistics      - Summary statistics across all trials
%                .timestamp       - When analysis was performed
%                .elapsed_time    - Total computation time
%
% EXAMPLE:
%   % Define parameter uncertainties
%   perturb_config.params = {
%       'm',   'normal',  0.5,  0.05;      % mass: 0.5 ± 10%
%       'Ixx', 'normal',  0.01, 0.002;     % Ixx: 0.01 ± 20%
%       'Iyy', 'normal',  0.01, 0.002;
%       'Izz', 'normal',  0.02, 0.004;
%       'L',   'uniform', 0.22, 0.27;      % arm length: uniform [0.22, 0.27]
%   };
%   
%   % Configure Monte Carlo run
%   mc_options.N_trials = 500;
%   mc_options.seed = 12345;
%   mc_options.parallel = true;
%   mc_options.verbose = true;
%   
%   % Run analysis
%   mc_results = run_monte_carlo('basic_maneuver.wpt', perturb_config, mc_options);
%   
%   % Analyze results
%   analyze_monte_carlo(mc_results);
%
% NOTES:
%   - Failed trials (e.g., unstable) are logged but don't stop execution
%   - Nominal controller (K matrix) is designed once and held fixed
%   - Physical parameter bounds are enforced (mass > 0, inertia > 0, etc.)
%   - For parallel processing, ensure init_project() was run before parpool
%
% See also: analyze_monte_carlo_results, simulate_quadrotor_pure

% Author: Trey Copeland
% Date: 2025-10-15

    %% Parse inputs and set defaults
    if nargin < 3
        mc_options = struct();
    end
    
    % Define default MC options
    defaults.N_trials = 100;
    defaults.seed = 42;
    defaults.parallel = false;
    defaults.verbose = true;
    defaults.save_file = '';
    defaults.save_all_trials = false;
    
    mc_options = set_default_options(mc_options, defaults);
    
    % Validate inputs
    assert(ischar(trajectory_file) || isstring(trajectory_file), ...
           'trajectory_file must be a string');
    assert(isstruct(perturb_config) && isfield(perturb_config, 'params'), ...
           'perturb_config must have .params field');
    assert(mc_options.N_trials > 0, 'N_trials must be positive');
    
    %% Initialize
    if mc_options.verbose
        fprintf('\n===================================\n');
        fprintf('MONTE CARLO ROBUSTNESS ANALYSIS\n');
        fprintf('===================================\n\n');
        fprintf('Trajectory:  %s\n', trajectory_file);
        fprintf('Trials:      %d\n', mc_options.N_trials);
        fprintf('Seed:        %d\n', mc_options.seed);
        fprintf('Parallel:    %s\n', bool2str(mc_options.parallel));
        fprintf('\n');
    end
    
    tic;  % Start timing
    
    %% Design nominal controller (fixed for all trials)
    if mc_options.verbose
        fprintf('Step 1/5: Designing nominal controller...\n');
    end
    
    if isfield(mc_options, 'nominal_Q') && isfield(mc_options, 'nominal_R')
        params_nominal = quadrotor_linear_6dof(mc_options.nominal_Q, mc_options.nominal_R);
    else
        params_nominal = quadrotor_linear_6dof();  % Use defaults
    end
    
    %% Run nominal (unperturbed) simulation
    if mc_options.verbose
        fprintf('Step 2/5: Running nominal simulation...\n');
    end
    
    opts_nominal.verbose = false;
    opts_nominal.save_results = false;
    opts_nominal.plot = false;
    opts_nominal.params = params_nominal;
    
    try
        results_nominal = simulate_trajectory(trajectory_file, [], [], [], opts_nominal);
        nominal_success = true;
    catch ME
        warning('Nominal simulation failed: %s', ME.message);
        results_nominal = [];
        nominal_success = false;
    end
    
    %% Generate parameter samples
    if mc_options.verbose
        fprintf('Step 3/5: Generating parameter samples...\n');
    end
    
    % Set random seed for reproducibility
    rng(mc_options.seed, 'twister');
    
    N_trials = mc_options.N_trials;
    param_samples = generate_samples(perturb_config, N_trials, params_nominal);
    
    %% Run Monte Carlo trials
    if mc_options.verbose
        fprintf('Step 4/5: Running %d Monte Carlo trials...\n', N_trials);
        if mc_options.parallel
            fprintf('            (Using parallel processing)\n');
        end
    end
    
    % Pre-allocate results cell array (avoids structure assignment issues)
    trials_cell = cell(N_trials, 1);
    
    % Simulation options (same for all trials)
    opts_sim.verbose = false;
    opts_sim.save_results = false;
    opts_sim.plot = false;
    
    % Progress reporting setup
    if mc_options.verbose
        progress_interval = max(1, floor(N_trials / 20));  % Report every 5%
        fprintf('Progress: ');
    end
    
    %% Run trials (parallel or serial)
    if mc_options.parallel
        % Parallel execution
        parfor i = 1:N_trials
            trials(i) = run_single_trial(i, trajectory_file, param_samples(i), ...
                                         params_nominal, opts_sim, ...
                                         mc_options.save_all_trials);
        end
        if mc_options.verbose
            fprintf('Done!\n');
        end
    else
        % Serial execution with progress reporting
        for i = 1:N_trials
            trials(i) = run_single_trial(i, trajectory_file, param_samples(i), ...
                                         params_nominal, opts_sim, ...
                                         mc_options.save_all_trials);
            
            % Progress reporting
            if mc_options.verbose && mod(i, progress_interval) == 0
                fprintf('.');
            end
        end
        if mc_options.verbose
            fprintf(' Done!\n');
        end
    end
    
    %% Compute statistics
    if mc_options.verbose
        fprintf('Step 5/5: Computing statistics...\n');
    end
    
    statistics = compute_mc_statistics(trials);
    
    %% Package results
    mc_results.config.perturb_config = perturb_config;
    mc_results.config.mc_options = mc_options;
    mc_results.config.trajectory_file = trajectory_file;
    
    mc_results.nominal.results = results_nominal;
    mc_results.nominal.success = nominal_success;
    mc_results.nominal.params = params_nominal;
    
    mc_results.trials = trials;
    mc_results.statistics = statistics;
    mc_results.timestamp = datetime('now');
    mc_results.elapsed_time = toc;
    
    %% Summary report
    if mc_options.verbose
        fprintf('\n===================================\n');
        fprintf('MONTE CARLO SUMMARY\n');
        fprintf('===================================\n');
        fprintf('Total trials:      %d\n', N_trials);
        fprintf('Successful:        %d (%.1f%%)\n', statistics.n_success, ...
                100 * statistics.n_success / N_trials);
        fprintf('Failed:            %d (%.1f%%)\n', statistics.n_failed, ...
                100 * statistics.n_failed / N_trials);
        fprintf('Elapsed time:      %.2f seconds\n', mc_results.elapsed_time);
        fprintf('Time per trial:    %.3f seconds\n', mc_results.elapsed_time / N_trials);
        
        if statistics.n_success > 0
            fprintf('\nPerformance Summary (Successful Trials):\n');
            fprintf('  Position RMSE:   %.4f ± %.4f m\n', ...
                    statistics.metrics.rmse_position_mean, ...
                    statistics.metrics.rmse_position_std);
            fprintf('  Attitude RMSE:   %.4f ± %.4f rad (%.2f ± %.2f deg)\n', ...
                    statistics.metrics.rmse_attitude_mean, ...
                    statistics.metrics.rmse_attitude_std, ...
                    rad2deg(statistics.metrics.rmse_attitude_mean), ...
                    rad2deg(statistics.metrics.rmse_attitude_std));
            fprintf('  Control effort:  %.2f ± %.2f\n', ...
                    statistics.metrics.control_effort_mean, ...
                    statistics.metrics.control_effort_std);
        end
        fprintf('\n===================================\n\n');
    end
    
    %% Save results
    if ~isempty(mc_options.save_file)
        % Use DataManager for validated save
        [filepath_dir, filename_base, ~] = fileparts(mc_options.save_file);
        if isempty(filepath_dir)
            filepath_dir = './results';
        end
        
        save_options = struct('verbose', mc_options.verbose, 'validate', true);
        actual_filepath = DataManager.save_results(mc_results, filename_base, filepath_dir, save_options);
        
        if mc_options.verbose && ~strcmp(actual_filepath, mc_options.save_file)
            fprintf('Note: File saved to: %s\n', actual_filepath);
        end
    elseif mc_options.verbose
        fprintf('Use DataManager.save_results(mc_results, ''label'', ''results'') to save.\n\n');
    end
    
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function samples = generate_samples(perturb_config, N_trials, params_nominal)
    % Generate parameter samples based on perturbation configuration
    
    n_params = size(perturb_config.params, 1);
    samples = repmat(params_nominal, N_trials, 1);
    
    for i = 1:n_params
        param_name = perturb_config.params{i, 1};
        dist_type = perturb_config.params{i, 2};
        param1 = perturb_config.params{i, 3};
        param2 = perturb_config.params{i, 4};
        
        switch lower(dist_type)
            case 'normal'
                % Normal distribution: param1 = mean, param2 = std
                values = param1 + param2 * randn(N_trials, 1);
                
            case 'uniform'
                % Uniform distribution: param1 = min, param2 = max
                values = param1 + (param2 - param1) * rand(N_trials, 1);
                
            case 'fixed'
                % Fixed value: param1 = value
                values = param1 * ones(N_trials, 1);
                
            otherwise
                error('Unknown distribution type: %s', dist_type);
        end
        
        % Enforce physical constraints
        values = enforce_constraints(param_name, values);
        
        % Assign to samples
        for j = 1:N_trials
            samples(j).(param_name) = values(j);
        end
    end
end

function values = enforce_constraints(param_name, values)
    % Enforce physical constraints on parameters
    
    switch param_name
        case 'm'
            % Mass must be positive
            values = max(values, 0.05);  % Minimum 50g
            
        case {'Ixx', 'Iyy', 'Izz'}
            % Moments of inertia must be positive
            values = max(values, 1e-6);
            
        case 'L'
            % Arm length must be positive
            values = max(values, 0.05);  % Minimum 5cm
            
        case {'k_thrust', 'k_torque'}
            % Thrust/torque coefficients must be positive
            values = max(values, 0);
            
        % Add other constraints as needed
    end
end

function trial = run_single_trial(trial_num, trajectory_file, params_perturbed, ...
                                  params_nominal, opts_sim, save_full_history)
    % Run a single Monte Carlo trial
    
    % Use perturbed physical parameters with nominal controller
    opts_sim.params_plant = params_perturbed;
    opts_sim.params.K = params_nominal.K;        % Keep nominal controller
    opts_sim.params.u_hover = params_nominal.u_hover;
    
    % Initialize trial structure
    trial.params = params_perturbed;
    trial.trial_num = trial_num;
    
    try
        % Run simulation
        results = simulate_trajectory(trajectory_file, [], [], [], opts_sim);
        
        % Store results
        trial.metrics = results.metrics;
        trial.success = true;
        trial.failure_msg = '';
        
        % Optionally save full state history
        if save_full_history
            trial.t = results.t;
            trial.x = results.x;
            trial.u = results.u_log;
        else
            trial.t = [];
            trial.x = [];
            trial.u = [];
        end
        
    catch ME
        % Trial failed (e.g., unstable, integration error)
        trial.metrics = struct();
        trial.success = false;
        trial.failure_msg = ME.message;
        trial.t = [];
        trial.x = [];
        trial.u = [];
    end
end

function statistics = compute_mc_statistics(trials)
    % Compute summary statistics across all Monte Carlo trials
    
    N_trials = length(trials);
    success_flags = [trials.success];
    
    statistics.n_trials = N_trials;
    statistics.n_success = sum(success_flags);
    statistics.n_failed = sum(~success_flags);
    statistics.success_rate = 100 * statistics.n_success / N_trials;
    
    % Extract metrics from successful trials
    if statistics.n_success > 0
        successful_trials = trials(success_flags);
        
        % Position tracking
        rmse_pos = arrayfun(@(t) t.metrics.tracking.rmse_position, successful_trials);
        statistics.metrics.rmse_position_mean = mean(rmse_pos);
        statistics.metrics.rmse_position_std = std(rmse_pos);
        statistics.metrics.rmse_position_median = median(rmse_pos);
        statistics.metrics.rmse_position_percentiles = prctile(rmse_pos, [5 25 75 95]);
        
        % Attitude tracking
        rmse_att = arrayfun(@(t) t.metrics.tracking.rmse_attitude, successful_trials);
        statistics.metrics.rmse_attitude_mean = mean(rmse_att);
        statistics.metrics.rmse_attitude_std = std(rmse_att);
        statistics.metrics.rmse_attitude_median = median(rmse_att);
        statistics.metrics.rmse_attitude_percentiles = prctile(rmse_att, [5 25 75 95]);
        
        % Control effort
        if isfield(successful_trials(1).metrics, 'control') && ...
           ~isempty(fieldnames(successful_trials(1).metrics.control))
            control_effort = arrayfun(@(t) t.metrics.control.total_effort, successful_trials);
            statistics.metrics.control_effort_mean = mean(control_effort);
            statistics.metrics.control_effort_std = std(control_effort);
            statistics.metrics.control_effort_median = median(control_effort);
            statistics.metrics.control_effort_percentiles = prctile(control_effort, [5 25 75 95]);
        end
        
        % Max position error
        max_pos_err = arrayfun(@(t) t.metrics.tracking.max_position_error, successful_trials);
        statistics.metrics.max_position_error_mean = mean(max_pos_err);
        statistics.metrics.max_position_error_std = std(max_pos_err);
        statistics.metrics.max_position_error_percentiles = prctile(max_pos_err, [5 25 75 95]);
        
    else
        % No successful trials
        statistics.metrics = struct();
    end
    
    % Failure analysis
    if statistics.n_failed > 0
        failed_trials = trials(~success_flags);
        failure_messages = {failed_trials.failure_msg};
        statistics.failure_messages = failure_messages;
    end
end

function str = bool2str(val)
    % Convert boolean to string for display
    if val
        str = 'Yes';
    else
        str = 'No';
    end
end