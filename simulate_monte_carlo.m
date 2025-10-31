% SIMULATE_MONTE_CARLO - Orchestrator for Monte Carlo robustness study
%
% High-level script that runs both nominal simulation and Monte Carlo analysis,
% saving all results with a user-specified run label for easy identification.
%
% SYNTAX:
%   simulate_monte_carlo(trajectory_file, run_label)
%   simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options)
%
% INPUTS:
%   trajectory_file - Waypoint file in ./trajectories/ (e.g., 'simple_square.wpt')
%   run_label       - String identifier for this run (e.g., 'full_run', 'baseline')
%                     Used in output filenames and saved in results structure
%   perturb_config  - (optional) Parameter perturbation configuration
%   mc_options      - (optional) Monte Carlo options
%
% OUTPUTS:
%   Saves to ./results/ with run_label in filename:
%     - nominal_[run_label]_[timestamp].mat    (single simulation)
%     - mc_[run_label]_[timestamp].mat         (Monte Carlo results)
%     - metrics_[run_label]_[timestamp].txt    (human-readable metrics)
%
% EXAMPLE:
%   % Run baseline configuration
%   simulate_monte_carlo('simple_square.wpt', 'baseline');
%
%   % Run with custom perturbations
%   perturb_config.params = {
%       'm',   'normal', 0.5, 0.025;
%       'Ixx', 'normal', 0.01, 0.0015;
%   };
%   mc_options.N_trials = 2000;
%   simulate_monte_carlo('simple_square.wpt', 'full_run', perturb_config, mc_options);

function simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options)
    
%% Input Handling
    if nargin < 3 || isempty(perturb_config)
        % Use default values
        params = quadrotor_linear_6dof();
        perturb_config.params = {
            'm',   'normal', params.m,   params.m * 0.05;
            'Ixx', 'normal', params.Ixx, params.Ixx * 0.075;
            'Iyy', 'normal', params.Iyy, params.Iyy * 0.075;
            'Izz', 'normal', params.Izz, params.Izz * 0.075;
            'L',   'normal', params.L,   params.L * 0.005;
        };
    end
    
    if nargin < 4
        mc_options = struct();
    end
    
    % Set defaults for mc_options
    if ~isfield(mc_options, 'N_trials')
        mc_options.N_trials = 500;
    end
    if ~isfield(mc_options, 'parallel')
        mc_options.parallel = true;
    end
    if ~isfield(mc_options, 'verbose')
        mc_options.verbose = false;
    end
    
    %% Create Nested Results Directory
    timestamp_str = datestr(now, 'yyyymmdd_HHMMSS');
    run_dir_name = sprintf('%s_%s', run_label, timestamp_str);
    results_dir = fullfile('.', 'results', run_dir_name);
    
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    %% Console Header
    fprintf('\n=======================================================\n');
    fprintf('MONTE CARLO STUDY: %s\n', run_label);
    fprintf('=======================================================\n');
    fprintf('Results directory: %s\n', results_dir);
    fprintf('Timestamp: %s\n', timestamp_str);
    fprintf('N_trials: %d\n', mc_options.N_trials);
    fprintf('=======================================================\n\n');
    
    %% Step 1: Run Nominal Simulation
    fprintf('[1/3] Running nominal simulation...\n');
    
    sim_options.verbose = mc_options.verbose;
    sim_options.plot = false;           % No plots during batch processing
    sim_options.save_results = false;   % Disable auto-save (we handle saving)
    nominal_results = simulate_quadrotor_pure(trajectory_file, [], [], [], sim_options);
    
    % Save in nested directory with simple filename
    nominal_file = fullfile(results_dir, 'nominal.mat');
    save(nominal_file, '-struct', 'nominal_results');
    fprintf('      Saved: %s\n\n', nominal_file);
    
    %% Step 2: Run Monte Carlo Analysis
    fprintf('[2/3] Running Monte Carlo analysis...\n');
    fprintf('      (N=%d trials, parallel=%d, verbose=%d)\n', ...
            mc_options.N_trials, mc_options.parallel, mc_options.verbose);
    
    mc_results = run_monte_carlo(trajectory_file, perturb_config, mc_options);
    
    % Save in nested directory with simple filename
    mc_file = fullfile(results_dir, 'monte_carlo.mat');
    save(mc_file, '-struct', 'mc_results');
    fprintf('      Saved: %s\n\n', mc_file);
    
    %% Step 3: Write Metrics Report
    fprintf('[3/3] Writing metrics report...\n');
    
    metrics_file = fullfile(results_dir, 'metrics.txt');
    save_metrics_report(nominal_results, mc_results, metrics_file);
    fprintf('      Saved: %s\n\n', metrics_file);
    
    %% Final Instructions
    fprintf('=======================================================\n');
    fprintf('✅ STUDY COMPLETE: %s\n', run_label);
    fprintf('=======================================================\n');
    fprintf('Directory: %s\n', results_dir);
    fprintf('\nTo generate figures:\n');
    fprintf('  >> generate_paper_figures(''%s'', ''%s'');\n', run_label, timestamp_str);
    fprintf('  >> generate_paper_figures(''%s'', ''latest'');\n', run_label);
    fprintf('\nResults saved:\n');
    fprintf('  - %s\n', nominal_file);
    fprintf('  - %s\n', mc_file);
    fprintf('  - %s\n', metrics_file);
    fprintf('=======================================================\n\n');
end