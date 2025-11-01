% EXAMPLE_PAPER_WORKFLOW - Demonstrates complete paper figure workflow
%
% This script shows the recommended workflow for generating figures for your
% ME590 paper. It separates data generation (expensive) from figure generation
% (fast), allowing you to iterate on visualizations without rerunning 4-hour
% Monte Carlo simulations.
%
% WORKFLOW:
%   1. Run this script to generate data (do once, takes hours)
%   2. Generate figures from saved data (do many times, takes seconds)
%   3. Iterate on figure appearance without regenerating data
%
% RUN TIME:
%   - Pilot study (500 trials):  ~20 minutes
%   - Full study (2000 trials):  ~4 hours with parallel processing

clear; clc; close all;
init_project();

%% ========================================================================
%% CONFIGURATION
%% ========================================================================

% Choose trajectory
TRAJECTORY_FILE = 'simple_square.wpt';

% Choose run configuration
RUN_TYPE = 'test';  % Options: 'test', 'pilot', 'full', 'custom'

fprintf('=================================================================\n');
fprintf('PAPER WORKFLOW EXAMPLE\n');
fprintf('=================================================================\n');
fprintf('Configuration: %s\n', RUN_TYPE);
fprintf('Trajectory:    %s\n', TRAJECTORY_FILE);
fprintf('=================================================================\n\n');

%% ========================================================================
%% STEP 1: GENERATE DATA (DO THIS ONCE)
%% ========================================================================

fprintf('STEP 1: DATA GENERATION\n');
fprintf('-----------------------------------------------------------------\n');

switch lower(RUN_TYPE)
    case 'test'
        % Quick test study - good for testing
        run_label = 'test';
        
        % Use defaults (50 trials, nominal uncertainties)
        fprintf('Running test study with default settings...\n');
        
        mc_options = struct();
        mc_options.N_trials = 50;     
        mc_options.parallel = true;   % Use all CPU cores
        mc_options.verbose = false;
        fprintf('Running pilot study with default settings...\n');
        fprintf('Expected time: ~1 minute\n\n');
        
        simulate_monte_carlo(TRAJECTORY_FILE, run_label, [], mc_options);

    case 'pilot'
        % Quick pilot study - good for more extensive testing of 
        % Monte Carlo analysis
        run_label = 'pilot_study';
        
        % Use defaults (500 trials, nominal uncertainties)
        mc_options = struct();
        mc_options.N_trials = 500;    % Paper-quality statistics
        mc_options.parallel = true;   % Use all CPU cores
        mc_options.verbose = false;
        fprintf('Running pilot study with default settings...\n');
        fprintf('Expected time: ~10-20 minutes\n\n');
        
        simulate_monte_carlo(TRAJECTORY_FILE, run_label, [], mc_options);
        
    case 'full'
        % Full production run for paper
        run_label = 'full_run';
        
        % Increase trials for better statistics
        mc_options = struct();
        mc_options.N_trials = 2000;  % Paper-quality statistics
        mc_options.parallel = true;   % Use all CPU cores
        mc_options.verbose = false;
        
        fprintf('Running full production study...\n');
        fprintf('Trials: %d (parallel processing enabled)\n', mc_options.N_trials);
        fprintf('Expected time: ~4 hours\n');
        fprintf('Recommendation: Start this before lunch/evening!\n\n');
        
        simulate_monte_carlo(TRAJECTORY_FILE, run_label, [], mc_options);
        
    case 'custom'
        % Custom perturbations
        run_label = 'custom_uncertainties';
        
        % Define custom parameter uncertainties
        params_nominal = quadrotor_linear_6dof();
        
        perturb_config = struct();
        perturb_config.params = {
            % Higher mass uncertainty (±20% instead of ±10%)
            'm',   'normal', params_nominal.m, params_nominal.m * 0.10;
            
            % Larger inertia uncertainty
            'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.10;
            'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.10;
            'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.10;
            
            % Standard arm length uncertainty
            'L',   'normal', params_nominal.L, params_nominal.L * 0.005;
        };
        
        mc_options = struct();
        mc_options.N_trials = 1000;
        mc_options.parallel = true;
        
        fprintf('Running custom uncertainty study...\n');
        fprintf('Trials: %d\n', mc_options.N_trials);
        fprintf('Custom perturbations: ±20%% mass, ±20%% inertia\n');
        fprintf('Expected time: ~2 hours\n\n');
        
        simulate_monte_carlo(TRAJECTORY_FILE, run_label, [], mc_options);
        
    otherwise
        error('Unknown RUN_TYPE: %s (use ''pilot'', ''full'', or ''custom'')', RUN_TYPE);
end

fprintf('\n');
fprintf('=================================================================\n');
fprintf('DATA GENERATION COMPLETE!\n');
fprintf('=================================================================\n\n');

%% ========================================================================
%% STEP 2: REVIEW METRICS
%% ========================================================================

fprintf('STEP 2: REVIEW METRICS\n');
fprintf('-----------------------------------------------------------------\n');

% Find the metrics file that was just created
metrics_files = dir(sprintf('./results/metrics_%s_*.txt', run_label));
if ~isempty(metrics_files)
    % Get most recent
    [~, idx] = max([metrics_files.datenum]);
    metrics_file = fullfile(metrics_files(idx).folder, metrics_files(idx).name);
    
    fprintf('Metrics saved to: %s\n', metrics_file);
    fprintf('\nTo view metrics:\n');
    fprintf('  >> edit %s\n', metrics_file);
    fprintf('  or in terminal: cat %s\n', metrics_file);
    
    % Display first few lines
    fprintf('\nPreview of metrics:\n');
    fprintf('-----------------------------------------------------------------\n');
    fid = fopen(metrics_file, 'r');
    for i = 1:15  % Show first 15 lines
        line = fgetl(fid);
        if line == -1, break; end
        fprintf('%s\n', line);
    end
    fclose(fid);
    fprintf('  [...see full file for complete metrics...]\n');
    fprintf('-----------------------------------------------------------------\n\n');
else
    warning('Could not find metrics file');
end

%% ========================================================================
%% STEP 3: GENERATE FIGURES
%% ========================================================================

fprintf('STEP 3: GENERATE FIGURES\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('Generating all paper figures from saved data...\n');
fprintf('This should take ~10-20 seconds (no simulation rerun!)\n\n');

fig_options.close_figures = true;  % Close figures after saving for clean batch processing
fig_options.verbose = true;

% Generate figures from saved data
generate_paper_figures(run_label, 'latest', fig_options);

fprintf('\n');

%% ========================================================================
%% STEP 4: EXAMINE RESULTS
%% ========================================================================

fprintf('STEP 4: EXAMINE RESULTS\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('All files saved successfully!\n\n');

% Find the output directory
run_dir_name = sprintf('%s_%s', run_label, datestr(now, 'yyyymmdd_HHMMSS'));
figures_dir = fullfile('.', 'figures', run_dir_name);

fprintf('Output locations:\n');
fprintf('  Results:  ./results/%s/\n', run_dir_name);
fprintf('  Figures:  ./figures/%s/\n', run_dir_name);
fprintf('\n');

fprintf('Generated figures:\n');
fprintf('  [1] square_tracking_3d.png\n');
fprintf('  [2] tracking_timeseries.png\n');
fprintf('  [3] control_inputs.png\n');
fprintf('  [4] mc_boxplots.png\n');
fprintf('  [5] mc_correlation.png\n');
fprintf('  [6] mc_distributions.png\n');
fprintf('\n');

fprintf('Metrics summary:\n');
fprintf('  Location: %s\n', fullfile(figures_dir, 'paper_metrics.txt'));
fprintf('  Contains: LaTeX-ready snippets for direct insertion into paper\n');
fprintf('\n');

% Load and display key metrics from nominal simulation
nominal_file = fullfile('.', 'results', run_dir_name, 'nominal.mat');
if exist(nominal_file, 'file')
    nominal = DataManager.load_results(nominal_file);
    fprintf('Quick metrics preview:\n');
    if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
        fprintf('  Nominal RMSE (position): %.4f m\n', nominal.metrics.tracking.rmse_position);
        fprintf('  Nominal RMSE (attitude): %.2f deg\n', rad2deg(nominal.metrics.tracking.rmse_attitude));
    end
end

% Load and display Monte Carlo summary
mc_file = fullfile('.', 'results', run_dir_name, 'monte_carlo.mat');
if exist(mc_file, 'file')
    mc = DataManager.load_results(mc_file, struct('validate', false, 'migrate', false));
    fprintf('  MC Success Rate:         %.1f%%\n', mc.statistics.success_rate);
    if isfield(mc.statistics, 'metrics')
        fprintf('  MC Position RMSE:        %.4f ± %.4f m\n', ...
                mc.statistics.metrics.rmse_position_mean, ...
                mc.statistics.metrics.rmse_position_std);
    end
end
fprintf('\n');

%% ========================================================================
%% CLEAN UP
%% ========================================================================
% Force close all file handles
fclose('all'); 


%% ========================================================================
%% WORKFLOW SUMMARY
%% ========================================================================

fprintf('\n');
fprintf('=================================================================\n');
fprintf('WORKFLOW COMPLETE!\n');
fprintf('=================================================================\n');
fprintf('What was created:\n');
fprintf('  [1] Data files in ./results/ (permanent, can reuse)\n');
fprintf('  [2] Metrics report in ./results/ (human-readable)\n');
fprintf('  [3] Figures in ./figures/ (paper-ready)\n');
fprintf('  [4] Metrics table in ./figures/paper_metrics.txt\n\n');

fprintf('Next steps:\n');
fprintf('  1. Review figures in ./figures/\n');
fprintf('  2. Read metrics in ./figures/paper_metrics.txt\n');
fprintf('  3. Insert figures into LaTeX paper\n');
fprintf('  4. Copy metrics into paper placeholders\n\n');

fprintf('To regenerate figures (without rerunning simulation):\n');
fprintf('  >> generate_paper_figures(''%s'', ''latest'')\n\n', run_label);

fprintf('To run a different configuration:\n');
fprintf('  1. Change RUN_TYPE in this script\n');
fprintf('  2. Run this script again\n');
fprintf('  3. Compare results using different run_labels\n');
fprintf('=================================================================\n\n');

%% ========================================================================
%% BONUS: COMPARISON ACROSS RUNS
%% ========================================================================

fprintf('BONUS TIP: Comparing multiple runs\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('If you have multiple runs (e.g., ''pilot_study'', ''full_run''):\n\n');

fprintf('% Load different configurations\n');
fprintf('load(''results/mc_pilot_study_[timestamp].mat'');\n');
fprintf('mc_pilot = mc_results;\n');
fprintf('load(''results/mc_full_run_[timestamp].mat'');\n');
fprintf('mc_full = mc_results;\n\n');

fprintf('%% Compare statistics\n');
fprintf('fprintf(''Pilot:  RMSE = %%.4f m\\n'', mc_pilot.statistics.metrics.rmse_position_mean);\n');
fprintf('fprintf(''Full:   RMSE = %%.4f m\\n'', mc_full.statistics.metrics.rmse_position_mean);\n\n');

fprintf('Or generate separate figure sets:\n');
fprintf('generate_paper_figures(''pilot_study'', ''latest'', ''./figures_pilot/'');\n');
fprintf('generate_paper_figures(''full_run'', ''latest'', ''./figures_full/'');\n');
fprintf('-----------------------------------------------------------------\n\n');