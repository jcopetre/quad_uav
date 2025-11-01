% GENERATE_RESULTS - Complete workflow for Monte Carlo study with figure generation
%
% This script demonstrates the complete workflow:
%   1. Run Monte Carlo simulation (expensive, do once)
%   2. Generate publication figures (fast, iterate as needed)
%
% WORKFLOW:
%   1. Run this script to generate data (do once, takes hours)
%   2. Generate figures from saved data (do many times, takes seconds)
%   3. Iterate on figure appearance without regenerating data

clear; clc; close all;
init_project();

%% ========================================================================
%% CONFIGURATION
%% ========================================================================

% Choose trajectory
TRAJECTORY_FILE = 'simple_square.wpt';

VERBOSE = false;
PARALLEL = false;
N_TRIALS = 10;

% Run identifier (used in directory naming)
[~, base_name, ~] = fileparts(TRAJECTORY_FILE);
run_label = sprintf('%s_%d', base_name, N_TRIALS);  

% Monte Carlo options
mc_options = struct();
mc_options.N_trials = N_TRIALS;         % Number of trials (use 500+ for real studies)
mc_options.parallel = PARALLEL;         % Use parallel processing
mc_options.verbose = VERBOSE;           % Show progress

% Optional: Define custom parameter uncertainties
% If left commented, simulate_monte_carlo uses sensible defaults
%
% params_nominal = quadrotor_linear_6dof();
% perturb_config = struct();
% perturb_config.params = {
%     'm',   'normal', params_nominal.m, params_nominal.m * 0.10;
%     'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.10;
%     'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.10;
%     'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.10;
%     'L',   'normal', params_nominal.L, params_nominal.L * 0.005;
% };

perturb_config = [];  % Use defaults

fprintf('\n');
fprintf('================================================================\n');
fprintf('QUADROTOR ROBUSTNESS STUDY WORKFLOW\n');
fprintf('================================================================\n');
fprintf('Trajectory:  %s\n', TRAJECTORY_FILE);
fprintf('Run Label:   %s\n', run_label);
fprintf('Trials:      %d\n', mc_options.N_trials);
fprintf('================================================================\n\n');

%% ========================================================================
%% STEP 1: GENERATE DATA (DO THIS ONCE)
%% ========================================================================

fprintf('STEP 1: DATA GENERATION\n');
fprintf('----------------------------------------------------------------\n');
fprintf('Running Monte Carlo simulation...\n');
fprintf('This may take a while (several minutes to hours)\n\n');

% Run simulation - returns results directory path
results_dir = simulate_monte_carlo(TRAJECTORY_FILE, run_label, perturb_config, mc_options);

fprintf('\n');
fprintf('================================================================\n');
fprintf('DATA GENERATION COMPLETE!\n');
fprintf('================================================================\n');
fprintf('Results saved to: %s\n', results_dir);
fprintf('================================================================\n\n');

%% ========================================================================
%% STEP 2: REVIEW METRICS
%% ========================================================================

fprintf('STEP 2: REVIEW METRICS\n');
fprintf('----------------------------------------------------------------\n');

% Find the metrics file that was just created
metrics_pattern = fullfile(results_dir, 'metrics_*.txt');
metrics_files = dir(metrics_pattern);

if ~isempty(metrics_files)
    % Get the file (should only be one)
    metrics_file = fullfile(results_dir, metrics_files(1).name);
    
    fprintf('Metrics saved to: %s\n', metrics_file);
    fprintf('\nTo view full metrics:\n');
    fprintf('  >> edit %s\n', metrics_file);
    fprintf('  or in terminal: cat %s\n', metrics_file);
    
    % Display preview
    fprintf('\nMetrics Preview:\n');
    fprintf('----------------------------------------------------------------\n');
    fid = fopen(metrics_file, 'r');
    for i = 1:20  % Show first 20 lines
        line = fgetl(fid);
        if line == -1, break; end
        fprintf('%s\n', line);
    end
    fclose(fid);
    fprintf('  [...see full file for complete metrics...]\n');
    fprintf('----------------------------------------------------------------\n\n');
else
    warning('Could not find metrics file');
end

%% ========================================================================
%% STEP 3: GENERATE FIGURES
%% ========================================================================

fprintf('STEP 3: GENERATE FIGURES\n');
fprintf('----------------------------------------------------------------\n');
fprintf('Generating all publication figures from saved data...\n');
fprintf('This is fast (<1 minute) and can be repeated easily.\n\n');

% Figure generation options
fig_options.close_figures = true;  % Close figures after saving
fig_options.verbose = true;

% Generate figures - uses results_dir directly!
generate_paper_figures(results_dir, fig_options);

fprintf('\n');

%% ========================================================================
%% STEP 4: EXAMINE RESULTS
%% ========================================================================

fprintf('STEP 4: EXAMINE RESULTS\n');
fprintf('----------------------------------------------------------------\n');
fprintf('All files saved successfully!\n\n');

figures_dir = fullfile(results_dir, 'figures');

fprintf('Output locations:\n');
fprintf('  Results:  %s\n', results_dir);
fprintf('  Figures:  %s\n', figures_dir);
fprintf('\n');

fprintf('Key files:\n');
fprintf('  Nominal data:    %s\n', fullfile(results_dir, 'nominal.mat'));
fprintf('  MC data:         %s\n', fullfile(results_dir, 'monte_carlo.mat'));
fprintf('  Metrics summary: %s\n', metrics_file);
fprintf('  Paper metrics:   %s\n', fullfile(figures_dir, 'paper_metrics.txt'));
fprintf('\n');

% Load and display key metrics
fprintf('Quick metrics preview:\n');
fprintf('----------------------------------------------------------------\n');
nominal_file = fullfile(results_dir, 'nominal.mat');
if exist(nominal_file, 'file')
    nominal = DataManager.load_results(nominal_file);
    if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
        fprintf('Nominal Performance:\n');
        fprintf('  Position RMSE:  %.4f m\n', nominal.metrics.tracking.rmse_position);
        fprintf('  Attitude RMSE:  %.2f deg\n', rad2deg(nominal.metrics.tracking.rmse_attitude));
    end
end

mc_file = fullfile(results_dir, 'monte_carlo.mat');
if exist(mc_file, 'file')
    mc = DataManager.load_results(mc_file, struct('validate', false, 'migrate', false));
    fprintf('\nMonte Carlo Results:\n');
    fprintf('  Success Rate:   %.1f%%\n', mc.statistics.success_rate);
    if isfield(mc.statistics, 'metrics')
        fprintf('  Position RMSE:  %.4f ± %.4f m\n', ...
                mc.statistics.metrics.rmse_position_mean, ...
                mc.statistics.metrics.rmse_position_std);
        fprintf('  Attitude RMSE:  %.2f ± %.2f deg\n', ...
                rad2deg(mc.statistics.metrics.rmse_attitude_mean), ...
                rad2deg(mc.statistics.metrics.rmse_attitude_std));
    end
end
fprintf('----------------------------------------------------------------\n\n');

%% ========================================================================
%% STEP 5: NEXT STEPS
%% ========================================================================

fprintf('NEXT STEPS\n');
fprintf('----------------------------------------------------------------\n');
fprintf('✓ Data generated and saved\n');
fprintf('✓ Figures created\n');
fprintf('✓ Metrics computed\n');
fprintf('\n');
fprintf('To regenerate figures with different styling:\n');
fprintf('  >> generate_paper_figures(''%s'')\n', results_dir);
fprintf('\n');
fprintf('To view figures:\n');
fprintf('  >> cd %s\n', figures_dir);
fprintf('  >> open tracking_3d.png  (or use your preferred image viewer)\n');
fprintf('\n');
fprintf('================================================================\n');
fprintf('WORKFLOW COMPLETE!\n');
fprintf('================================================================\n\n');

%% Clean up
fclose('all');