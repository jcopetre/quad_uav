% Demonstrates complete workflow
%
% It separates data generation (expensive) from figure generation
% (fast), allowing you to iterate on visualizations without rerunning 4-hour
% Monte Carlo simulations.
%
% WORKFLOW:
%   1. Run this script to generate data (do once, takes hours)
%   2. Generate figures from saved data (do many times, takes seconds)
%   3. Iterate on figure appearance without regenerating data
%

clear; clc; close all;
init_project();

%% ========================================================================
%% CONFIGURATION
%% ========================================================================

% Choose trajectory
TRAJECTORY_FILE = 'simple_square.wpt';

mc_options = struct();
mc_options.N_trials = 2000;
mc_options.parallel = true;
mc_options.verbose = false;

% Define custom parameter uncertainties
% params_nominal = quadrotor_linear_6dof();

% perturb_config = struct();
% perturb_config.params = {
%     % Higher mass uncertainty (±20% instead of ±10%)
%     'm',   'normal', params_nominal.m, params_nominal.m * 0.10;
% 
%     % Larger inertia uncertainty
%     'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.10;
%     'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.10;
%     'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.10;
% 
%     % Standard arm length uncertainty
%     'L',   'normal', params_nominal.L, params_nominal.L * 0.005;
% };

fprintf('Trajectory:    %s\n', TRAJECTORY_FILE);
fprintf('=================================================================\n\n');

%% ========================================================================
%% STEP 1: GENERATE DATA (DO THIS ONCE)
%% ========================================================================

fprintf('STEP 1: DATA GENERATION\n');
fprintf('-----------------------------------------------------------------\n');

fprintf('Running uncertainty study...\n');
fprintf('Trials: %d\n', mc_options.N_trials);

[~, base_name, ~] = fileparts(TRAJECTORY_FILE);
run_label = sprintf('%s_%d', base_name, mc_options.N_trials);
simulate_monte_carlo(TRAJECTORY_FILE, run_label, [], mc_options);

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