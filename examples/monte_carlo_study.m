% MONTE_CARLO_ROBUSTNESS_STUDY - Example Monte Carlo analysis
%
% Demonstrates how to set up and run a complete Monte Carlo robustness
% study with parameter uncertainties, parallel processing, and analysis.
%
% This script reproduces the pilot study results with operational
% uncertainties typical of quadrotor systems.

clear; clc; close all;
init_project();

%% Configuration
TRAJECTORY = 'simple_square.wpt';

%% Get nominal parameters to compute absolute uncertainties
params_nominal = quadrotor_linear_6dof();

%% Define Operational Uncertainties
% These represent realistic parameter variations due to:
% - Payload changes and battery depletion (mass)
% - Payload distribution and manufacturing tolerances (inertia)
% - Manufacturing tolerances (arm length)

perturb_config = struct();
perturb_config.params = {
    % Parameter, Type, Param1, Param2
    
    % Mass: ±10% (payload changes, battery depletion)
    'm', 'normal', params_nominal.m, params_nominal.m * 0.05;
    
    % Inertia: ±15% (payload distribution, manufacturing)
    'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.075;
    'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.075;
    'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.075;
    
    % Arm length: ±1% (manufacturing tolerance)
    'L', 'normal', params_nominal.L, params_nominal.L * 0.005;
};

%% Monte Carlo Configuration
mc_options = struct();
mc_options.N_trials = 500;      % Good statistical power for pilot study
mc_options.seed = 42;           % Reproducibility
mc_options.parallel = true;     % Use parallel processing if available
mc_options.verbose = true;      % Monitor progress

% Auto-generate save file with timestamp
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
mc_options.save_file = sprintf('./results/mc_pilot_study_%s.mat', timestamp);

%% Run Monte Carlo Study
fprintf('Starting Monte Carlo robustness analysis...\n\n');
mc_results = run_monte_carlo(TRAJECTORY, perturb_config, mc_options);

%% Analyze Results
fprintf('Analyzing results and generating plots...\n\n');
analysis = analyze_monte_carlo(mc_results);

%% Display Key Findings
fprintf('\n===========================================\n');
fprintf('KEY FINDINGS\n');
fprintf('===========================================\n');
fprintf('Most sensitive parameter: %s\n', analysis.sensitivity.most_sensitive);
fprintf('Success rate: %.1f%%\n', mc_results.statistics.success_rate);
fprintf('Mean position RMSE: %.4f m\n', mc_results.statistics.metrics.rmse_position_mean);
fprintf('===========================================\n\n');