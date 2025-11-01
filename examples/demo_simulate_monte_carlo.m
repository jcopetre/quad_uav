% Demonstrates various methods of how to call simulate_monte_carlo
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
fprintf('DATA GENERATION EXAMPLE\n');
fprintf('=================================================================\n');
fprintf('Configuration: %s\n', RUN_TYPE);
fprintf('Trajectory:    %s\n', TRAJECTORY_FILE);
fprintf('=================================================================\n\n');

fprintf('-----------------------------------------------------------------\n');

switch lower(RUN_TYPE)
    case 'test'
        % Quick test study - good for testing
        run_label = 'test';
        
        % Use defaults (50 trials, nominal uncertainties)
        fprintf('Running test study with default settings...\n');
        
        mc_options = struct();
        mc_options.N_trials = 2000;     
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
