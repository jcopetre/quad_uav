function results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options)
% SIMULATE_MONTE_CARLO - High-level Monte Carlo orchestrator with organized output
%
% Runs Monte Carlo robustness analysis and saves all results to a single
% organized directory structure.
%
% SYNTAX:
%   results_dir = simulate_monte_carlo(trajectory_file, run_label)
%   results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config)
%   results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options)
%
% INPUTS:
%   trajectory_file - Waypoint file (e.g., 'basic_maneuver.wpt')
%   run_label       - Identifier for this study (e.g., 'paper_final', 'square_10')
%   perturb_config  - (optional) Parameter perturbation configuration
%                     If empty, uses default operational uncertainties
%   mc_options      - (optional) Monte Carlo options struct
%
% OUTPUTS:
%   results_dir     - Path to results directory containing all outputs
%
% DIRECTORY STRUCTURE:
%   ./results/<run_label>_<timestamp>/
%   ├── nominal.mat              % Nominal simulation results
%   ├── monte_carlo.mat          % MC results structure
%   ├── metrics_<timestamp>.txt  % Human-readable metrics
%   └── figures/                 % (created later by generate_paper_figures)
%       ├── tracking_3d.png
%       ├── tracking_timeseries.png
%       ├── control_inputs.png
%       ├── attitude_dynamics.png
%       ├── distributions.png
%       ├── boxplots.png
%       ├── correlation.png
%       └── paper_metrics.txt
%
% EXAMPLE:
%   % Run study
%   results_dir = simulate_monte_carlo('simple_square.wpt', 'paper_final');
%   
%   % Later, generate figures
%   generate_paper_outputs(results_dir);
%
% See also: run_monte_carlo, generate_paper_figures

% Author: Trey Copeland
% Date: 2025-11-01 (Refactored for unified directory structure)

    init_project();
    
    %% Input validation
    if nargin < 2
        error('Must provide trajectory_file and run_label');
    end
    
    if nargin < 3 || isempty(perturb_config)
        % Use default operational uncertainties
        perturb_config = get_default_perturbations();
    end
    
    if nargin < 4 || isempty(mc_options)
        mc_options = struct();
    end
    
    % Set MC defaults using standardized helper
    mc_defaults = struct(...
        'N_trials', 500, ...
        'seed', 42, ...
        'parallel', true, ...
        'verbose', true ...
    );
    mc_options = set_default_options(mc_options, mc_defaults);
    
    %% Create results directory
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    run_dir_name = sprintf('%s_%s', run_label, timestamp);
    results_dir = fullfile('./results', run_dir_name);
    
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    if mc_options.verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('MONTE CARLO ROBUSTNESS STUDY\n');
        fprintf('================================================================\n');
        fprintf('Run Label:    %s\n', run_label);
        fprintf('Trajectory:   %s\n', trajectory_file);
        fprintf('Trials:       %d\n', mc_options.N_trials);
        fprintf('Results dir:  %s\n', results_dir);
        fprintf('================================================================\n\n');
    end
    
    %% Run Monte Carlo analysis
    mc_results = run_monte_carlo(trajectory_file, perturb_config, mc_options);

    %% Save nominal simulation
    if mc_options.verbose
        fprintf('Saving nominal simulation...\n');
    end
    nominal = mc_results.nominal.results;
    DataManager.save_results(nominal, 'nominal', results_dir, ...
       struct('timestamp', false, 'validate', true, 'verbose', mc_options.verbose));
    
    %% Save Monte Carlo results
    if mc_options.verbose
        fprintf('Saving Monte Carlo results...\n');
    end
    DataManager.save_monte_carlo(mc_results, results_dir, ...
        struct('validate', true, 'verbose', mc_options.verbose));
    
    %% Generate metrics file
    if mc_options.verbose
        fprintf('Writing metrics summary...\n');
    end
    metrics_file = fullfile(results_dir, sprintf('metrics_%s.txt', timestamp));
    write_metrics_summary(mc_results, nominal, metrics_file);
    
    %% Summary
    if mc_options.verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('MONTE CARLO COMPLETE\n');
        fprintf('================================================================\n');
        fprintf('Results saved to: %s\n', results_dir);
        fprintf('\nNext steps:\n');
        fprintf('  1. Review metrics: edit %s\n', metrics_file);
        fprintf('  2. Generate figures: generate_paper_outputs(''%s'')\n', results_dir);
        fprintf('================================================================\n\n');
    end
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function perturb_config = get_default_perturbations()
    % Default operational uncertainties
    
    params_nominal = quadrotor_linear_6dof();
    
    perturb_config.params = {
        % Mass: ±10% (payload changes, battery depletion)
        'm',   'normal', params_nominal.m, params_nominal.m * 0.05;
        
        % Inertia: ±15% (payload distribution, manufacturing)
        'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.075;
        'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.075;
        'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.075;
        
        % Arm length: ±1% (manufacturing tolerance)
        'L',   'normal', params_nominal.L, params_nominal.L * 0.005;
    };
end

function write_metrics_summary(mc_results, nominal, filename)
    % Write human-readable metrics summary
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not create metrics file: %s', filename);
    end
    
    try
        fprintf(fid, '================================================================\n');
        fprintf(fid, 'MONTE CARLO ROBUSTNESS STUDY - METRICS SUMMARY\n');
        fprintf(fid, '================================================================\n');
        fprintf(fid, 'Generated: %s\n', datestr(now));
        fprintf(fid, 'Trajectory: %s\n', mc_results.config.trajectory_file);
        fprintf(fid, '================================================================\n\n');
        
        % Overall statistics
        fprintf(fid, 'OVERALL STATISTICS\n');
        fprintf(fid, '------------------\n');
        fprintf(fid, 'Total trials:     %d\n', mc_results.statistics.n_trials);
        fprintf(fid, 'Successful:       %d (%.1f%%)\n', ...
                mc_results.statistics.n_success, mc_results.statistics.success_rate);
        fprintf(fid, 'Failed:           %d (%.1f%%)\n\n', ...
                mc_results.statistics.n_failed, ...
                100 * mc_results.statistics.n_failed / mc_results.statistics.n_trials);
        fprintf(fid, 'Elapsed time:     %.2f seconds\n', mc_results.elapsed_time);
        fprintf(fid, 'Time per trial:   %.3f seconds\n\n', ...
                mc_results.elapsed_time / mc_results.statistics.n_trials);
        
        % Nominal performance
        fprintf(fid, 'NOMINAL PERFORMANCE\n');
        fprintf(fid, '-------------------\n');
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            fprintf(fid, 'Position RMSE:    %.4f m\n', nominal.metrics.tracking.rmse_position);
            fprintf(fid, 'Attitude RMSE:    %.4f rad (%.2f deg)\n', ...
                    nominal.metrics.tracking.rmse_attitude, ...
                    rad2deg(nominal.metrics.tracking.rmse_attitude));
        end
        fprintf(fid, '\n');
        
        % Monte Carlo performance
        if isfield(mc_results.statistics, 'metrics')
            m = mc_results.statistics.metrics;
            fprintf(fid, 'MONTE CARLO PERFORMANCE (Successful Trials)\n');
            fprintf(fid, '-------------------------------------------\n');
            fprintf(fid, 'Position RMSE:    %.4f ± %.4f m (mean ± std)\n', ...
                    m.rmse_position_mean, m.rmse_position_std);
            fprintf(fid, 'Attitude RMSE:    %.4f ± %.4f rad (%.2f ± %.2f deg)\n', ...
                    m.rmse_attitude_mean, m.rmse_attitude_std, ...
                    rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
            
            fprintf(fid, '\nPercentiles:\n');
            fprintf(fid, '  5th:  %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(1), rad2deg(m.rmse_attitude_percentiles(1)));
            fprintf(fid, '  25th: %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(2), rad2deg(m.rmse_attitude_percentiles(2)));
            fprintf(fid, '  75th: %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(3), rad2deg(m.rmse_attitude_percentiles(3)));
            fprintf(fid, '  95th: %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(4), rad2deg(m.rmse_attitude_percentiles(4)));
        end
        
        fprintf(fid, '\n================================================================\n');
        fprintf(fid, 'END OF SUMMARY\n');
        fprintf(fid, '================================================================\n');
        
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end