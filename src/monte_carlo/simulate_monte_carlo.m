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
    % Write human-readable metrics summary with complete configuration
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not create metrics file: %s', filename);
    end
    
    try
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'MONTE CARLO ROBUSTNESS ANALYSIS - COMPREHENSIVE REPORT\n');
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'Generated: %s\n', datestr(now));
        fprintf(fid, '=================================================================\n\n');
        
        %% ============================================================
        %% CONFIGURATION SECTION (For Reproducibility)
        %% ============================================================
        fprintf(fid, 'CONFIGURATION (For Reproducibility)\n');
        fprintf(fid, '-----------------------------------------------------------------\n\n');
        
        % Trajectory information
        fprintf(fid, 'Trajectory:\n');
        fprintf(fid, '  File:              %s\n', mc_results.config.trajectory_file);
        fprintf(fid, '  Duration:          %.2f s\n', nominal.trajectory.time(end));
        fprintf(fid, '  Time Step (dt):    %.4f s\n', mean(diff(nominal.trajectory.time)));
        
        % Trajectory generation method
        if isfield(nominal.trajectory, 'method')
            fprintf(fid, '  Generation Method: %s\n', nominal.trajectory.method);
            if isfield(nominal.trajectory, 'method_reason')
                fprintf(fid, '    Reason:          %s\n', nominal.trajectory.method_reason);
            end
            if isfield(nominal.trajectory, 'selection_criteria')
                sc = nominal.trajectory.selection_criteria;
                if isfield(sc, 'min_segment')
                    fprintf(fid, '    Min Segment:     %.2f s\n', sc.min_segment);
                end
                if isfield(sc, 'threshold')
                    fprintf(fid, '    Threshold:       %.2f s\n', sc.threshold);
                end
            end
        end
        fprintf(fid, '\n');
        
        % Vehicle parameters
        params = nominal.params;
        fprintf(fid, 'Vehicle Parameters:\n');
        fprintf(fid, '  Mass (m):          %.4f kg\n', params.m);
        fprintf(fid, '  Arm Length (L):    %.4f m\n', params.L);
        fprintf(fid, '  Inertia:           Ixx=%.6f, Iyy=%.6f, Izz=%.6f kg·m²\n', ...
                params.Ixx, params.Iyy, params.Izz);
        fprintf(fid, '  Gravity:           %.2f m/s²\n', params.g);
        fprintf(fid, '\n');
        
        % Actuator limits
        fprintf(fid, 'Actuator Limits:\n');
        fprintf(fid, '  Thrust:            [%.2f, %.2f] N\n', params.u_min(1), params.u_max(1));
        fprintf(fid, '  Torques:           [%.3f, %.3f] N·m\n', params.u_min(2), params.u_max(2));
        fprintf(fid, '\n');
        
        % Controller design
        fprintf(fid, 'LQR Controller Design:\n');
        Q_diag = diag(params.Q);
        fprintf(fid, '  State Weights (Q):\n');
        fprintf(fid, '    Position:        [%.1f, %.1f, %.1f]\n', Q_diag(1), Q_diag(2), Q_diag(3));
        fprintf(fid, '    Attitude:        [%.1f, %.1f, %.1f]\n', Q_diag(4), Q_diag(5), Q_diag(6));
        fprintf(fid, '    Velocity:        [%.1f, %.1f, %.1f]\n', Q_diag(7), Q_diag(8), Q_diag(9));
        fprintf(fid, '    Angular Rate:    [%.1f, %.1f, %.1f]\n', Q_diag(10), Q_diag(11), Q_diag(12));
        
        R_diag = diag(params.R);
        fprintf(fid, '  Control Weights (R): [%.2f, %.2f, %.2f, %.2f]\n', ...
                R_diag(1), R_diag(2), R_diag(3), R_diag(4));
        
        fprintf(fid, '  Hover Thrust:      %.4f N\n', params.u_hover(1));
        fprintf(fid, '  Closed-Loop Poles:\n');
        fprintf(fid, '    Range:           [%.2f, %.2f] (real part)\n', ...
                min(real(params.poles)), max(real(params.poles)));
        
        % Stability check (no ternary)
        if all(real(params.poles) < 0)
            fprintf(fid, '    All Stable:      YES\n');
        else
            fprintf(fid, '    All Stable:      NO\n');
        end
        fprintf(fid, '\n');
        
        % Monte Carlo configuration
        fprintf(fid, 'Monte Carlo Configuration:\n');
        fprintf(fid, '  Trials:            %d\n', mc_results.config.mc_options.N_trials);
        fprintf(fid, '  Random Seed:       %d\n', mc_results.config.mc_options.seed);
        
        % Parallel check (no ternary)
        if mc_results.config.mc_options.parallel
            fprintf(fid, '  Parallel:          YES\n');
        else
            fprintf(fid, '  Parallel:          NO\n');
        end
        fprintf(fid, '\n');
        
        % Parameter perturbations
        fprintf(fid, 'Parameter Perturbations:\n');
        perturb = mc_results.config.perturb_config.params;
        for i = 1:size(perturb, 1)
            param_name = perturb{i, 1};
            dist_type = perturb{i, 2};
            param1 = perturb{i, 3};
            param2 = perturb{i, 4};
            
            switch lower(dist_type)
                case 'normal'
                    pct = 100 * param2 / param1;
                    fprintf(fid, '  %-6s: %s(μ=%.4f, σ=%.4f) → ±%.1f%% std\n', ...
                            param_name, dist_type, param1, param2, pct);
                case 'uniform'
                    fprintf(fid, '  %-6s: %s[%.4f, %.4f]\n', ...
                            param_name, dist_type, param1, param2);
                case 'fixed'
                    fprintf(fid, '  %-6s: %s = %.4f\n', ...
                            param_name, dist_type, param1);
            end
        end
        fprintf(fid, '\n');
        fprintf(fid, '=================================================================\n\n');
        
        %% ============================================================
        %% RESULTS SECTION
        %% ============================================================
        
        % Overall statistics
        fprintf(fid, 'OVERALL STATISTICS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Total trials:      %d\n', mc_results.statistics.n_trials);
        fprintf(fid, 'Successful:        %d (%.1f%%)\n', ...
                mc_results.statistics.n_success, mc_results.statistics.success_rate);
        fprintf(fid, 'Failed:            %d (%.1f%%)\n\n', ...
                mc_results.statistics.n_failed, ...
                100 * mc_results.statistics.n_failed / mc_results.statistics.n_trials);
        fprintf(fid, 'Elapsed time:      %.2f seconds\n', mc_results.elapsed_time);
        fprintf(fid, 'Time per trial:    %.3f seconds\n\n', ...
                mc_results.elapsed_time / mc_results.statistics.n_trials);
        
        % Nominal performance
        fprintf(fid, 'NOMINAL PERFORMANCE (Baseline)\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            fprintf(fid, 'Position RMSE:     %.4f m\n', nominal.metrics.tracking.rmse_position);
            fprintf(fid, 'Attitude RMSE:     %.4f rad (%.2f deg)\n', ...
                    nominal.metrics.tracking.rmse_attitude, ...
                    rad2deg(nominal.metrics.tracking.rmse_attitude));
            
            if isfield(nominal.metrics.tracking, 'max_position_error')
                fprintf(fid, 'Max Position Err:  %.4f m\n', nominal.metrics.tracking.max_position_error);
            end
            if isfield(nominal.metrics, 'control') && isfield(nominal.metrics.control, 'total_effort')
                fprintf(fid, 'Control Effort:    %.2f\n', nominal.metrics.control.total_effort);
            end
        end
        fprintf(fid, '\n');
        
        % Monte Carlo performance
        if isfield(mc_results.statistics, 'metrics')
            m = mc_results.statistics.metrics;
            fprintf(fid, 'MONTE CARLO PERFORMANCE (Successful Trials)\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            fprintf(fid, 'Position RMSE:     %.4f ± %.4f m (mean ± std)\n', ...
                    m.rmse_position_mean, m.rmse_position_std);
            fprintf(fid, 'Attitude RMSE:     %.4f ± %.4f rad (%.2f ± %.2f deg)\n', ...
                    m.rmse_attitude_mean, m.rmse_attitude_std, ...
                    rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
            
            fprintf(fid, '\nPercentiles:\n');
            fprintf(fid, '  5th:             %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(1), rad2deg(m.rmse_attitude_percentiles(1)));
            fprintf(fid, '  25th:            %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(2), rad2deg(m.rmse_attitude_percentiles(2)));
            fprintf(fid, '  50th (median):   %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_median, rad2deg(m.rmse_attitude_median));
            fprintf(fid, '  75th:            %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(3), rad2deg(m.rmse_attitude_percentiles(3)));
            fprintf(fid, '  95th:            %.4f m (position), %.2f deg (attitude)\n', ...
                    m.rmse_position_percentiles(4), rad2deg(m.rmse_attitude_percentiles(4)));
        end
        
        fprintf(fid, '\n=================================================================\n');
        fprintf(fid, 'END OF REPORT\n');
        fprintf(fid, '=================================================================\n');
        
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end

function str = ternary(condition, true_val, false_val)
    % Simple ternary operator helper
    if condition
        str = true_val;
    else
        str = false_val;
    end
end