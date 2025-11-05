function write_unified_metrics_report(results, mc_results, output_file)
    % WRITE_UNIFIED_METRICS_REPORT - Single comprehensive metrics file
    %
    % Works for BOTH single-run and Monte Carlo contexts
    %
    % INPUTS:
    %   results    - Results struct from simulate_trajectory
    %                Must have: .trajectory, .params, .metrics, .t, .x, .u_log
    %   mc_results - (optional) MC results struct, or [] for single-run
    %   output_file - Path to output .txt file
    %
    % OUTPUT FILE STRUCTURE:
    %   1. Header & Timestamp
    %   2. Configuration (trajectory, vehicle, controller)
    %   3. Performance Results (single-run or nominal baseline)
    %   4. Monte Carlo Statistics (if mc_results provided)
    %   5. Sensitivity Analysis (if mc_results provided)
    %   6. Footer
    %
    % EXAMPLE:
    %   % Single-run context
    %   results = simulate_trajectory('simple_square.wpt');
    %   write_unified_metrics_report(results, [], './metrics.txt');
    %
    %   % Monte Carlo context
    %   mc = simulate_monte_carlo('simple_square.wpt', 'study1');
    %   nominal = load(fullfile(results_dir, Constants.NOMINAL_DATA));
    %   write_unified_metrics_report(nominal, mc, './metrics.txt');
    
    % Detect context
    has_mc = ~isempty(mc_results) && isstruct(mc_results);
    
    % Open file
    fid = fopen(output_file, 'w');
    if fid == -1
        error('Cannot create metrics file: %s', output_file);
    end
    
    try
        % Write all sections
        write_header(fid, has_mc);
        write_configuration(fid, results);
        write_trajectory_feasibility(fid, results);
        write_performance(fid, results, has_mc);
        
        if has_mc
            write_mc_statistics(fid, mc_results);
            write_sensitivity(fid, mc_results);
    
            % Trajectory feasibility (if tracked)
            if isfield(mc_results.statistics, 'trajectory_feasibility')
                fprintf(fid, '\nMONTE CARLO TRAJECTORY FEASIBILITY\n');
                fprintf(fid, '-----------------------------------------------------------------\n');
                fprintf(fid, 'Trials with feasibility warnings: %d / %d (%.1f%%)\n', ...
                        mc_results.statistics.trajectory_feasibility.n_infeasible, ...
                        mc_results.statistics.n_trials, ...
                        mc_results.statistics.trajectory_feasibility.pct_infeasible);
                fprintf(fid, '\n');
            end
        end
        
        write_footer(fid);
        
        fclose(fid);
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function write_header(fid, has_mc)
    % Write report header
    
    fprintf(fid, '=================================================================\n');
    if has_mc
        fprintf(fid, 'MONTE CARLO ROBUSTNESS ANALYSIS - COMPREHENSIVE REPORT\n');
    else
        fprintf(fid, 'TRAJECTORY SIMULATION - PERFORMANCE REPORT\n');
    end
    fprintf(fid, '=================================================================\n');
    fprintf(fid, 'Generated: %s\n', datestr(now));
    fprintf(fid, '=================================================================\n\n');
end

function write_configuration(fid, results)
    % Write configuration section
    % EXTRACTED FROM: simulate_monte_carlo.m lines 175-279
    
    fprintf(fid, 'CONFIGURATION\n');
    fprintf(fid, '-----------------------------------------------------------------\n\n');
    
    %% Trajectory Information
    fprintf(fid, 'Trajectory:\n');
    
    % Try to get filename from trajectory metadata
    if isfield(results.trajectory, 'metadata') && isfield(results.trajectory.metadata, 'filename')
        fprintf(fid, '  File:              %s\n', results.trajectory.metadata.filename);
    elseif isfield(results.trajectory, 'filename')
        fprintf(fid, '  File:              %s\n', results.trajectory.filename);
    else
        fprintf(fid, '  File:              <programmatic>\n');
    end
    
    fprintf(fid, '  Duration:          %.2f s\n', results.trajectory.time(end));
    fprintf(fid, '  Time Step (dt):    %.4f s\n', mean(diff(results.trajectory.time)));
    
    % Trajectory generation method (if available)
    if isfield(results.trajectory, 'method')
        fprintf(fid, '  Generation Method: %s\n', results.trajectory.method);
        if isfield(results.trajectory, 'method_reason')
            fprintf(fid, '    Reason:          %s\n', results.trajectory.method_reason);
        end
        if isfield(results.trajectory, 'selection_criteria')
            sc = results.trajectory.selection_criteria;
            if isfield(sc, 'min_segment')
                fprintf(fid, '    Min Segment:     %.2f s\n', sc.min_segment);
            end
            if isfield(sc, 'threshold')
                fprintf(fid, '    Threshold:       %.2f s\n', sc.threshold);
            end
        end
    end
    fprintf(fid, '\n');
    
    %% Vehicle Parameters
    params = results.params;
    fprintf(fid, 'Vehicle Parameters:\n');
    fprintf(fid, '  Mass (m):          %.4f kg\n', params.m);
    fprintf(fid, '  Arm Length (L):    %.4f m\n', params.L);
    fprintf(fid, '  Inertia:           Ixx=%.6f, Iyy=%.6f, Izz=%.6f kg·m²\n', ...
            params.Ixx, params.Iyy, params.Izz);
    fprintf(fid, '  Gravity:           %.2f m/s²\n', params.g);
    fprintf(fid, '\n');
    
    %% Actuator Limits
    fprintf(fid, 'Actuator Limits:\n');
    fprintf(fid, '  Thrust:            [%.2f, %.2f] N\n', params.u_min(1), params.u_max(1));
    
    % Check if torque limits are symmetric
    if params.u_max(2) == params.u_max(3)
        fprintf(fid, '  Roll/Pitch Torque: [%.3f, %.3f] N·m\n', params.u_min(2), params.u_max(2));
    else
        fprintf(fid, '  Roll Torque:       [%.3f, %.3f] N·m\n', params.u_min(2), params.u_max(2));
        fprintf(fid, '  Pitch Torque:      [%.3f, %.3f] N·m\n', params.u_min(3), params.u_max(3));
    end
    fprintf(fid, '  Yaw Torque:        [%.3f, %.3f] N·m\n', params.u_min(4), params.u_max(4));
    fprintf(fid, '\n');
    
    %% Controller Design
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
    fprintf(fid, '    Count:           %d\n', length(params.poles));
    fprintf(fid, '    Range:           [%.2f, %.2f] (real part)\n', ...
            min(real(params.poles)), max(real(params.poles)));
    
    % Sort poles for display
    [~, idx] = sort(real(params.poles), 'ascend');
    poles_sorted = params.poles(idx);
    fprintf(fid, '    Slowest 3:       [%.2f, %.2f, %.2f]\n', real(poles_sorted(1:3)));
    fprintf(fid, '    Fastest 3:       [%.2f, %.2f, %.2f]\n', real(poles_sorted(end-2:end)));
    
    if all(real(params.poles) < 0)
        fprintf(fid, '    All Stable:      YES\n');
    else
        fprintf(fid, '    All Stable:      NO\n');
    end
    fprintf(fid, '\n');
end

function write_performance(fid, results, has_mc)
    % Write performance results section
    % EXTRACTED FROM: simulate_monte_carlo.m lines 300-314
    %                 generate_standard_outputs.m lines 610-673
    
    if has_mc
        fprintf(fid, 'NOMINAL SIMULATION (Baseline)\n');
    else
        fprintf(fid, 'SIMULATION RESULTS\n');
    end
    fprintf(fid, '-----------------------------------------------------------------\n');
    
    if ~isfield(results, 'metrics')
        fprintf(fid, 'No metrics available.\n\n');
        return;
    end
    
    metrics = results.metrics;
    
    %% Position Tracking
    if isfield(metrics, 'tracking')
        fprintf(fid, 'Position RMSE:       %.4f m\n', metrics.tracking.rmse_position);
        if isfield(metrics.tracking, 'max_position_error')
            fprintf(fid, 'Max Position Error:  %.4f m\n', metrics.tracking.max_position_error);
        end
    end
    
    %% Attitude Tracking
    if isfield(metrics, 'tracking')
        fprintf(fid, 'Attitude RMSE:       %.4f rad (%.2f deg)\n', ...
                metrics.tracking.rmse_attitude, ...
                rad2deg(metrics.tracking.rmse_attitude));
        
        if isfield(metrics.tracking, 'max_roll')
            fprintf(fid, 'Max Roll:            %.4f rad (%.2f deg)\n', ...
                    metrics.tracking.max_roll, rad2deg(metrics.tracking.max_roll));
        end
        if isfield(metrics.tracking, 'max_pitch')
            fprintf(fid, 'Max Pitch:           %.4f rad (%.2f deg)\n', ...
                    metrics.tracking.max_pitch, rad2deg(metrics.tracking.max_pitch));
        end
    end
    
    fprintf(fid, '\n');
    
    %% Control Performance
    if isfield(metrics, 'control') && ~isempty(fieldnames(metrics.control))
        fprintf(fid, 'Control Performance:\n');
        
        % RMS values (physically meaningful)
        if isfield(metrics.control, 'rms_thrust')
            fprintf(fid, '  RMS Thrust:        %.4f N\n', metrics.control.rms_thrust);
        end
        if isfield(metrics.control, 'rms_roll_torque')
            fprintf(fid, '  RMS Roll Torque:   %.4f N·m\n', metrics.control.rms_roll_torque);
            fprintf(fid, '  RMS Pitch Torque:  %.4f N·m\n', metrics.control.rms_pitch_torque);
            fprintf(fid, '  RMS Yaw Torque:    %.4f N·m\n', metrics.control.rms_yaw_torque);
        end
        
        fprintf(fid, '\n');
        
        % Peak values
        if isfield(metrics.control, 'max_thrust')
            fprintf(fid, '  Peak Thrust:       %.4f N\n', metrics.control.max_thrust);
        end
        if isfield(metrics.control, 'max_torque')
            fprintf(fid, '  Peak Torque:       %.4f N·m\n', metrics.control.max_torque);
        end
        
        fprintf(fid, '\n');
        
        % Saturation
        if isfield(metrics.control, 'thrust_saturation_pct')
            fprintf(fid, '  Thrust Sat.:       %.1f%%\n', metrics.control.thrust_saturation_pct);
        end
        if isfield(metrics.control, 'torque_saturation_pct')
            fprintf(fid, '  Torque Sat.:       %.1f%%\n', metrics.control.torque_saturation_pct);
        end
        
        fprintf(fid, '\n');
        
        % Total effort (with clear documentation)
        if isfield(metrics.control, 'total_effort')
            fprintf(fid, '  Total Effort:      %.4f\n', metrics.control.total_effort);
            if isfield(metrics.control, 'total_effort_units')
                fprintf(fid, '    Units:           %s\n', metrics.control.total_effort_units);
            end
            if isfield(metrics.control, 'total_effort_note')
                fprintf(fid, '    Note:            %s\n', metrics.control.total_effort_note);
            end
        end
    end
    
    fprintf(fid, '\n');
end

function write_mc_statistics(fid, mc_results)
    % Write Monte Carlo statistics section
    % EXTRACTED FROM: simulate_monte_carlo.m lines 286-339
    %                 generate_standard_outputs.m lines 625-690
    
    fprintf(fid, 'MONTE CARLO ROBUSTNESS ANALYSIS\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    
    %% MC Configuration
    if isfield(mc_results, 'config')
        fprintf(fid, 'Total Trials:        %d\n', mc_results.config.mc_options.N_trials);
        if mc_results.config.mc_options.parallel
            fprintf(fid, 'Parallel:            YES\n');
        else
            fprintf(fid, 'Parallel:            NO\n');
        end
        fprintf(fid, 'Random Seed:         %d\n', mc_results.config.mc_options.seed);
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
    end
    
    %% Overall Statistics
    fprintf(fid, 'Successful:          %d (%.1f%%)\n', ...
            mc_results.statistics.n_success, mc_results.statistics.success_rate);
    fprintf(fid, 'Failed:              %d (%.1f%%)\n', ...
            mc_results.statistics.n_failed, ...
            100 * mc_results.statistics.n_failed / mc_results.statistics.n_trials);
    
    if isfield(mc_results, 'elapsed_time')
        fprintf(fid, 'Computation Time:    %.1f seconds (%.2f sec/trial)\n', ...
                mc_results.elapsed_time, ...
                mc_results.elapsed_time / mc_results.statistics.n_trials);
    end
    fprintf(fid, '\n');
    
    %% Performance Statistics
    if isfield(mc_results.statistics, 'metrics')
        fprintf(fid, 'PERFORMANCE STATISTICS (Successful Trials)\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        
        m = mc_results.statistics.metrics;
        
        % Position RMSE
        fprintf(fid, 'Position RMSE:\n');
        fprintf(fid, '  Mean ± Std:        %.4f ± %.4f m\n', ...
                m.rmse_position_mean, m.rmse_position_std);
        fprintf(fid, '  Median:            %.4f m\n', m.rmse_position_median);
        fprintf(fid, '  Percentiles:\n');
        fprintf(fid, '    5th:             %.4f m\n', m.rmse_position_percentiles(1));
        fprintf(fid, '    25th:            %.4f m\n', m.rmse_position_percentiles(2));
        fprintf(fid, '    75th:            %.4f m\n', m.rmse_position_percentiles(3));
        fprintf(fid, '    95th:            %.4f m\n', m.rmse_position_percentiles(4));
        fprintf(fid, '\n');
        
        % Attitude RMSE
        fprintf(fid, 'Attitude RMSE:\n');
        fprintf(fid, '  Mean ± Std:        %.4f ± %.4f rad (%.2f ± %.2f deg)\n', ...
                m.rmse_attitude_mean, m.rmse_attitude_std, ...
                rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
        fprintf(fid, '  Median:            %.4f rad (%.2f deg)\n', ...
                m.rmse_attitude_median, rad2deg(m.rmse_attitude_median));
        fprintf(fid, '  Percentiles:\n');
        fprintf(fid, '    5th:             %.4f rad (%.2f deg)\n', ...
                m.rmse_attitude_percentiles(1), rad2deg(m.rmse_attitude_percentiles(1)));
        fprintf(fid, '    95th:            %.4f rad (%.2f deg)\n', ...
                m.rmse_attitude_percentiles(4), rad2deg(m.rmse_attitude_percentiles(4)));
        fprintf(fid, '\n');
        
        % Control effort (if available)
        if isfield(m, 'control_effort_mean')
            fprintf(fid, 'Control Effort:\n');
            fprintf(fid, '  Mean ± Std:        %.4f ± %.4f [N²·s + (N·m)²·s, mixed units]\n', ...
                    m.control_effort_mean, m.control_effort_std);
            fprintf(fid, '  Note:              For comparison only; dominated by thrust\n');
            fprintf(fid, '\n');
        end
    end
end

function write_sensitivity(fid, mc_results)
    % Write sensitivity analysis section
    % EXTRACTED FROM: generate_standard_outputs.m lines 634-673
    
    % Check if we have analysis results
    if ~isfield(mc_results, 'statistics') || ~isfield(mc_results.statistics, 'correlations')
        fprintf(fid, 'SENSITIVITY ANALYSIS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Correlation analysis not available.\n');
        fprintf(fid, 'Run analyze_monte_carlo() to compute correlations.\n\n');
        return;
    end
    
    fprintf(fid, 'PARAMETER CORRELATION ANALYSIS\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    fprintf(fid, '(Pearson correlation coefficients: -1 to +1)\n');
    fprintf(fid, '(Positive = parameter increase worsens performance)\n\n');
    
    % Get correlation data
    corr_data = mc_results.statistics.correlations;
    param_names = mc_results.statistics.param_names;
    
    % Position RMSE correlations
    fprintf(fid, 'Correlation with Position RMSE:\n');
    [~, sorted_idx] = sort(abs(corr_data.rmse_position), 'descend');
    for i = 1:length(param_names)
        idx = sorted_idx(i);
        marker = '';
        if i == 1
            marker = '  ** STRONGEST **';
        end
        fprintf(fid, '  %8s:  %+.3f%s\n', param_names{idx}, ...
                corr_data.rmse_position(idx), marker);
    end
    fprintf(fid, '\n');
    
    % Attitude RMSE correlations
    fprintf(fid, 'Correlation with Attitude RMSE:\n');
    [~, sorted_idx] = sort(abs(corr_data.rmse_attitude), 'descend');
    for i = 1:length(param_names)
        idx = sorted_idx(i);
        marker = '';
        if i == 1
            marker = '  ** STRONGEST **';
        end
        fprintf(fid, '  %8s:  %+.3f%s\n', param_names{idx}, ...
                corr_data.rmse_attitude(idx), marker);
    end
    fprintf(fid, '\n');
    
    % Max position error correlations (if available)
    if isfield(corr_data, 'max_position_error')
        fprintf(fid, 'Correlation with Max Position Error:\n');
        [~, sorted_idx] = sort(abs(corr_data.max_position_error), 'descend');
        for i = 1:length(param_names)
            idx = sorted_idx(i);
            marker = '';
            if i == 1
                marker = '  ** STRONGEST **';
            end
            fprintf(fid, '  %8s:  %+.3f%s\n', param_names{idx}, ...
                    corr_data.max_position_error(idx), marker);
        end
        fprintf(fid, '\n');
    end
    
    %% Normalized Sensitivity (if available)
    if isfield(mc_results.statistics, 'normalized_sensitivity')
        fprintf(fid, 'NORMALIZED SENSITIVITY INDICES\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, '(Normalized by parameter variation range)\n');
        fprintf(fid, '(Higher absolute values = greater relative impact)\n\n');
        
        sens = mc_results.statistics.normalized_sensitivity;
        
        fprintf(fid, 'Position RMSE Sensitivity:\n');
        [~, sorted_idx] = sort(abs(sens.rmse_position), 'descend');
        for i = 1:length(param_names)
            idx = sorted_idx(i);
            marker = '';
            if i == 1
                marker = '  ** MOST SENSITIVE **';
            end
            fprintf(fid, '  %8s:  %+.3f%s\n', param_names{idx}, ...
                    sens.rmse_position(idx), marker);
        end
        fprintf(fid, '\n');
        
        fprintf(fid, 'Attitude RMSE Sensitivity:\n');
        [~, sorted_idx] = sort(abs(sens.rmse_attitude), 'descend');
        for i = 1:length(param_names)
            idx = sorted_idx(i);
            marker = '';
            if i == 1
                marker = '  ** MOST SENSITIVE **';
            end
            fprintf(fid, '  %8s:  %+.3f%s\n', param_names{idx}, ...
                    sens.rmse_attitude(idx), marker);
        end
        fprintf(fid, '\n');
    end
    
    %% Worst-case performance
    fprintf(fid, 'WORST-CASE PERFORMANCE\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    
    if isfield(mc_results.statistics, 'worst_case')
        wc = mc_results.statistics.worst_case;
        
        if isfield(wc, 'rmse_position_max')
            fprintf(fid, 'Worst Position RMSE:  %.4f m (successful trial #%d)\n', ...
                    wc.rmse_position_max, wc.worst_pos_trial);
        end
        if isfield(wc, 'rmse_attitude_max')
            fprintf(fid, 'Worst Attitude RMSE:  %.4f rad (%.2f deg) (successful trial #%d)\n', ...
                    wc.rmse_attitude_max, ...
                    rad2deg(wc.rmse_attitude_max), ...
                    wc.worst_att_trial);
        end
        if isfield(wc, 'max_position_error_max')
            fprintf(fid, 'Worst Max Error:      %.4f m\n', wc.max_position_error_max);
        end
        
        % Show parameter configuration for worst position RMSE case
        if isfield(wc, 'worst_pos_params') && isfield(mc_results.statistics, 'param_names')
            fprintf(fid, '\nWorst Position RMSE Configuration (Trial #%d):\n', wc.worst_pos_trial);
            param_names = mc_results.statistics.param_names;
            for i = 1:length(param_names)
                fprintf(fid, '  %8s: %.6f\n', param_names{i}, wc.worst_pos_params(i));
            end
        end
        
        % Show parameter configuration for worst attitude RMSE case (if different)
        if isfield(wc, 'worst_att_params') && wc.worst_att_trial ~= wc.worst_pos_trial
            fprintf(fid, '\nWorst Attitude RMSE Configuration (Trial #%d):\n', wc.worst_att_trial);
            param_names = mc_results.statistics.param_names;
            for i = 1:length(param_names)
                fprintf(fid, '  %8s: %.6f\n', param_names{i}, wc.worst_att_params(i));
            end
        end
    else
        fprintf(fid, 'Worst-case data not available.\n');
    end
    
    fprintf(fid, '\n');
end

function write_trajectory_feasibility(fid, results)
    % Write trajectory feasibility analysis
    
    if ~isfield(results.trajectory, 'feasibility')
        return;  % Not checked
    end
    
    fprintf(fid, '\nTRAJECTORY FEASIBILITY\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    
    feas = results.trajectory.feasibility;
    
    % Overall status
    if feas.feasible
        fprintf(fid, 'Status: ✅ All checks passed\n\n');
    else
        fprintf(fid, 'Status: ⚠️ %d warning(s) detected\n\n', length(feas.warnings));
    end
    
    % Key metrics
    v = feas.violations;
    fprintf(fid, 'Trajectory Demands:\n');
    fprintf(fid, '  Max Attitude:     %.1f° (limit: <%.1f°)\n', ...
            v.max_attitude, v.limits.small_angle);
    fprintf(fid, '  Max Velocity:     %.2f m/s (warning: >%.1f m/s)\n', ...
            v.max_velocity, v.limits.velocity_warning);
    fprintf(fid, '  Max Horiz Accel:  %.2f m/s² (safe: <%.2f m/s²)\n', ...
            v.max_horiz_accel, v.limits.accel_safe);
    fprintf(fid, '  Max Yaw Rate:     %.1f°/s (warning: >%.1f°/s)\n', ...
            v.max_yaw_rate, v.limits.yaw_rate_warning);
    fprintf(fid, '  RMS Yaw Rate:     %.1f°/s (limit: <20.0°/s)\n', v.rms_yaw_rate);
    fprintf(fid, '  Max Yaw Accel:    %.1f°/s² (physical: <%.1f°/s²)\n', ...
            v.max_yaw_accel, v.limits.yaw_accel_physical);
    fprintf(fid, '\n');
    
    % Warnings
    if ~feas.feasible
        fprintf(fid, 'Warnings:\n');
        for i = 1:length(feas.warnings)
            fprintf(fid, '  %d. %s\n', i, feas.warnings{i});
        end
        fprintf(fid, '\n');
    end
end

function write_footer(fid)
    % Write report footer
    
    fprintf(fid, '=================================================================\n');
    fprintf(fid, 'END OF METRICS REPORT\n');
    fprintf(fid, '=================================================================\n');
end