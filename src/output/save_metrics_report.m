function save_metrics_report(nominal_results, mc_results, output_file)
    % SAVE_METRICS_REPORT Writes comprehensive metrics report to disk
    %
    % USAGE:
    %   save_metrics_report(nominal_results, mc_results, output_file)
    %
    % INPUTS:
    %   nominal_results - Struct from simulate_trajectory()
    %   mc_results      - Struct from run_monte_carlo()
    %   output_file     - Full path to output text file (e.g., './results/study_1/metrics.txt')
    %
    % This function is typically called by simulate_monte_carlo() but can
    % be used standalone if you have the data structures.
    
    %% Open file
    fid = fopen(output_file, 'w');
    if fid == -1
        error('Could not open file for writing: %s', output_file);
    end
    
    try
        %% Header
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'SIMULATION METRICS REPORT\n');
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'Generated: %s\n', datestr(now));
        fprintf(fid, '=================================================================\n\n');
        
        %% Nominal Simulation Metrics
        fprintf(fid, 'NOMINAL SIMULATION\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        
        if isfield(nominal_results, 'metrics')
            metrics = nominal_results.metrics;
            
            % Position tracking
            if isfield(metrics, 'position_rmse')
                fprintf(fid, 'Position RMSE:       %.6f m\n', metrics.position_rmse);
            end
            if isfield(metrics, 'position_mae')
                fprintf(fid, 'Position MAE:        %.6f m\n', metrics.position_mae);
            end
            if isfield(metrics, 'position_max_error')
                fprintf(fid, 'Position Max Error:  %.6f m\n', metrics.position_max_error);
            end
            
            fprintf(fid, '\n');
            
            % Attitude tracking
            if isfield(metrics, 'attitude_rmse')
                fprintf(fid, 'Attitude RMSE:       %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse, rad2deg(metrics.attitude_rmse));
            end
            if isfield(metrics, 'attitude_mae')
                fprintf(fid, 'Attitude MAE:        %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_mae, rad2deg(metrics.attitude_mae));
            end
            if isfield(metrics, 'attitude_max_error')
                fprintf(fid, 'Attitude Max Error:  %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_max_error, rad2deg(metrics.attitude_max_error));
            end
            
            fprintf(fid, '\n');
            
            % Control metrics
            if isfield(metrics, 'control_effort')
                fprintf(fid, 'Control Effort:      %.6f\n', metrics.control_effort);
            end
            if isfield(metrics, 'mean_thrust')
                fprintf(fid, 'Mean Thrust:         %.6f N\n', metrics.mean_thrust);
            end
            if isfield(metrics, 'mean_torque_mag')
                fprintf(fid, 'Mean Torque (mag):   %.6f N·m\n', metrics.mean_torque_mag);
            end
            if isfield(metrics, 'max_thrust')
                fprintf(fid, 'Max Thrust:          %.6f N\n', metrics.max_thrust);
            end
            if isfield(metrics, 'max_torque_mag')
                fprintf(fid, 'Max Torque (mag):    %.6f N·m\n', metrics.max_torque_mag);
            end
            
            % Saturation metrics
            if isfield(metrics, 'thrust_saturation_pct')
                fprintf(fid, 'Thrust Saturation:   %.2f%%\n', metrics.thrust_saturation_pct);
            end
            if isfield(metrics, 'torque_saturation_pct')
                fprintf(fid, 'Torque Saturation:   %.2f%%\n', metrics.torque_saturation_pct);
            end
        else
            fprintf(fid, 'No metrics available for nominal simulation.\n');
        end
        
        fprintf(fid, '\n');
        
        %% Monte Carlo Statistics
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'MONTE CARLO ANALYSIS\n');
        fprintf(fid, '=================================================================\n');
        
        if isfield(mc_results, 'config')
            fprintf(fid, 'N_trials:            %d\n', mc_results.config.mc_options.N_trials);
            fprintf(fid, 'Parallel:            %s\n', mat2str(mc_results.config.mc_options.parallel));
            fprintf(fid, '\n');
        end
        
        if isfield(mc_results, 'metrics')
            metrics = mc_results.metrics;
            
            % Success rate
            if isfield(metrics, 'success_rate')
                fprintf(fid, 'Success Rate:        %.2f%%\n', metrics.success_rate * 100);
            end
            
            fprintf(fid, '\n');
            
            % Position statistics
            fprintf(fid, 'POSITION RMSE STATISTICS\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            if isfield(metrics, 'position_rmse_mean')
                fprintf(fid, 'Mean:                %.6f m\n', metrics.position_rmse_mean);
            end
            if isfield(metrics, 'position_rmse_std')
                fprintf(fid, 'Std Dev:             %.6f m\n', metrics.position_rmse_std);
            end
            if isfield(metrics, 'position_rmse_min')
                fprintf(fid, 'Min:                 %.6f m\n', metrics.position_rmse_min);
            end
            if isfield(metrics, 'position_rmse_max')
                fprintf(fid, 'Max:                 %.6f m\n', metrics.position_rmse_max);
            end
            if isfield(metrics, 'position_rmse_median')
                fprintf(fid, 'Median:              %.6f m\n', metrics.position_rmse_median);
            end
            
            fprintf(fid, '\n');
            
            % Attitude statistics
            fprintf(fid, 'ATTITUDE RMSE STATISTICS\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            if isfield(metrics, 'attitude_rmse_mean')
                fprintf(fid, 'Mean:                %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse_mean, rad2deg(metrics.attitude_rmse_mean));
            end
            if isfield(metrics, 'attitude_rmse_std')
                fprintf(fid, 'Std Dev:             %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse_std, rad2deg(metrics.attitude_rmse_std));
            end
            if isfield(metrics, 'attitude_rmse_min')
                fprintf(fid, 'Min:                 %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse_min, rad2deg(metrics.attitude_rmse_min));
            end
            if isfield(metrics, 'attitude_rmse_max')
                fprintf(fid, 'Max:                 %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse_max, rad2deg(metrics.attitude_rmse_max));
            end
            if isfield(metrics, 'attitude_rmse_median')
                fprintf(fid, 'Median:              %.6f rad (%.4f deg)\n', ...
                        metrics.attitude_rmse_median, rad2deg(metrics.attitude_rmse_median));
            end
            
            fprintf(fid, '\n');
            
            % Control effort statistics
            if isfield(metrics, 'control_effort_mean')
                fprintf(fid, 'CONTROL EFFORT STATISTICS\n');
                fprintf(fid, '-----------------------------------------------------------------\n');
                fprintf(fid, 'Mean:                %.6f\n', metrics.control_effort_mean);
                if isfield(metrics, 'control_effort_std')
                    fprintf(fid, 'Std Dev:             %.6f\n', metrics.control_effort_std);
                end
                if isfield(metrics, 'control_effort_min')
                    fprintf(fid, 'Min:                 %.6f\n', metrics.control_effort_min);
                end
                if isfield(metrics, 'control_effort_max')
                    fprintf(fid, 'Max:                 %.6f\n', metrics.control_effort_max);
                end
                fprintf(fid, '\n');
            end
        else
            fprintf(fid, 'No metrics available for Monte Carlo analysis.\n\n');
        end
        
        %% Sensitivity Analysis
        if isfield(mc_results, 'sensitivity')
            fprintf(fid, '=================================================================\n');
            fprintf(fid, 'SENSITIVITY ANALYSIS\n');
            fprintf(fid, '=================================================================\n');
            
            sens = mc_results.sensitivity;
            
            if isfield(sens, 'corr_matrix')
                fprintf(fid, '\nCORRELATION MATRIX (Parameter vs Performance)\n');
                fprintf(fid, '-----------------------------------------------------------------\n');
                
                % Print header
                fprintf(fid, '%-15s', 'Parameter');
                for j = 1:length(sens.metric_names)
                    fprintf(fid, '%12s', sens.metric_names{j});
                end
                fprintf(fid, '\n');
                fprintf(fid, '%s\n', repmat('-', 1, 70));
                
                % Print correlations
                for i = 1:length(sens.param_names)
                    fprintf(fid, '%-15s', sens.param_names{i});
                    for j = 1:length(sens.metric_names)
                        fprintf(fid, '%12.4f', sens.corr_matrix(j,i));
                    end
                    fprintf(fid, '\n');
                end
                
                fprintf(fid, '\n');
                
                % Highlight strongest correlations
                fprintf(fid, 'STRONGEST CORRELATIONS (|r| > 0.3)\n');
                fprintf(fid, '-----------------------------------------------------------------\n');
                for i = 1:size(sens.corr_matrix, 1)
                    for j = 1:size(sens.corr_matrix, 2)
                        r = sens.corr_matrix(i,j);
                        if abs(r) > 0.3
                            fprintf(fid, '%s ↔ %s: r = %.4f\n', ...
                                    sens.param_names{j}, sens.metric_names{i}, r);
                        end
                    end
                end
            end
            
            fprintf(fid, '\n');
        end
        
        %% Footer
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'END OF REPORT\n');
        fprintf(fid, '=================================================================\n');
        
        fclose(fid);
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end