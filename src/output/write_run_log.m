function write_run_log(mc_results, nominal, output_file)
    % WRITE_RUN_LOG - Record simulation configuration (no results analysis)
    %
    % Logs what was run, when, and with what settings.
    % Does NOT include results - see metrics_report.txt for that.
    
    fid = fopen(output_file, 'w');
    if fid == -1
        error('Cannot create run log: %s', output_file);
    end
    
    try
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'MONTE CARLO SIMULATION RUN LOG\n');
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'Generated: %s\n', datestr(now));
        fprintf(fid, '=================================================================\n\n');
        
        %% Trajectory
        fprintf(fid, 'TRAJECTORY\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(nominal.trajectory, 'metadata') && isfield(nominal.trajectory.metadata, 'filename')
            fprintf(fid, 'File:              %s\n', nominal.trajectory.metadata.filename);
        end
        fprintf(fid, 'Duration:          %.2f s\n', nominal.trajectory.time(end));
        fprintf(fid, 'Method:            %s\n', nominal.trajectory.method);
        fprintf(fid, '\n');
        
        %% Vehicle Parameters
        fprintf(fid, 'VEHICLE CONFIGURATION\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        params = nominal.params;
        fprintf(fid, 'Mass:              %.4f kg\n', params.m);
        fprintf(fid, 'Arm Length:        %.4f m\n', params.L);
        fprintf(fid, 'Inertia:           Ixx=%.6f, Iyy=%.6f, Izz=%.6f kg·m²\n', ...
                params.Ixx, params.Iyy, params.Izz);
        fprintf(fid, '\n');
        
        %% Monte Carlo Configuration
        fprintf(fid, 'MONTE CARLO CONFIGURATION\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Trials:            %d\n', mc_results.config.mc_options.N_trials);
        fprintf(fid, 'Random Seed:       %d\n', mc_results.config.mc_options.seed);
        fprintf(fid, 'Parallel:          %s\n', bool2str(mc_results.config.mc_options.parallel));
        fprintf(fid, '\n');
        
        %% Parameter Perturbations
        fprintf(fid, 'PARAMETER PERTURBATIONS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
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
            end
        end
        fprintf(fid, '\n');
        
        %% Execution Summary
        fprintf(fid, 'EXECUTION SUMMARY\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Success Rate:      %.1f%%\n', mc_results.statistics.success_rate);
        fprintf(fid, 'Failed Trials:     %d\n', mc_results.statistics.n_failed);
        fprintf(fid, 'Total Time:        %.1f seconds\n', mc_results.elapsed_time);
        fprintf(fid, 'Time per Trial:    %.2f seconds\n', ...
                mc_results.elapsed_time / mc_results.statistics.n_trials);
        fprintf(fid, '\n');
        
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'Data saved to:\n');
        fprintf(fid, '  - nominal.mat\n');
        fprintf(fid, '  - monte_carlo.mat\n');
        fprintf(fid, '\n');
        fprintf(fid, 'Next step:\n');
        fprintf(fid, '  Run generate_paper_outputs() to analyze results and generate\n');
        fprintf(fid, '  comprehensive metrics report with correlations and figures.\n');
        fprintf(fid, '=================================================================\n');
        
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end

function str = bool2str(val)
    if val, str = 'YES'; else, str = 'NO'; end
end