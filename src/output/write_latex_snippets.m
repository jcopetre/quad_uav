function write_latex_snippets(nominal, mc_results, output_file)
    % WRITE_LATEX_SNIPPETS - Generate copy-paste LaTeX for paper
    %
    
    fid = fopen(output_file, 'w');
    
    try
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'LATEX SNIPPETS FOR PAPER\n');
        fprintf(fid, '=================================================================\n\n');
        
        % Abstract / Introduction
        fprintf(fid, '%% ABSTRACT / INTRODUCTION\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            fprintf(fid, 'Nominal tracking accuracy:\n');
            fprintf(fid, '$\\mathrm{RMSE}_{pos} = %.3f$ m, $\\mathrm{RMSE}_{att} = %.2f$ deg\n\n', ...
                    nominal.metrics.tracking.rmse_position, ...
                    rad2deg(nominal.metrics.tracking.rmse_attitude));
        end
        
        n_trials = mc_results.statistics.n_trials;
        success_rate = mc_results.statistics.success_rate;
        fprintf(fid, 'Monte Carlo validation: $N = %d$ trials with $%.1f\\%%$ success rate\n\n', ...
                n_trials, success_rate);
        
        % Results Section
        fprintf(fid, '%% RESULTS SECTION\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        
        if isfield(mc_results.statistics, 'metrics')
            m = mc_results.statistics.metrics;
            
            fprintf(fid, '%% Performance under parameter uncertainty:\n');
            fprintf(fid, 'Position tracking: $%.3f \\pm %.3f$ m (mean $\\pm$ std)\n\n', ...
                    m.rmse_position_mean, m.rmse_position_std);
            fprintf(fid, 'Attitude tracking: $%.2f \\pm %.2f$ deg (mean $\\pm$ std)\n\n', ...
                    rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
            
            % (Continue with table generation as in lines 730-753)
            % ...
        end
        
        fprintf(fid, '=================================================================\n');
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end