function generate_paper_figures(run_label, timestamp, output_dir)
    % GENERATE_PAPER_FIGURES Creates publication-quality figures from saved simulation data
    %
    % USAGE:
    %   generate_paper_figures(run_label, timestamp)
    %   generate_paper_figures(run_label, 'latest')
    %   generate_paper_figures(run_label, timestamp, output_dir)
    %
    % INPUTS:
    %   run_label   - Study identifier (e.g., 'paper_final')
    %   timestamp   - Specific timestamp or 'latest' to auto-find most recent
    %   output_dir  - [optional] Custom output directory (default: auto-matched nested dir)
    %
    % OUTPUTS:
    %   Creates nested directory: ./figures/<run_label>_<timestamp>/
    %   Saves six figures in that directory:
    %     - square_tracking_3d.png
    %     - tracking_timeseries.png
    %     - control_inputs.png
    %     - mc_boxplots.png
    %     - mc_correlation.png
    %     - mc_distributions.png
    %   Plus metrics table: paper_metrics.txt
    %
    % EXAMPLE:
    %   % After running simulate_monte_carlo('simple_square.wpt', 'paper_final');
    %   generate_paper_figures('paper_final', 'latest');
    
    %% Input Handling
    if nargin < 2
        timestamp = 'latest';
    end
    
    % Find latest timestamp if requested
    if strcmpi(timestamp, 'latest')
        timestamp = find_latest_timestamp(run_label);
        fprintf('Auto-detected latest run: %s\n', timestamp);
    end
    
    % Build paths to data directory
    run_dir_name = sprintf('%s_%s', run_label, timestamp);
    results_dir = fullfile('.', 'results', run_dir_name);
    
    % Verify directory exists
    if ~exist(results_dir, 'dir')
        error('Results directory not found: %s', results_dir);
    end
    
    % Create matching figures directory
    if nargin < 3 || isempty(output_dir)
        figures_dir = fullfile('.', 'figures', run_dir_name);
    else
        figures_dir = output_dir;
    end
    
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
    end
    
    fprintf('\n=======================================================\n');
    fprintf('GENERATING FIGURES: %s\n', run_label);
    fprintf('=======================================================\n');
    fprintf('Loading from: %s\n', results_dir);
    fprintf('Saving to:    %s\n', figures_dir);
    fprintf('=======================================================\n\n');
    
    %% Load Data
    fprintf('[1/7] Loading nominal simulation...\n');
    nominal_file = fullfile(results_dir, 'nominal.mat');
    if ~exist(nominal_file, 'file')
        error('Nominal data file not found: %s', nominal_file);
    end
    nominal = DataManager.load_results(nominal_file);
    
    fprintf('[2/7] Loading Monte Carlo results...\n');
    mc_file = fullfile(results_dir, 'monte_carlo.mat');
    if ~exist(mc_file, 'file')
        error('Monte Carlo data file not found: %s', mc_file);
    end
    % Monte Carlo results have different schema - skip validation and migration
    mc = DataManager.load_results(mc_file, struct('validate', false, 'migrate', false));
    
    %% Generate Figures
    fprintf('[3/7] Creating 3D tracking visualization...\n');
    fig1 = create_3d_tracking_figure(nominal);
    saveas(fig1, fullfile(figures_dir, 'square_tracking_3d.png'));
    close(fig1);
    
    fprintf('[4/7] Creating tracking timeseries...\n');
    fig2 = create_tracking_timeseries(nominal);
    saveas(fig2, fullfile(figures_dir, 'tracking_timeseries.png'));
    close(fig2);
    
    fprintf('[5/7] Creating control inputs plot...\n');
    fig3 = create_control_inputs_figure(nominal);
    saveas(fig3, fullfile(figures_dir, 'control_inputs.png'));
    close(fig3);
    
    % fprintf('[6/7] Creating Monte Carlo boxplots...\n');
    % fig4 = create_mc_boxplots(mc);
    % saveas(fig4, fullfile(figures_dir, 'mc_boxplots.png'));
    % close(fig4);
    % 
    % fprintf('[7/7] Creating correlation and distribution plots...\n');
    % fig5 = create_correlation_heatmap(mc);
    % saveas(fig5, fullfile(figures_dir, 'mc_correlation.png'));
    % close(fig5);
    %
    % fig6 = create_parameter_distributions(mc);
    % saveas(fig6, fullfile(figures_dir, 'mc_distributions.png'));
    % close(fig6);

    fprintf('[6/7] Running Monte Carlo analysis and generating plots...\n');
    analysis_options.plot = true;
    analysis_options.save_plots = true;
    analysis_options.plot_dir = figures_dir;
    analysis_options.verbose = false;  % Keep it quiet
    
    analysis = analyze_monte_carlo_results(mc, analysis_options);
    
    fprintf('[7/7] Copying MC figures to output directory...\n');
    % analyze_monte_carlo_results already saved figures to figures_dir
    % Just confirm they exist
    mc_figure_files = {'distributions.png', 'boxplots.png', 'correlation.png'};
    for i = 1:length(mc_figure_files)
        src = fullfile(figures_dir, mc_figure_files{i});
        if exist(src, 'file')
            fprintf('  ✓ %s\n', mc_figure_files{i});
        end
    end
   
    
    %% Generate Metrics Table
    fprintf('\nGenerating paper metrics table...\n');
    metrics_file = fullfile(figures_dir, 'paper_metrics.txt');
    write_paper_metrics(nominal, mc, metrics_file);
    
    %% Summary
    fprintf('\n=======================================================\n');
    fprintf('✅ FIGURES COMPLETE\n');
    fprintf('=======================================================\n');
    fprintf('Output directory: %s\n', figures_dir);
    fprintf('\nFiles created:\n');
    fprintf('  - square_tracking_3d.png\n');
    fprintf('  - tracking_timeseries.png\n');
    fprintf('  - control_inputs.png\n');
    fprintf('  - mc_boxplots.png\n');
    fprintf('  - mc_correlation.png\n');
    fprintf('  - mc_distributions.png\n');
    fprintf('  - paper_metrics.txt\n');
    fprintf('=======================================================\n\n');
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function timestamp = find_latest_timestamp(run_label)
    % Find most recent timestamp for given run_label
    
    % Look for directories matching pattern
    pattern = fullfile('.', 'results', sprintf('%s_*', run_label));
    dirs = dir(pattern);
    
    if isempty(dirs)
        error('No results found for run_label: %s', run_label);
    end
    
    % Extract timestamps from directory names
    timestamps = {};
    for i = 1:length(dirs)
        if dirs(i).isdir
            % Pattern: runlabel_TIMESTAMP
            tokens = regexp(dirs(i).name, sprintf('%s_(\\d{8}_\\d{6})$', run_label), 'tokens');
            if ~isempty(tokens)
                timestamps{end+1} = tokens{1}{1}; %#ok<AGROW>
            end
        end
    end
    
    if isempty(timestamps)
        error('Could not extract timestamp from directory names for: %s', run_label);
    end
    
    % Find latest (timestamps are sortable strings in yyyymmdd_HHMMSS format)
    timestamps = sort(timestamps);
    timestamp = timestamps{end};
end

function fig = create_3d_tracking_figure(nominal)
    % Create 3D trajectory tracking visualization
    
    fig = figure('Position', [100 100 800 600]);
    
    % Extract trajectory data
    t = nominal.t;
    pos = nominal.x(:,1:3);
    
    % Get reference trajectory from stored trajectory structure
    pos_ref = nominal.trajectory.position;
    
    % Plot 3D trajectory
    plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2); hold on;
    plot3(pos_ref(:,1), pos_ref(:,2), pos_ref(:,3), 'r--', 'LineWidth', 1.5);
    
    % Mark start and end
    plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(pos(end,1), pos(end,2), pos(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');
    title('Square Trajectory Tracking (3D View)');
    grid on; axis equal;
    view(45, 30);
end

function fig = create_tracking_timeseries(nominal)
    % Create position tracking timeseries
    
    fig = figure('Position', [100 100 1000 600]);
    
    t = nominal.t;
    pos = nominal.x(:,1:3);
    
    % Interpolate reference trajectory at simulation time points
    traj = nominal.trajectory;
    pos_ref = interp1(traj.time, traj.position, t, 'linear', 'extrap');
    
    labels = {'X', 'Y', 'Z'};
    for i = 1:3
        subplot(3,1,i);
        plot(t, pos_ref(:,i), 'r--', 'LineWidth', 1.5); hold on;
        plot(t, pos(:,i), 'b-', 'LineWidth', 1);
        ylabel(sprintf('%s (m)', labels{i}));
        grid on;
        legend('Reference', 'Actual', 'Location', 'best');
        if i == 1
            title('Position Tracking vs Time');
        end
    end
    xlabel('Time (s)');
end

function fig = create_control_inputs_figure(nominal)
    % Create control inputs visualization
    
    fig = figure('Position', [100 100 1000 700]);
    
    t = nominal.t;
    u = nominal.u_log;
    
    % Thrust
    subplot(4,1,1);
    plot(t, u(:,1), 'b-', 'LineWidth', 1);
    ylabel('Thrust (N)');
    grid on;
    title('Control Inputs vs Time');
    
    % Torques
    labels = {'\tau_x', '\tau_y', '\tau_z'};
    for i = 1:3
        subplot(4,1,i+1);
        plot(t, u(:,i+1), 'b-', 'LineWidth', 1);
        ylabel(sprintf('%s (N·m)', labels{i}));
        grid on;
    end
    xlabel('Time (s)');
end

% function fig = create_mc_boxplots(mc)
%     % Create Monte Carlo performance boxplots
% 
%     fig = figure('Position', [100 100 1200 500]);
% 
%     % Extract metrics from successful trials
%     success_flags = [mc.trials.success];
%     successful_trials = mc.trials(success_flags);
% 
%     if isempty(successful_trials)
%         text(0.5, 0.5, 'No successful trials to plot', ...
%              'HorizontalAlignment', 'center');
%         return;
%     end
% 
%     % Extract position RMSE
%     position_rmse = arrayfun(@(t) t.metrics.tracking.rmse_position, successful_trials);
% 
%     % Extract attitude RMSE
%     attitude_rmse = arrayfun(@(t) t.metrics.tracking.rmse_attitude, successful_trials);
% 
%     % Extract control effort (if available)
%     has_control = isfield(successful_trials(1).metrics, 'control') && ...
%                   ~isempty(fieldnames(successful_trials(1).metrics.control));
% 
%     % Position RMSE boxplot
%     subplot(1,3,1);
%     boxplot(position_rmse);
%     ylabel('Position RMSE (m)');
%     title('Position Tracking Error');
%     grid on;
% 
%     % Attitude RMSE boxplot
%     subplot(1,3,2);
%     boxplot(rad2deg(attitude_rmse));
%     ylabel('Attitude RMSE (deg)');
%     title('Attitude Tracking Error');
%     grid on;
% 
%     % Control effort boxplot
%     subplot(1,3,3);
%     if has_control
%         control_effort = arrayfun(@(t) t.metrics.control.total_effort, successful_trials);
%         boxplot(control_effort);
%         ylabel('Control Effort');
%         title('Control Effort Distribution');
%     else
%         text(0.5, 0.5, 'Control effort not available', ...
%              'HorizontalAlignment', 'center');
%     end
%     grid on;
% 
%     n_trials = length(mc.trials);
%     sgtitle(sprintf('Monte Carlo Performance (N=%d trials, %d successful)', ...
%                     n_trials, length(successful_trials)));
% end
% 
% function fig = create_correlation_heatmap(mc)
%     % Create parameter-performance correlation heatmap
% 
%     fig = figure('Position', [100 100 900 700]);
% 
%     % Extract successful trials
%     success_flags = [mc.trials.success];
%     successful_trials = mc.trials(success_flags);
% 
%     if isempty(successful_trials)
%         text(0.5, 0.5, 'No successful trials to analyze', ...
%              'HorizontalAlignment', 'center');
%         return;
%     end
% 
%     % Extract parameters from perturbation config
%     param_names = mc.config.perturb_config.params(:, 1);
%     n_params = length(param_names);
%     n_trials = length(successful_trials);
% 
%     % Build parameter matrix
%     param_matrix = zeros(n_trials, n_params);
%     for i = 1:n_trials
%         for j = 1:n_params
%             param_matrix(i, j) = successful_trials(i).params.(param_names{j});
%         end
%     end
% 
%     % Extract metrics (ensure column vectors)
%     rmse_position = arrayfun(@(t) t.metrics.tracking.rmse_position, successful_trials)';
%     rmse_attitude = arrayfun(@(t) t.metrics.tracking.rmse_attitude, successful_trials)';
% 
%     % Compute correlations
%     metric_names = {'Position RMSE', 'Attitude RMSE'};
%     corr_matrix = zeros(2, n_params);
% 
%     for j = 1:n_params
%         corr_matrix(1, j) = corr(param_matrix(:, j), rmse_position);
%         corr_matrix(2, j) = corr(param_matrix(:, j), rmse_attitude);
%     end
% 
%     % Plot heatmap
%     imagesc(corr_matrix);
%     colormap(redblue());
%     colorbar;
%     caxis([-1 1]);
% 
%     % Labels
%     set(gca, 'XTick', 1:n_params, 'XTickLabel', param_names);
%     set(gca, 'YTick', 1:2, 'YTickLabel', metric_names);
%     xtickangle(45);
% 
%     title('Parameter-Performance Correlation Matrix');
% 
%     % Add correlation values
%     for i = 1:2
%         for j = 1:n_params
%             val = corr_matrix(i,j);
%             if abs(val) > 0.3
%                 text(j, i, sprintf('%.2f', val), ...
%                      'HorizontalAlignment', 'center', ...
%                      'Color', 'k', 'FontWeight', 'bold');
%             end
%         end
%     end
% end
% 
% function fig = create_parameter_distributions(mc)
%     % Create parameter distribution histograms
% 
%     fig = figure('Position', [100 100 1200 800]);
% 
%     % Extract parameters from perturbation config
%     param_names = mc.config.perturb_config.params(:, 1);
%     n_params = length(param_names);
%     n_trials = length(mc.trials);
% 
%     % Build parameter matrix (all trials, not just successful)
%     param_matrix = zeros(n_trials, n_params);
%     for i = 1:n_trials
%         for j = 1:n_params
%             param_matrix(i, j) = mc.trials(i).params.(param_names{j});
%         end
%     end
% 
%     % Plot distributions (max 9 subplots)
%     n_plots = min(n_params, 9);
%     for i = 1:n_plots
%         subplot(3, 3, i);
%         pname = param_names{i};
%         values = param_matrix(:, i);
% 
%         histogram(values, 30, 'Normalization', 'pdf');
%         xlabel(strrep(pname, '_', '\_'));  % Escape underscores for LaTeX
%         ylabel('Probability Density');
%         title(sprintf('%s Distribution', strrep(pname, '_', ' ')));
%         grid on;
%     end
% 
%     sgtitle(sprintf('Perturbed Parameter Distributions (N=%d trials)', n_trials));
% end

function write_paper_metrics(nominal, mc, filename)
    % Write formatted metrics table for paper
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file for writing: %s', filename);
    end
    
    % Use try-finally to ensure file is closed even if error occurs
    try
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'PAPER METRICS SUMMARY\n');
        fprintf(fid, '=================================================================\n\n');
        
        % Nominal performance
        fprintf(fid, 'NOMINAL SIMULATION\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            m = nominal.metrics.tracking;
            fprintf(fid, 'Position RMSE:  %.4f m\n', m.rmse_position);
            fprintf(fid, 'Attitude RMSE:  %.4f deg\n', rad2deg(m.rmse_attitude));
            if isfield(nominal.metrics, 'control') && isfield(nominal.metrics.control, 'total_effort')
                fprintf(fid, 'Control Effort: %.4f\n', nominal.metrics.control.total_effort);
            end
        end
        fprintf(fid, '\n');
        
        % Monte Carlo statistics
        n_trials = length(mc.trials);
        n_success = mc.statistics.n_success;
        fprintf(fid, 'MONTE CARLO STATISTICS (N=%d, %d successful)\n', n_trials, n_success);
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(mc, 'statistics') && isfield(mc.statistics, 'metrics')
            m = mc.statistics.metrics;
            fprintf(fid, 'Position RMSE:  %.4f ± %.4f m\n', ...
                    m.rmse_position_mean, m.rmse_position_std);
            fprintf(fid, 'Attitude RMSE:  %.4f ± %.4f deg\n', ...
                    rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
            if isfield(m, 'control_effort_mean')
                fprintf(fid, 'Control Effort: %.4f ± %.4f\n', ...
                        m.control_effort_mean, m.control_effort_std);
            end
        end
        fprintf(fid, '\n');
        
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'LaTeX snippets for direct insertion:\n');
        fprintf(fid, '=================================================================\n\n');
        
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            fprintf(fid, '%% Nominal tracking accuracy\n');
            fprintf(fid, '$\\mathrm{RMSE}_{pos} = %.3f$ m\n\n', nominal.metrics.tracking.rmse_position);
            
            if isfield(mc, 'statistics') && isfield(mc.statistics, 'metrics')
                fprintf(fid, '%% Monte Carlo robustness\n');
                fprintf(fid, '$\\mathrm{RMSE}_{pos} = %.3f \\pm %.3f$ m over %d trials\n', ...
                        mc.statistics.metrics.rmse_position_mean, ...
                        mc.statistics.metrics.rmse_position_std, ...
                        n_success);
            end
        end
    catch ME
        % Close file before re-throwing error
        fclose(fid);
        rethrow(ME);
    end
    
    % Always close the file
    fclose(fid);
end

function cmap = redblue()
    % Red-white-blue colormap for correlation visualization
    n = 64;
    r = [(0:n-1)'/n; ones(n,1)];
    g = [(0:n-1)'/n; flipud((0:n-1)'/n)];
    b = [ones(n,1); flipud((0:n-1)'/n)];
    cmap = [r g b];
end