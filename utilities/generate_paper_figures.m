function generate_paper_figures(run_label, timestamp, output_dir, options)
    % GENERATE_PAPER_FIGURES Creates publication-quality figures from saved simulation data
    %
    % USAGE:
    %   generate_paper_figures(run_label, timestamp)
    %   generate_paper_figures(run_label, 'latest')
    %   generate_paper_figures(run_label, timestamp, output_dir)
    %   generate_paper_figures(run_label, timestamp, output_dir, options)
    %
    % INPUTS:
    %   run_label   - Study identifier (e.g., 'paper_final')
    %   timestamp   - Specific timestamp or 'latest' to auto-find most recent
    %   output_dir  - [optional] Custom output directory (default: auto-matched nested dir)
    %   options     - [optional] Struct with:
    %                 .close_figures - Close figures after saving (default: true)
    %                 .verbose       - Print progress messages (default: true)
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
    
    % Smart argument detection for 3rd and 4th arguments
    if nargin == 3
        % Check if 3rd argument is a struct (options) or string/char (output_dir)
        if isstruct(output_dir)
            % User passed options as 3rd argument
            options = output_dir;
            output_dir = [];  % Use default
        else
            % User passed output_dir as 3rd argument
            options = struct();
        end
    elseif nargin < 3
        output_dir = [];
        options = struct();
    elseif nargin < 4
        options = struct();
    end
    
    % Set default options
    if ~isfield(options, 'close_figures'), options.close_figures = true; end
    if ~isfield(options, 'verbose'), options.verbose = true; end
    
    % Find latest timestamp if requested (MUST happen before building paths)
    if strcmpi(timestamp, 'latest')
        timestamp = find_latest_timestamp(run_label);
        if options.verbose
            fprintf('Auto-detected latest run: %s\n', timestamp);
        end
    end
    
    % Build paths to data directory
    run_dir_name = sprintf('%s_%s', run_label, timestamp);
    results_dir = fullfile('.', 'results', run_dir_name);
    
    % Verify directory exists
    if ~exist(results_dir, 'dir')
        error('Results directory not found: %s\nAvailable directories:\n%s', ...
              results_dir, list_available_runs(run_label));
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
    if options.close_figures
        close(fig1);
    end
    
    fprintf('[4/7] Creating tracking timeseries...\n');
    fig2 = create_tracking_timeseries(nominal);
    saveas(fig2, fullfile(figures_dir, 'tracking_timeseries.png'));
    if options.close_figures
        close(fig2);
    end
    
    fprintf('[5/7] Creating control inputs plot...\n');
    fig3 = create_control_inputs_figure(nominal);
    saveas(fig3, fullfile(figures_dir, 'control_inputs.png'));
    if options.close_figures
        close(fig3);
    end
    
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
    analysis_options.verbose = false;
    analysis_options.close_figures = options.close_figures;
    
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
    write_paper_metrics(nominal, mc, analysis, metrics_file);
    
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
    
    % Add waypoints if available
    show_waypoints = false;
    if isfield(nominal, 'config') && isfield(nominal.config, 'trajectory_file')
        try
            % Load original waypoint file
            wpt = load_waypoints(nominal.config.trajectory_file);
            
            % Plot waypoint markers
            plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
                  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
            
            % Add labels with smart positioning
            if isfield(wpt, 'labels')
                n_wpt = length(wpt.time);
                
                % Check if start and end are at same location (closed loop)
                start_end_same = norm(wpt.position(1,:) - wpt.position(end,:)) < 0.01;
                
                for i = 1:n_wpt
                    if ~isempty(wpt.labels{i})
                        % Calculate offset direction (radial outward from trajectory center)
                        traj_center = mean(wpt.position, 1);
                        offset_dir = wpt.position(i,:) - traj_center;
                        offset_dir = offset_dir / (norm(offset_dir) + eps);  % Normalize
                        
                        % Scale offset based on trajectory size
                        traj_size = max(range(wpt.position));
                        offset_scale = traj_size * 0.08;  % 8% of trajectory range
                        
                        % Apply offset
                        label_pos = wpt.position(i,:) + offset_dir * offset_scale;
                        
                        % Special handling for start/end if they overlap
                        if start_end_same
                            if i == 1
                                % Start label - offset upward
                                label_pos(3) = label_pos(3) + offset_scale * 0.5;
                                label_text = sprintf('%s (start)', wpt.labels{i});
                            elseif i == n_wpt
                                % End label - offset downward
                                label_pos(3) = label_pos(3) - offset_scale * 0.5;
                                label_text = sprintf('%s (end)', wpt.labels{i});
                            else
                                label_text = wpt.labels{i};
                            end
                        else
                            label_text = wpt.labels{i};
                        end
                        
                        text(label_pos(1), label_pos(2), label_pos(3), label_text, ...
                             'FontSize', 9, 'FontWeight', 'bold', ...
                             'HorizontalAlignment', 'center');
                    end
                end
            end
            
            show_waypoints = true;
        catch ME
            % If waypoint loading fails, skip waypoint markers
            warning('Could not load waypoints: %s', ME.message);
        end
    end
    
    % Mark start and end
    plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot3(pos(end,1), pos(end,2), pos(end,3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    xlabel('X (m)', 'FontSize', 11); 
    ylabel('Y (m)', 'FontSize', 11); 
    zlabel('Z (m)', 'FontSize', 11);
    
    if show_waypoints
        legend('Actual', 'Reference', 'Waypoints', 'Start', 'End', 'Location', 'best');
    else
        legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');
    end
    
    title('Square Trajectory Tracking (3D View)', 'FontSize', 12, 'FontWeight', 'bold');
    grid on; 
    axis equal;
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
    
    % Load waypoints if available
    waypoint_times = [];
    waypoint_labels = {};
    if isfield(nominal, 'config') && isfield(nominal.config, 'trajectory_file')
        try
            wpt = load_waypoints(nominal.config.trajectory_file);
            waypoint_times = wpt.time;
            if isfield(wpt, 'labels')
                waypoint_labels = wpt.labels;
            end
        catch
            % Skip waypoints if loading fails
        end
    end
    
    labels = {'X', 'Y', 'Z'};
    for i = 1:3
        subplot(3,1,i);
        plot(t, pos_ref(:,i), 'r--', 'LineWidth', 1.5); hold on;
        plot(t, pos(:,i), 'b-', 'LineWidth', 1);
        
        % Add vertical lines at waypoint times
        if ~isempty(waypoint_times)
            y_limits = ylim;
            for j = 1:length(waypoint_times)
                line([waypoint_times(j) waypoint_times(j)], y_limits, ...
                     'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1);
                
                % Add waypoint label at top of plot
                if ~isempty(waypoint_labels) && j <= length(waypoint_labels)
                    text(waypoint_times(j), y_limits(2), sprintf(' %s', waypoint_labels{j}), ...
                         'VerticalAlignment', 'top', 'FontSize', 7, 'Color', [0.4 0.4 0.4]);
                end
            end
        end
        
        ylabel(sprintf('%s (m)', labels{i}));
        grid on;
        
        if i == 1
            if ~isempty(waypoint_times)
                legend('Reference', 'Actual', 'Waypoint', 'Location', 'best');
            else
                legend('Reference', 'Actual', 'Location', 'best');
            end
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

function write_paper_metrics(nominal, mc, analysis, filename)
    % Write comprehensive metrics table for paper using pre-computed analysis
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file for writing: %s', filename);
    end
    
    try
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'COMPREHENSIVE METRICS SUMMARY FOR PAPER\n');
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'Generated: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        fprintf(fid, '=================================================================\n\n');
        
        %% Trajectory Information
        fprintf(fid, 'TRAJECTORY INFORMATION\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(mc.config, 'trajectory_file')
            fprintf(fid, 'Trajectory:     %s\n', mc.config.trajectory_file);
        end
        fprintf(fid, 'Duration:       %.2f seconds\n', nominal.t(end));
        fprintf(fid, 'Time steps:     %d\n', length(nominal.t));
        fprintf(fid, '\n');
        
        %% Nominal Performance (Baseline)
        fprintf(fid, 'NOMINAL SIMULATION (Baseline)\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            m = nominal.metrics.tracking;
            fprintf(fid, 'Position RMSE:       %.4f m\n', m.rmse_position);
            fprintf(fid, 'Attitude RMSE:       %.4f rad (%.2f deg)\n', ...
                    m.rmse_attitude, rad2deg(m.rmse_attitude));
            fprintf(fid, 'Max Position Error:  %.4f m\n', m.max_position_error);
            fprintf(fid, 'Max Roll:            %.4f rad (%.2f deg)\n', ...
                    m.max_roll, rad2deg(m.max_roll));
            fprintf(fid, 'Max Pitch:           %.4f rad (%.2f deg)\n', ...
                    m.max_pitch, rad2deg(m.max_pitch));
            
            if isfield(nominal.metrics, 'control')
                fprintf(fid, '\nControl Performance:\n');
                if isfield(nominal.metrics.control, 'total_effort')
                    fprintf(fid, '  Total Effort:      %.4f\n', nominal.metrics.control.total_effort);
                end
                if isfield(nominal.metrics.control, 'thrust_saturation_pct')
                    fprintf(fid, '  Thrust Sat.:       %.1f%%\n', nominal.metrics.control.thrust_saturation_pct);
                    fprintf(fid, '  Torque Sat.:       %.1f%%\n', nominal.metrics.control.torque_saturation_pct);
                end
            end
        end
        fprintf(fid, '\n');
        
        %% Monte Carlo Statistics
        n_trials = length(mc.trials);
        n_success = mc.statistics.n_success;
        n_failed = mc.statistics.n_failed;
        success_rate = 100 * n_success / n_trials;
        
        fprintf(fid, 'MONTE CARLO ROBUSTNESS ANALYSIS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Total Trials:        %d\n', n_trials);
        fprintf(fid, 'Successful:          %d (%.1f%%)\n', n_success, success_rate);
        fprintf(fid, 'Failed:              %d (%.1f%%)\n', n_failed, 100 - success_rate);
        if isfield(mc, 'elapsed_time')
            fprintf(fid, 'Computation Time:    %.1f seconds (%.2f sec/trial)\n', ...
                    mc.elapsed_time, mc.elapsed_time / n_trials);
        end
        fprintf(fid, '\n');
        
        %% Performance Statistics (Successful Trials Only)
        if isfield(mc, 'statistics') && isfield(mc.statistics, 'metrics')
            m = mc.statistics.metrics;
            
            fprintf(fid, 'PERFORMANCE STATISTICS (Successful Trials)\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            
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
            
            % Control Effort (if available)
            if isfield(m, 'control_effort_mean')
                fprintf(fid, 'Control Effort:\n');
                fprintf(fid, '  Mean ± Std:        %.4f ± %.4f\n', ...
                        m.control_effort_mean, m.control_effort_std);
                fprintf(fid, '\n');
            end
        end
        
        %% Parameter Correlation Analysis
        fprintf(fid, 'PARAMETER CORRELATION ANALYSIS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, '(Pearson correlation coefficients: -1 to +1)\n');
        fprintf(fid, '(Positive = parameter increase worsens performance)\n\n');
        
        if isfield(analysis, 'correlations')
            corr_data = analysis.correlations;
            param_names = analysis.param_data.param_names;
            
            % Position RMSE correlations
            fprintf(fid, 'Correlation with Position RMSE:\n');
            [~, pos_idx] = sort(abs(corr_data.rmse_position), 'descend');
            for i = 1:length(param_names)
                idx = pos_idx(i);
                fprintf(fid, '  %10s:  %+.3f', param_names{idx}, corr_data.rmse_position(idx));
                if i == 1
                    fprintf(fid, '  ** STRONGEST **\n');
                else
                    fprintf(fid, '\n');
                end
            end
            fprintf(fid, '\n');
            
            % Attitude RMSE correlations
            fprintf(fid, 'Correlation with Attitude RMSE:\n');
            [~, att_idx] = sort(abs(corr_data.rmse_attitude), 'descend');
            for i = 1:length(param_names)
                idx = att_idx(i);
                fprintf(fid, '  %10s:  %+.3f', param_names{idx}, corr_data.rmse_attitude(idx));
                if i == 1
                    fprintf(fid, '  ** STRONGEST **\n');
                else
                    fprintf(fid, '\n');
                end
            end
            fprintf(fid, '\n');
            
            % Max position error correlations
            if isfield(corr_data, 'max_position_error')
                fprintf(fid, 'Correlation with Max Position Error:\n');
                [~, max_idx] = sort(abs(corr_data.max_position_error), 'descend');
                for i = 1:length(param_names)
                    idx = max_idx(i);
                    fprintf(fid, '  %10s:  %+.3f', param_names{idx}, corr_data.max_position_error(idx));
                    if i == 1
                        fprintf(fid, '  ** STRONGEST **\n');
                    else
                        fprintf(fid, '\n');
                    end
                end
                fprintf(fid, '\n');
            end
        end
        
        %% Normalized Sensitivity Analysis
        if isfield(analysis, 'sensitivity')
            fprintf(fid, 'NORMALIZED SENSITIVITY INDICES\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            fprintf(fid, '(Normalized by parameter variation range)\n');
            fprintf(fid, '(Higher absolute values = greater relative impact)\n\n');
            
            sens = analysis.sensitivity;
            param_names = analysis.param_data.param_names;
            
            % Position RMSE sensitivity
            fprintf(fid, 'Position RMSE Sensitivity:\n');
            [~, idx] = sort(abs(sens.rmse_position), 'descend');
            for i = 1:length(param_names)
                fprintf(fid, '  %10s:  %+.3f', param_names{idx(i)}, sens.rmse_position(idx(i)));
                if i == 1
                    fprintf(fid, '  ** MOST SENSITIVE **\n');
                else
                    fprintf(fid, '\n');
                end
            end
            fprintf(fid, '\n');
            
            % Attitude RMSE sensitivity
            fprintf(fid, 'Attitude RMSE Sensitivity:\n');
            [~, idx] = sort(abs(sens.rmse_attitude), 'descend');
            for i = 1:length(param_names)
                fprintf(fid, '  %10s:  %+.3f', param_names{idx(i)}, sens.rmse_attitude(idx(i)));
                if i == 1
                    fprintf(fid, '  ** MOST SENSITIVE **\n');
                else
                    fprintf(fid, '\n');
                end
            end
            fprintf(fid, '\n');
        end
        
        %% Worst-Case Analysis
        fprintf(fid, 'WORST-CASE PERFORMANCE\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        if isfield(analysis, 'metrics_data')
            metrics = analysis.metrics_data;
            
            [worst_pos, worst_pos_idx] = max(metrics.rmse_position);
            [worst_att, worst_att_idx] = max(metrics.rmse_attitude);
            
            fprintf(fid, 'Worst Position RMSE:  %.4f m (successful trial #%d)\n', ...
                    worst_pos, worst_pos_idx);
            fprintf(fid, 'Worst Attitude RMSE:  %.4f rad (%.2f deg) (successful trial #%d)\n', ...
                    worst_att, rad2deg(worst_att), worst_att_idx);
            
            if isfield(metrics, 'max_position_error')
                [worst_max_err, worst_max_idx] = max(metrics.max_position_error);
                fprintf(fid, 'Worst Max Error:      %.4f m (successful trial #%d)\n', ...
                        worst_max_err, worst_max_idx);
            end
        end
        fprintf(fid, '\n');
        
        %% Failure Analysis
        if isfield(analysis, 'failure_analysis') && n_failed > 0
            fprintf(fid, 'FAILURE ANALYSIS\n');
            fprintf(fid, '-----------------------------------------------------------------\n');
            fprintf(fid, 'Failed Trials:       %d (%.1f%%)\n\n', ...
                    n_failed, 100 * n_failed / n_trials);
            
            fail = analysis.failure_analysis;
            
            fprintf(fid, 'Unique failure types: %d\n\n', ...
                    length(fail.error_messages));
            
            fprintf(fid, 'Common failure messages:\n');
            for i = 1:min(5, length(fail.error_messages))  % Top 5
                fprintf(fid, '  %d. %s\n', i, fail.error_messages{i});
            end
            fprintf(fid, '\n');
            
            % Parameter patterns in failures
            fprintf(fid, 'Average parameter values in failed trials:\n');
            param_names = analysis.param_data.param_names;
            for i = 1:length(param_names)
                fprintf(fid, '  %10s:  %.4f ± %.4f\n', ...
                        param_names{i}, ...
                        fail.param_means(i), ...
                        fail.param_stds(i));
            end
            fprintf(fid, '\n');
        end
        
        %% LaTeX Snippets
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'LATEX SNIPPETS FOR PAPER\n');
        fprintf(fid, '=================================================================\n\n');
        
        % Abstract / Introduction
        fprintf(fid, '%% ABSTRACT / INTRODUCTION\n');
        fprintf(fid, '%% Copy-paste these values directly into your paper\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        if isfield(nominal, 'metrics') && isfield(nominal.metrics, 'tracking')
            fprintf(fid, 'Nominal tracking accuracy:\n');
            fprintf(fid, '$\\mathrm{RMSE}_{pos} = %.3f$ m, $\\mathrm{RMSE}_{att} = %.2f$ deg\n\n', ...
                    nominal.metrics.tracking.rmse_position, ...
                    rad2deg(nominal.metrics.tracking.rmse_attitude));
        end
        fprintf(fid, 'Monte Carlo validation: $N = %d$ trials with $%.1f\\%%$ success rate\n\n', ...
                n_trials, success_rate);
        
        % Results Section
        fprintf(fid, '%% RESULTS SECTION\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        if isfield(mc.statistics, 'metrics')
            m = mc.statistics.metrics;
            
            fprintf(fid, '%% Performance under parameter uncertainty:\n');
            fprintf(fid, 'Position tracking: $%.3f \\pm %.3f$ m (mean $\\pm$ std)\n\n', ...
                    m.rmse_position_mean, m.rmse_position_std);
            fprintf(fid, 'Attitude tracking: $%.2f \\pm %.2f$ deg (mean $\\pm$ std)\n\n', ...
                    rad2deg(m.rmse_attitude_mean), rad2deg(m.rmse_attitude_std));
            
            % Table for percentiles
            fprintf(fid, '%% Table: Performance Distribution\n');
            fprintf(fid, '\\begin{table}[h]\n');
            fprintf(fid, '\\centering\n');
            fprintf(fid, '\\begin{tabular}{lcc}\n');
            fprintf(fid, '\\hline\n');
            fprintf(fid, 'Metric & Position RMSE (m) & Attitude RMSE (deg) \\\\\n');
            fprintf(fid, '\\hline\n');
            fprintf(fid, 'Mean & %.3f & %.2f \\\\\n', ...
                    m.rmse_position_mean, rad2deg(m.rmse_attitude_mean));
            fprintf(fid, 'Std Dev & %.3f & %.2f \\\\\n', ...
                    m.rmse_position_std, rad2deg(m.rmse_attitude_std));
            fprintf(fid, 'Median & %.3f & %.2f \\\\\n', ...
                    m.rmse_position_median, rad2deg(m.rmse_attitude_median));
            fprintf(fid, '5th Percentile & %.3f & %.2f \\\\\n', ...
                    m.rmse_position_percentiles(1), rad2deg(m.rmse_attitude_percentiles(1)));
            fprintf(fid, '95th Percentile & %.3f & %.2f \\\\\n', ...
                    m.rmse_position_percentiles(4), rad2deg(m.rmse_attitude_percentiles(4)));
            fprintf(fid, '\\hline\n');
            fprintf(fid, '\\end{tabular}\n');
            fprintf(fid, '\\caption{Monte Carlo Performance Distribution ($N=%d$ successful trials)}\n', n_success);
            fprintf(fid, '\\label{tab:mc_performance}\n');
            fprintf(fid, '\\end{table}\n\n');
        end
        
        % Discussion Section - Parameter Sensitivity
        fprintf(fid, '%% DISCUSSION SECTION - Parameter Sensitivity\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        if isfield(analysis, 'correlations')
            param_names = analysis.param_data.param_names;
            corr_data = analysis.correlations;
            
            % Find strongest correlations
            [~, pos_strongest_idx] = max(abs(corr_data.rmse_position));
            [~, att_strongest_idx] = max(abs(corr_data.rmse_attitude));
            
            fprintf(fid, '%% Key sensitivity findings:\n');
            fprintf(fid, 'Position tracking most sensitive to %s ($r = %+.3f$)\n\n', ...
                    param_names{pos_strongest_idx}, ...
                    corr_data.rmse_position(pos_strongest_idx));
            
            fprintf(fid, 'Attitude tracking most sensitive to %s ($r = %+.3f$)\n\n', ...
                    param_names{att_strongest_idx}, ...
                    corr_data.rmse_attitude(att_strongest_idx));
        end
        
        % Conclusion Section
        fprintf(fid, '%% CONCLUSION SECTION\n');
        fprintf(fid, '%%-----------------------------------------------------------\n\n');
        fprintf(fid, '%% Robustness summary:\n');
        fprintf(fid, 'System achieved $%.1f\\%%$ success rate across $N=%d$ trials\n', ...
                success_rate, n_trials);
        fprintf(fid, 'with parameter variations representing operational uncertainties.\n\n');
        
        if isfield(mc.statistics, 'metrics')
            m = mc.statistics.metrics;
            fprintf(fid, '%% Performance remained within acceptable bounds:\n');
            fprintf(fid, '95th percentile errors: $%.3f$ m (position), $%.2f$ deg (attitude)\n\n', ...
                    m.rmse_position_percentiles(4), ...
                    rad2deg(m.rmse_attitude_percentiles(4)));
        end
        
        fprintf(fid, '=================================================================\n');
        fprintf(fid, 'END OF METRICS REPORT\n');
        fprintf(fid, '=================================================================\n');
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
    
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

function listing = list_available_runs(run_label)
    % List available run directories for debugging
    pattern = fullfile('.', 'results', sprintf('%s_*', run_label));
    dirs = dir(pattern);
    
    if isempty(dirs)
        listing = sprintf('  (No results found for "%s")', run_label);
    else
        listing = '';
        for i = 1:length(dirs)
            if dirs(i).isdir
                listing = [listing sprintf('  - %s\n', dirs(i).name)]; %#ok<AGROW>
            end
        end
    end
end