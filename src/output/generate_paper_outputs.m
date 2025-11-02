function generate_paper_outputs(results_dir, options)
% GENERATE_PAPER_FIGURES Creates publication-quality figures from saved simulation data
%
% SYNTAX:
%   generate_paper_outputs(results_dir)
%   generate_paper_outputs(results_dir, options)
%
% INPUTS:
%   results_dir - Path to results directory containing nominal.mat and monte_carlo.mat
%                 Example: './results/simple_square_10_20251101_143022'
%   options     - [optional] Struct with:
%                 .close_figures - Close figures after saving (default: true)
%                 .verbose       - Print progress messages (default: true)
%
% OUTPUTS:
%   Creates figures subdirectory: <results_dir>/figures/
%   Saves six figures plus metrics:
%     - tracking_3d.png
%     - tracking_timeseries.png
%     - control_inputs.png
%     - attitude_dynamics.png
%     - distributions.png
%     - boxplots.png
%     - correlation.png
%     - paper_metrics.txt
%
% EXAMPLE:
%   % After running simulate_monte_carlo
%   results_dir = simulate_monte_carlo('simple_square.wpt', 'paper_final');
%   generate_paper_outputs(results_dir);
%
%   % With options
%   opts.close_figures = false;  % Keep figures open
%   generate_paper_outputs(results_dir, opts);
%
% See also: simulate_monte_carlo, analyze_monte_carlo_results

% Author: Trey Copeland
% Date: 2025-11-01 (Refactored for unified directory structure)

    %% Input handling
    if nargin < 1
        error('Must provide results_dir');
    end
    
    if nargin < 2
        options = struct();
    end
    
    % Set default options using standardized helper
    defaults = struct(...
        'close_figures', true, ...
        'verbose', true ...
    );
    options = set_default_options(options, defaults);
    
    %% Verify results directory exists
    if ~exist(results_dir, 'dir')
        error('Results directory not found: %s\nRun simulate_monte_carlo first or check the path.', results_dir);
    end
    
    if options.verbose
        fprintf('Using results directory: %s\n', results_dir);
    end
    
    %% Create figures subdirectory
    figures_dir = fullfile(results_dir, Constants.FIGURES_DIR);
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
    end
    
    if options.verbose
        fprintf('\n=======================================================\n');
        fprintf('GENERATING FIGURES\n');
        fprintf('=======================================================\n');
        fprintf('Loading from: %s\n', results_dir);
        fprintf('Saving to:    %s\n', figures_dir);
        fprintf('=======================================================\n\n');
    end
    
    %% Load data
    if options.verbose
        fprintf('Loading nominal simulation...\n');
    end
    nominal_file = fullfile(results_dir, Constants.NOMINAL_DATA);
    if ~exist(nominal_file, 'file')
        error('Nominal data file not found: %s\nRun simulate_monte_carlo first.', nominal_file);
    end
    nominal = DataManager.load_results(nominal_file);
    
    if options.verbose
        fprintf('Loading Monte Carlo results...\n');
    end
    mc_file = fullfile(results_dir, Constants.MC_DATA);
    if ~exist(mc_file, 'file')
        error('Monte Carlo data file not found: %s\nRun simulate_monte_carlo first.', mc_file);
    end
    % Monte Carlo results have different schema - skip validation and migration
    mc = DataManager.load_monte_carlo(mc_file, struct('verbose', options.verbose));
    
    %% Generate figures
    if options.verbose
        fprintf('Generating tracking figures...\n');
    end
    
    fig1 = create_3d_tracking_figure(nominal);
    save_and_close_figure(fig1, fullfile(figures_dir, 'tracking_3d.png'), options.close_figures);
    
    fig2 = create_tracking_timeseries(nominal);
    save_and_close_figure(fig2, fullfile(figures_dir, 'tracking_timeseries.png'), options.close_figures);
    
    fig3 = create_control_inputs_figure(nominal);
    save_and_close_figure(fig3, fullfile(figures_dir, 'control_inputs.png'), options.close_figures);
    
    fig4 = create_attitude_dynamics_figure(nominal);
    save_and_close_figure(fig4, fullfile(figures_dir, 'attitude_dynamics.png'), options.close_figures);
    
    %% Generate Monte Carlo analysis figures
    if options.verbose
        fprintf('Running Monte Carlo analysis and generating plots...\n');
    end
    
    analysis_options.plot = true;
    analysis_options.save_plots = true;
    analysis_options.plot_dir = figures_dir;
    analysis_options.verbose = false;
    analysis_options.close_figures = options.close_figures;
    
    analysis = analyze_monte_carlo(mc, analysis_options);

    % Generate comprehensive metrics report
    if options.verbose
        fprintf('\nGenerating comprehensive metrics report...\n');
    end
    
    % Store analysis results in mc structure (only if they exist)
    if isfield(analysis, 'correlations')
        mc.statistics.correlations = analysis.correlations;
    end  
    if isfield(analysis, 'sensitivity')
        % Might be named 'sensitivity' instead
        mc.statistics.normalized_sensitivity = analysis.sensitivity;
    end
    if isfield(analysis, 'param_data') && isfield(analysis.param_data, 'param_names')
        mc.statistics.param_names = analysis.param_data.param_names;
    end
    
    % Compute worst-case values from raw trial data
    if isfield(analysis, 'metrics_data')
        mc.statistics.worst_case = struct();
        mc.statistics.worst_case.rmse_position_max = max(analysis.metrics_data.rmse_position);
        mc.statistics.worst_case.rmse_attitude_max = max(analysis.metrics_data.rmse_attitude);
        mc.statistics.worst_case.max_position_error_max = max(analysis.metrics_data.max_position_error);
        
        % Find which trial had worst values
        [~, worst_pos_idx] = max(analysis.metrics_data.rmse_position);
        [~, worst_att_idx] = max(analysis.metrics_data.rmse_attitude);
        
        mc.statistics.worst_case.worst_pos_trial = worst_pos_idx;
        mc.statistics.worst_case.worst_att_trial = worst_att_idx;
    
        % Store parameter values for worst-case trial (position RMSE)
        if isfield(analysis, 'param_data') && isfield(analysis.param_data, 'matrix')
            % param_data.matrix is [n_trials x n_params]
            mc.statistics.worst_case.worst_pos_params = analysis.param_data.matrix(worst_pos_idx, :);
            mc.statistics.worst_case.worst_att_params = analysis.param_data.matrix(worst_att_idx, :);
        end
    end

    % Write comprehensive report (with correlations)
    metrics_file = fullfile(results_dir, Constants.ANALYSIS_REPORT);
    write_unified_metrics_report(nominal, mc, metrics_file);

    %% Generate LaTeX snippets
    if options.verbose
        fprintf('Generating LaTeX snippets...\n');
    end
    latex_file = fullfile(results_dir, Constants.LATEX_SNIPPETS);
    write_latex_snippets(nominal, mc, latex_file);
    
    if options.verbose
        fprintf('  Saved: %s\n', metrics_file);
    end
    
    % Verify MC figures were created
    if options.verbose
        fprintf('Verifying Monte Carlo figures...\n');
        mc_figure_files = {'distributions.png', 'boxplots.png', 'correlation.png'};
        for i = 1:length(mc_figure_files)
            if exist(fullfile(figures_dir, mc_figure_files{i}), 'file')
                fprintf('  ✓ %s\n', mc_figure_files{i});
            else
                fprintf('  ✗ %s (missing)\n', mc_figure_files{i});
            end
        end
    end       
    
    %% Regenerate run log (if it doesn't exist or we're forcing regeneration)
    if options.verbose
        fprintf('Regenerating run log...\n');
    end
    log_file = fullfile(results_dir, Constants.RUN_LOG);
    write_run_log(mc, nominal, log_file);

    %% Summary
    if options.verbose
        fprintf('\n=======================================================\n');
        fprintf('✅ FIGURES COMPLETE\n');
        fprintf('=======================================================\n');
        fprintf('All outputs saved to: %s\n', figures_dir);
        fprintf('\nGenerated files:\n');
        fprintf('  - tracking_3d.png\n');
        fprintf('  - tracking_timeseries.png\n');
        fprintf('  - control_inputs.png\n');
        fprintf('  - attitude_dynamics.png\n');
        fprintf('  - distributions.png\n');
        fprintf('  - boxplots.png\n');
        fprintf('  - correlation.png\n');
        fprintf('  - paper_metrics.txt (LaTeX snippets)\n');
        fprintf('=======================================================\n\n');
    end
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

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
    show_waypoints = true;
    if isfield(nominal.trajectory, 'waypoints')
        try
            wpt = nominal.trajectory.waypoints;
            
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
                             'HorizontalAlignment', 'center', ...
                             'Interpreter', 'none');
                    end
                end
            end
        catch ME
            % If waypoint plotting fails, skip waypoint markers
            warning('Could not plot waypoints: %s', ME.message);
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
        
        % Add waypoint markers (labels only on first subplot)
        add_waypoint_markers(nominal, true, i, 3);
    end
    xlabel('Time (s)');
end

function fig = create_control_inputs_figure(nominal)
    % Create control inputs visualization with waypoint markers
    
    fig = figure('Position', [100 100 1000 700]);
    
    t = nominal.t;
    u = nominal.u_log;
    
    % Thrust
    subplot(4,1,1);
    plot(t, u(:,1), 'b-', 'LineWidth', 1.5);
    ylabel('Thrust (N)', 'FontSize', 10);
    grid on;
    title('Control Inputs vs Time', 'FontSize', 12, 'FontWeight', 'bold');
    add_waypoint_markers(nominal, true, 1, 4);  % Labels on top
    
    % Torques
    labels = {'\tau_\phi (Roll)', '\tau_\theta (Pitch)', '\tau_\psi (Yaw)'};
    for i = 1:3
        subplot(4,1,i+1);
        plot(t, u(:,i+1), 'b-', 'LineWidth', 1.5);
        ylabel(sprintf('%s (N·m)', labels{i}), 'FontSize', 10);
        grid on;
        
        % Add waypoint markers (no labels on lower subplots)
        add_waypoint_markers(nominal, true, i+1, 4);
    end
    
    xlabel('Time (s)', 'FontSize', 10);
end

function fig = create_attitude_dynamics_figure(nominal)
    % Shows how attitude and rates evolve during tracking
    
    fig = figure('Position', [100 100 1200 800]);
    
    t = nominal.t;
    att = nominal.x(:,4:6);      % [roll, pitch, yaw]
    omega = nominal.x(:,10:12);  % [p, q, r] angular rates
    vel = nominal.x(:,7:9);      % [vx, vy, vz]
    vel_mag = sqrt(sum(vel.^2, 2));
    
    % Subplot 1: Attitude angles
    subplot(3,1,1);
    plot(t, rad2deg(att(:,1)), 'r-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(att(:,2)), 'g-', 'LineWidth', 1.5);
    plot(t, rad2deg(att(:,3)), 'b-', 'LineWidth', 1.5);
    ylabel('Angle (deg)', 'FontSize', 10);
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best', 'FontSize', 9);
    title('Attitude and Rate Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    add_waypoint_markers(nominal, true, 1, 3);  % Labels on top subplot
    
    % Subplot 2: Angular rates
    subplot(3,1,2);
    plot(t, rad2deg(omega(:,1)), 'r-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(omega(:,2)), 'g-', 'LineWidth', 1.5);
    plot(t, rad2deg(omega(:,3)), 'b-', 'LineWidth', 1.5);
    ylabel('Angular Rate (deg/s)', 'FontSize', 10);
    legend('p (roll rate)', 'q (pitch rate)', 'r (yaw rate)', 'Location', 'best', 'FontSize', 9);
    grid on;
    add_waypoint_markers(nominal, true, 2, 3);  % No labels
    
    % Subplot 3: Linear velocity
    subplot(3,1,3);
    plot(t, vel_mag, 'k-', 'LineWidth', 2); hold on;
    plot(t, vel(:,1), 'r--', 'LineWidth', 1);
    plot(t, vel(:,2), 'g--', 'LineWidth', 1);
    plot(t, vel(:,3), 'b--', 'LineWidth', 1);
    ylabel('Velocity (m/s)', 'FontSize', 10);
    xlabel('Time (s)', 'FontSize', 10);
    legend('|V| (magnitude)', 'V_x', 'V_y', 'V_z', 'Location', 'best', 'FontSize', 9);
    grid on;
    add_waypoint_markers(nominal, true, 3, 3);  % No labels
end

function add_waypoint_markers(nominal, show_labels, subplot_idx, total_subplots)
    % ADD_WAYPOINT_MARKERS - Add vertical lines and labels for waypoints
    %
    % INPUTS:
    %   nominal         - Nominal simulation results structure
    %   show_labels     - (optional) Boolean to show waypoint labels (default: true)
    %   subplot_idx     - (optional) Current subplot index (1-based)
    %   total_subplots  - (optional) Total number of subplots in figure
    %
    % NOTES:
    %   Labels only appear on the first (top) subplot to avoid duplication
    %   Last waypoint label is skipped (overlaps with trajectory end marker)
    
    if nargin < 2
        show_labels = true;
    end
    if nargin < 3
        subplot_idx = 1;
    end
    if nargin < 4
        total_subplots = 1;
    end
    
    % Try to get waypoints from trajectory structure
    if ~isfield(nominal, 'trajectory') || ~isfield(nominal.trajectory, 'waypoints')
        return;
    end
    
    try
        wpt = nominal.trajectory.waypoints;
        waypoint_times = wpt.time;
        if isfield(wpt, 'labels')
            waypoint_labels = wpt.labels;
        else
            waypoint_labels = {};
        end
    catch
        return;
    end    
    % Ensure waypoints are sorted (defensive check, should already be sorted)
    [waypoint_times, sort_idx] = sort(waypoint_times);
    if ~isempty(waypoint_labels)
        waypoint_labels = waypoint_labels(sort_idx);
    end
    
    % Get current axis limits
    ylim_current = ylim;
    
    % Add vertical lines at each waypoint time
    n_waypoints = length(waypoint_times);
    for j = 1:n_waypoints
        line([waypoint_times(j) waypoint_times(j)], ylim_current, ...
             'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1, ...
             'HandleVisibility', 'off');
        
        % Add label ONLY on first subplot, and skip the last waypoint (overlaps with end)
        if show_labels && subplot_idx == 1 && j < n_waypoints && ...
           ~isempty(waypoint_labels) && j <= length(waypoint_labels)
            if ~isempty(waypoint_labels{j})
                text(waypoint_times(j), ylim_current(2), sprintf(' %s', waypoint_labels{j}), ...
                     'VerticalAlignment', 'top', 'FontSize', 8, ...
                     'Color', [0.3 0.3 0.3], 'FontWeight', 'bold', ...
                     'Interpreter', 'none');
            end
        end
    end
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

function save_and_close_figure(fig, filename, close_fig)
    % SAVE_AND_CLOSE_FIGURE - Unified figure saving and optional closing
    %
    % INPUTS:
    %   fig       - Figure handle
    %   filename  - Full path to save location
    %   close_fig - Boolean to close after saving
    
    saveas(fig, filename);
    if close_fig
        close(fig);
    end
end

function results_dir = find_latest_results_dir(run_label)
    % Find most recent results directory for given run_label
    
    % Look for directories matching pattern
    pattern = fullfile('./results', sprintf('%s_*', run_label));
    dirs = dir(pattern);
    
    if isempty(dirs)
        error('No results found for run_label: %s\nRun simulate_monte_carlo first.', run_label);
    end
    
    % Extract timestamps from directory names
    timestamps = {};
    dir_names = {};
    for i = 1:length(dirs)
        if dirs(i).isdir && ~strcmp(dirs(i).name, '.') && ~strcmp(dirs(i).name, '..')
            % Pattern: runlabel_YYYYMMDD_HHMMSS
            tokens = regexp(dirs(i).name, sprintf('%s_(\\d{8}_\\d{6})$', run_label), 'tokens');
            if ~isempty(tokens)
                timestamps{end+1} = tokens{1}{1}; %#ok<AGROW>
                dir_names{end+1} = dirs(i).name; %#ok<AGROW>
            end
        end
    end
    
    if isempty(timestamps)
        error('No valid results directories found for: %s\nExpected format: %s_YYYYMMDD_HHMMSS', ...
              run_label, run_label);
    end
    
    % Sort timestamps and get latest
    [~, idx] = max(cellfun(@(x) datenum(x, 'yyyymmdd_HHMMSS'), timestamps));
    latest_dir = dir_names{idx};
    
    results_dir = fullfile('./results', latest_dir);
end