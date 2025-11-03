function analysis = analyze_monte_carlo(mc_results, analysis_options)
% ANALYZE_MONTE_CARLO_RESULTS - Statistical analysis and visualization of MC data
%
% Performs comprehensive statistical analysis of Monte Carlo simulation results
% including parameter sensitivity, failure analysis, and performance distributions.
%
% SYNTAX:
%   analyze_monte_carlo(mc_results)
%   analyze_monte_carlo(mc_results, analysis_options)
%   analysis = analyze_monte_carlo(...)
%
% INPUTS:
%   mc_results       - Results structure from run_monte_carlo()
%   analysis_options - (optional) Analysis configuration:
%                      .plot            - Generate plots (default: true)
%                      .save_plots      - Save figures to disk (default: false)
%                      .plot_dir        - Directory for saved plots (default: './figures/')
%                      .correlation     - Compute parameter correlations (default: true)
%                      .sensitivity     - Perform sensitivity analysis (default: true)
%                      .percentiles     - Percentile levels for analysis (default: [5 25 50 75 95])
%                      .verbose         - Print detailed report (default: true)
%
% OUTPUTS:
%   analysis - Structure containing:
%              .statistics       - Summary statistics
%              .correlations     - Parameter-metric correlations
%              .sensitivity      - Sensitivity indices
%              .failure_analysis - Details on failed trials
%              .figures          - Figure handles (if plots generated)
%
% PLOTS GENERATED:
%   1. Performance distribution histograms (RMSE, control effort)
%   2. Parameter vs. performance scatter plots
%   3. Box plots for robustness visualization
%   4. Correlation matrix heatmap
%   5. Success rate by parameter ranges
%   6. Cumulative distribution functions (CDFs)
%
% EXAMPLE:
%   % Run Monte Carlo
%   mc_results = run_monte_carlo('basic_maneuver.wpt', perturb_config, mc_options);
%   
%   % Analyze with plots
%   analysis = analyze_monte_carlo(mc_results);
%   
%   % Custom analysis
%   opts.save_plots = true;
%   opts.plot_dir = './mc_results/figures/';
%   analysis = analyze_monte_carlo(mc_results, opts);
%
% See also: run_monte_carlo, simulate_quadrotor_pure

% Author: Trey Copeland
% Date: 2025-10-15

    %% Parse inputs
    if nargin < 2
        analysis_options = struct();
    end
    
    % Define analysis defaults
    defaults.plot = true;
    defaults.save_plots = false;
    defaults.plot_dir = './figures/';
    defaults.correlation = true;
    defaults.sensitivity = true;
    defaults.percentiles = [5 25 50 75 95];
    defaults.verbose = true;
    
    analysis_options = set_default_options(analysis_options, defaults);

    %% Header
    if analysis_options.verbose
        fprintf('\n===================================\n');
        fprintf('MONTE CARLO ANALYSIS\n');
        fprintf('===================================\n\n');
    end
    
    %% Extract data
    trials = mc_results.trials;
    statistics = mc_results.statistics;
    perturb_config = mc_results.config.perturb_config;
    
    success_flags = [trials.success];
    successful_trials = trials(success_flags);
    failed_trials = trials(~success_flags);
    
    n_success = sum(success_flags);
    n_failed = sum(~success_flags);
    n_trials = length(trials);
    
    %% Basic Statistics Report
    if analysis_options.verbose
        fprintf('OVERALL STATISTICS\n');
        fprintf('------------------\n');
        fprintf('Total trials:        %d\n', n_trials);
        fprintf('Successful:          %d (%.1f%%)\n', n_success, 100*n_success/n_trials);
        fprintf('Failed:              %d (%.1f%%)\n\n', n_failed, 100*n_failed/n_trials);
    end
    
    if n_success == 0
        warning('No successful trials to analyze!');
        analysis = struct();
        return;
    end
    
    % Check if any trials had feasibility warnings
    n_infeasible = 0;
    for i = 1:length(trials)
        if isfield(trials(i), 'trajectory') && ...
           isfield(trials(i).trajectory, 'feasibility') && ...
           ~trials(i).trajectory.feasibility.feasible
            n_infeasible = n_infeasible + 1;
        end
    end
    
    statistics.trajectory_feasibility = struct(...
        'n_infeasible', n_infeasible, ...
        'pct_infeasible', 100 * n_infeasible / n_trials);

    %% Extract metrics from successful trials
    metrics_data = extract_metrics(successful_trials);
    
    %% Extract parameter variations
    param_data = extract_parameters(successful_trials, perturb_config);
    
    %% Performance Statistics
    if analysis_options.verbose
        fprintf('PERFORMANCE METRICS (Successful Trials)\n');
        fprintf('---------------------------------------\n');
        print_metric_stats('Position RMSE (m)', metrics_data.rmse_position, ...
                          analysis_options.percentiles);
        print_metric_stats('Attitude RMSE (rad)', metrics_data.rmse_attitude, ...
                          analysis_options.percentiles);
        print_metric_stats('Attitude RMSE (deg)', rad2deg(metrics_data.rmse_attitude), ...
                          analysis_options.percentiles);
        
        if isfield(metrics_data, 'control_effort')
            print_metric_stats('Control Effort', metrics_data.control_effort, ...
                              analysis_options.percentiles);
        end
        
        print_metric_stats('Max Position Error (m)', metrics_data.max_position_error, ...
                          analysis_options.percentiles);
        fprintf('\n');
    end
    
    %% Parameter Correlation Analysis
    if analysis_options.correlation
        if analysis_options.verbose
            fprintf('PARAMETER CORRELATION ANALYSIS\n');
            fprintf('------------------------------\n');
        end
        
        correlations = compute_correlations(param_data, metrics_data);
        
        if analysis_options.verbose
            print_correlation_table(correlations, param_data.param_names);
        end
        
        analysis.correlations = correlations;
    end
    
    %% Sensitivity Analysis
    if analysis_options.sensitivity
        if analysis_options.verbose
            fprintf('\nSENSITIVITY ANALYSIS\n');
            fprintf('--------------------\n');
        end
        
        sensitivity = compute_sensitivity(param_data, metrics_data);
        
        if analysis_options.verbose
            print_sensitivity_table(sensitivity, param_data.param_names);
        end
        
        analysis.sensitivity = sensitivity;
    end
    
    %% Failure Analysis
    if n_failed > 0
        if analysis_options.verbose
            fprintf('\nFAILURE ANALYSIS\n');
            fprintf('----------------\n');
            fprintf('Failed trials: %d (%.1f%%)\n\n', n_failed, 100*n_failed/n_trials);
        end
        
        failure_analysis = analyze_failures(failed_trials, perturb_config);
        
        if analysis_options.verbose
            print_failure_summary(failure_analysis);
        end
        
        analysis.failure_analysis = failure_analysis;
    end
    
    %% Generate Plots
    if analysis_options.plot
        if analysis_options.verbose
            fprintf('\nGENERATING VISUALIZATIONS\n');
            fprintf('-------------------------\n');
        end
        
        figures = generate_plots(mc_results, param_data, metrics_data, ...
                                correlations, analysis_options);
        
        analysis.figures = figures;
        
        % Save plots if requested
        if analysis_options.save_plots
            save_figures(figures, analysis_options.plot_dir);
        end
        
        % Close figures if requested
        if analysis_options.close_figures
            close_figures_from_struct(figures);
        end
    end
    
    %% Package results
    analysis.statistics = statistics;
    analysis.metrics_data = metrics_data;
    analysis.param_data = param_data;
    analysis.options = analysis_options;
    
    if analysis_options.verbose
        fprintf('\n===================================\n');
        fprintf('Analysis Complete!\n');
        fprintf('===================================\n\n');
    end
end

%% ========================================================================
%% HELPER FUNCTIONS - Data Extraction
%% ========================================================================

function metrics_data = extract_metrics(successful_trials)
    % Extract metric arrays from successful trials
    
    n = length(successful_trials);
    
    metrics_data.rmse_position = zeros(n, 1);
    metrics_data.rmse_attitude = zeros(n, 1);
    metrics_data.max_position_error = zeros(n, 1);
    metrics_data.max_roll = zeros(n, 1);
    metrics_data.max_pitch = zeros(n, 1);
    
    % Check if ANY trial has control effort
    has_control = false;
    for i = 1:n
        if isfield(successful_trials(i).metrics, 'control') && ...
           ~isempty(fieldnames(successful_trials(i).metrics.control))
            has_control = true;
            break;
        end
    end
    
    % Pre-allocate control_effort if needed
    if has_control
        metrics_data.control_effort = zeros(n, 1);
    end
    
    for i = 1:n
        m = successful_trials(i).metrics.tracking;
        metrics_data.rmse_position(i) = m.rmse_position;
        metrics_data.rmse_attitude(i) = m.rmse_attitude;
        metrics_data.max_position_error(i) = m.max_position_error;
        metrics_data.max_roll(i) = m.max_roll;
        metrics_data.max_pitch(i) = m.max_pitch;
        
        % Control effort (if available for this trial)
        if has_control
            if isfield(successful_trials(i).metrics, 'control') && ...
               ~isempty(fieldnames(successful_trials(i).metrics.control))
                metrics_data.control_effort(i) = successful_trials(i).metrics.control.total_effort;
            else
                metrics_data.control_effort(i) = 0;  % Default to 0 if missing
            end
        end
    end
end


function param_data = extract_parameters(successful_trials, perturb_config)
    % Extract parameter variations from trials
    
    n_trials = length(successful_trials);
    param_names = perturb_config.params(:, 1);
    n_params = length(param_names);
    
    param_matrix = zeros(n_trials, n_params);
    
    for i = 1:n_trials
        for j = 1:n_params
            param_matrix(i, j) = successful_trials(i).params.(param_names{j});
        end
    end
    
    param_data.matrix = param_matrix;
    param_data.param_names = param_names;
    param_data.n_params = n_params;
end

%% ========================================================================
%% HELPER FUNCTIONS - Statistical Analysis
%% ========================================================================

function r = correlation_coefficient(X, y)
    % CORRELATION_COEFFICIENT - Compute correlation without Statistics Toolbox
    %
    % Computes Pearson correlation coefficient between each column of X and vector y
    %
    % INPUTS:
    %   X - [n x p] matrix of predictors
    %   y - [n x 1] vector of responses
    %
    % OUTPUTS:
    %   r - [p x 1] vector of correlation coefficients
    
    n = size(X, 1);
    p = size(X, 2);
    r = zeros(p, 1);
    
    % Center and normalize y
    y_centered = y - mean(y);
    y_norm = sqrt(sum(y_centered.^2));
    
    % Compute correlation for each column
    for i = 1:p
        x_centered = X(:, i) - mean(X(:, i));
        x_norm = sqrt(sum(x_centered.^2));
        
        if x_norm > eps && y_norm > eps
            r(i) = sum(x_centered .* y_centered) / (x_norm * y_norm);
        else
            r(i) = 0;  % Handle zero variance case
        end
    end
end

function correlations = compute_correlations(param_data, metrics_data)
    % Compute correlation between parameters and performance metrics
    
    param_matrix = param_data.matrix;
    
    % Correlations with position RMSE
    correlations.rmse_position = correlation_coefficient(param_matrix, metrics_data.rmse_position);
    
    % Correlations with attitude RMSE
    correlations.rmse_attitude = correlation_coefficient(param_matrix, metrics_data.rmse_attitude);
    
    % Correlations with control effort (if available)
    if isfield(metrics_data, 'control_effort')
        correlations.control_effort = correlation_coefficient(param_matrix, metrics_data.control_effort);
    end
    
    % Correlations with max error
    correlations.max_position_error = correlation_coefficient(param_matrix, metrics_data.max_position_error);
end

function sensitivity = compute_sensitivity(param_data, metrics_data)
    % Compute sensitivity indices (normalized correlation coefficients)
    
    param_matrix = param_data.matrix;
    
    % Normalize parameters to [0, 1] for fair comparison
    param_normalized = (param_matrix - min(param_matrix)) ./ ...
                       (max(param_matrix) - min(param_matrix) + eps);
    
    % Compute normalized sensitivity (standardized regression coefficients)
    sensitivity.rmse_position = compute_std_regression(param_normalized, ...
                                                       metrics_data.rmse_position);
    sensitivity.rmse_attitude = compute_std_regression(param_normalized, ...
                                                       metrics_data.rmse_attitude);
    
    if isfield(metrics_data, 'control_effort')
        sensitivity.control_effort = compute_std_regression(param_normalized, ...
                                                           metrics_data.control_effort);
    end
end

function beta_std = compute_std_regression(X, y)
    % Compute standardized regression coefficients
    
    % Standardize inputs
    X_std = (X - mean(X)) ./ std(X);
    y_std = (y - mean(y)) ./ std(y);
    
    % Regression
    beta_std = (X_std' * X_std) \ (X_std' * y_std);
end

function failure_analysis = analyze_failures(failed_trials, perturb_config)
    % Analyze characteristics of failed trials
    
    n_failed = length(failed_trials);
    param_names = perturb_config.params(:, 1);
    n_params = length(param_names);
    
    % Extract parameters from failed trials
    param_matrix_failed = zeros(n_failed, n_params);
    for i = 1:n_failed
        for j = 1:n_params
            param_matrix_failed(i, j) = failed_trials(i).params.(param_names{j});
        end
    end
    
    failure_analysis.n_failed = n_failed;
    failure_analysis.param_means = mean(param_matrix_failed, 1);
    failure_analysis.param_stds = std(param_matrix_failed, 0, 1);
    failure_analysis.error_messages = unique({failed_trials.failure_msg});
end

%% ========================================================================
%% HELPER FUNCTIONS - Printing
%% ========================================================================

function print_metric_stats(metric_name, data, percentiles)
    % Print statistics for a metric
    
    fprintf('  %s:\n', metric_name);
    fprintf('    Mean:       %.4f\n', mean(data));
    fprintf('    Std Dev:    %.4f\n', std(data));
    fprintf('    Median:     %.4f\n', median(data));
    fprintf('    Min:        %.4f\n', min(data));
    fprintf('    Max:        %.4f\n', max(data));
    
    prc_values = prctile(data, percentiles);
    prc_str = sprintf('    Percentiles [%s]: [%s]\n', ...
                      sprintf('%g ', percentiles), ...
                      sprintf('%.4f ', prc_values));
    fprintf('%s\n', prc_str);
end

function print_correlation_table(correlations, param_names)
    % Print correlation coefficients in table format
    
    fprintf('\nCorrelation with Performance Metrics:\n');
    fprintf('%-12s | %-15s | %-15s | %-15s\n', ...
            'Parameter', 'Position RMSE', 'Attitude RMSE', 'Max Pos Error');
    fprintf('%s\n', repmat('-', 1, 68));
    
    for i = 1:length(param_names)
        fprintf('%-12s | %+14.3f | %+14.3f | %+14.3f\n', ...
                param_names{i}, ...
                correlations.rmse_position(i), ...
                correlations.rmse_attitude(i), ...
                correlations.max_position_error(i));
    end
end

function print_sensitivity_table(sensitivity, param_names)
    % Print sensitivity indices
    
    fprintf('Sensitivity Indices (Normalized):\n');
    fprintf('%-12s | %-15s | %-15s\n', ...
            'Parameter', 'Position RMSE', 'Attitude RMSE');
    fprintf('%s\n', repmat('-', 1, 48));
    
    for i = 1:length(param_names)
        fprintf('%-12s | %+14.3f | %+14.3f\n', ...
                param_names{i}, ...
                sensitivity.rmse_position(i), ...
                sensitivity.rmse_attitude(i));
    end
    
    fprintf('\nInterpretation: Larger absolute values indicate higher sensitivity\n');
end

function print_failure_summary(failure_analysis)
    % Print summary of failed trials
    
    fprintf('Unique failure types: %d\n', length(failure_analysis.error_messages));
    fprintf('\nFailure messages:\n');
    for i = 1:length(failure_analysis.error_messages)
        fprintf('  %d. %s\n', i, failure_analysis.error_messages{i});
    end
end

%% ========================================================================
%% HELPER FUNCTIONS - Visualization
%% ========================================================================

function figures = generate_plots(mc_results, param_data, metrics_data, ...
                                 correlations, options)
    % Generate all visualization plots
    
    figures = struct();
    
    % Figure 1: Performance distributions
    figures.distributions = plot_distributions(metrics_data);
    
    % Figure 2: Parameter vs. performance scatter
    figures.scatter = plot_parameter_scatter(param_data, metrics_data);
    
    % Figure 3: Box plots for robustness
    figures.boxplots = plot_boxplots(metrics_data);
    
    % Figure 4: Correlation heatmap
    if options.correlation
        figures.correlation = plot_correlation_heatmap(correlations, param_data.param_names);
    end
    
    % Figure 5: CDFs
    figures.cdfs = plot_cdfs(metrics_data, mc_results.nominal);
end

function fig = plot_distributions(metrics_data)
    % Plot histograms of performance metrics
    
    fig = figure('Name', 'Performance Distributions', 'Position', [100 100 1200 600]);
    
    subplot(2, 3, 1);
    histogram(metrics_data.rmse_position, 30, 'Normalization', 'probability');
    xlabel('Position RMSE (m)');
    ylabel('Probability');
    title('Position Tracking Error');
    grid on;
    
    subplot(2, 3, 2);
    histogram(rad2deg(metrics_data.rmse_attitude), 30, 'Normalization', 'probability');
    xlabel('Attitude RMSE (deg)');
    ylabel('Probability');
    title('Attitude Tracking Error');
    grid on;
    
    subplot(2, 3, 3);
    if isfield(metrics_data, 'control_effort')
        histogram(metrics_data.control_effort, 30, 'Normalization', 'probability');
        xlabel('Control Effort');
        ylabel('Probability');
        title('Total Control Effort');
        grid on;
    end
    
    subplot(2, 3, 4);
    histogram(metrics_data.max_position_error, 30, 'Normalization', 'probability');
    xlabel('Max Position Error (m)');
    ylabel('Probability');
    title('Peak Tracking Error');
    grid on;
    
    subplot(2, 3, 5);
    histogram(rad2deg(metrics_data.max_roll), 30, 'Normalization', 'probability');
    xlabel('Max Roll (deg)');
    ylabel('Probability');
    title('Peak Roll Angle');
    grid on;
    
    subplot(2, 3, 6);
    histogram(rad2deg(metrics_data.max_pitch), 30, 'Normalization', 'probability');
    xlabel('Max Pitch (deg)');
    ylabel('Probability');
    title('Peak Pitch Angle');
    grid on;
end

function fig = plot_parameter_scatter(param_data, metrics_data)
    % Scatter plots of parameters vs. performance
    
    n_params = param_data.n_params;
    n_rows = ceil(n_params / 2);
    
    fig = figure('Name', 'Parameter Sensitivity', 'Position', [100 100 1000 400*n_rows]);
    
    for i = 1:n_params
        subplot(n_rows, 2, i);
        scatter(param_data.matrix(:, i), metrics_data.rmse_position, 20, 'filled', 'MarkerFaceAlpha', 0.6);
        xlabel(param_data.param_names{i});
        ylabel('Position RMSE (m)');
        title(sprintf('%s vs Performance', param_data.param_names{i}));
        grid on;
        
        % Add trend line
        hold on;
        p = polyfit(param_data.matrix(:, i), metrics_data.rmse_position, 1);
        x_fit = linspace(min(param_data.matrix(:, i)), max(param_data.matrix(:, i)), 100);
        plot(x_fit, polyval(p, x_fit), 'r-', 'LineWidth', 2);
        hold off;
    end
end

% function fig = plot_boxplots(metrics_data)
%     % Box plots for robustness visualization
% 
%     fig = figure('Name', 'Robustness Analysis', 'Position', [100 100 1000 400]);
% 
%     data_matrix = [metrics_data.rmse_position, ...
%                    rad2deg(metrics_data.rmse_attitude), ...
%                    metrics_data.max_position_error];
% 
%     boxplot(data_matrix, 'Labels', {'Position RMSE (m)', 'Attitude RMSE (deg)', 'Max Error (m)'});
%     ylabel('Value');
%     title('Performance Metric Distributions');
%     grid on;
% end

function fig = plot_boxplots(metrics_data)
    % Plot robustness box plots
    
    fig = figure('Name', 'MC Boxplots', 'Position', [100 100 1200 500]);
    
    n_trials = length(metrics_data.rmse_position);
    
    % Position RMSE boxplot
    subplot(1,3,1);
    boxplot(metrics_data.rmse_position);
    ylabel('Position RMSE (m)');
    title('Position Tracking Error');
    grid on;
    set(gca, 'XTickLabel', {''});  % Remove x-axis label
    
    % Attitude RMSE boxplot
    subplot(1,3,2);
    boxplot(rad2deg(metrics_data.rmse_attitude));
    ylabel('Attitude RMSE (deg)');
    title('Attitude Tracking Error');
    grid on;
    set(gca, 'XTickLabel', {''});
    
    % Control effort boxplot
    subplot(1,3,3);
    if isfield(metrics_data, 'control_effort')
        boxplot(metrics_data.control_effort);
        ylabel('Control Effort');
        title('Control Effort Distribution');
    else
        text(0.5, 0.5, 'Control effort not available', ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        axis off;
    end
    grid on;
    set(gca, 'XTickLabel', {''});
    
    sgtitle(sprintf('Monte Carlo Performance Distribution (N=%d successful trials)', n_trials), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

function fig = plot_correlation_heatmap(correlations, param_names)
    % Heatmap of parameter-metric correlations
    
    fig = figure('Name', 'Correlation Matrix', 'Position', [100 100 800 600]);
    
    % Build correlation matrix
    corr_matrix = [correlations.rmse_position, ...
                   correlations.rmse_attitude, ...
                   correlations.max_position_error];
    
    imagesc(corr_matrix');
    colorbar;
    colormap(redblue());
    caxis([-1 1]);
    
    set(gca, 'XTick', 1:length(param_names), 'XTickLabel', param_names);
    set(gca, 'YTick', 1:3, 'YTickLabel', {'Pos RMSE', 'Att RMSE', 'Max Error'});
    xlabel('Parameters');
    ylabel('Metrics');
    title('Parameter-Metric Correlations');
    
    % Add text annotations
    for i = 1:length(param_names)
        for j = 1:3
            text(i, j, sprintf('%.2f', corr_matrix(i, j)), ...
                 'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold');
        end
    end
end

function fig = plot_cdfs(metrics_data, nominal)
    % Cumulative distribution functions
    
    fig = figure('Name', 'CDFs', 'Position', [100 100 1200 400]);
    
    subplot(1, 3, 1);
    [f, x] = ecdf(metrics_data.rmse_position);
    plot(x, f, 'LineWidth', 2);
    xlabel('Position RMSE (m)');
    ylabel('Cumulative Probability');
    title('Position RMSE CDF');
    grid on;
    
    if nominal.success
        hold on;
        xline(nominal.results.metrics.tracking.rmse_position, 'r--', 'LineWidth', 2, ...
              'Label', 'Nominal');
        hold off;
    end
    
    subplot(1, 3, 2);
    [f, x] = ecdf(rad2deg(metrics_data.rmse_attitude));
    plot(x, f, 'LineWidth', 2);
    xlabel('Attitude RMSE (deg)');
    ylabel('Cumulative Probability');
    title('Attitude RMSE CDF');
    grid on;
    
    if nominal.success
        hold on;
        xline(rad2deg(nominal.results.metrics.tracking.rmse_attitude), 'r--', 'LineWidth', 2, ...
              'Label', 'Nominal');
        hold off;
    end
    
    subplot(1, 3, 3);
    [f, x] = ecdf(metrics_data.max_position_error);
    plot(x, f, 'LineWidth', 2);
    xlabel('Max Position Error (m)');
    ylabel('Cumulative Probability');
    title('Max Error CDF');
    grid on;
    
    if nominal.success
        hold on;
        xline(nominal.results.metrics.tracking.max_position_error, 'r--', 'LineWidth', 2, ...
              'Label', 'Nominal');
        hold off;
    end
end

function save_figures(figures, plot_dir)
    % Save all figures to disk
    
    if ~exist(plot_dir, 'dir')
        mkdir(plot_dir);
    end
    
    field_names = fieldnames(figures);
    for i = 1:length(field_names)
        fig = figures.(field_names{i});
        filename = fullfile(plot_dir, [field_names{i} '.png']);
        saveas(fig, filename);
        fprintf('  Saved: %s\n', filename);
    end
end

function close_figures_from_struct(figures)
    % Close all figures in a structure
    field_names = fieldnames(figures);
    for i = 1:length(field_names)
        if ishandle(figures.(field_names{i}))
            close(figures.(field_names{i}));
        end
    end
end