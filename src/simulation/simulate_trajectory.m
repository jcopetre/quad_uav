function results = simulate_trajectory(trajectory_input, Q, R, x0, options)
% SIMULATE_TRAJECTORY - Main simulation script for quadrotor trajectory tracking
%
% Simulates a quadrotor following a specified trajectory using LQR control.
% Handles waypoint loading, trajectory generation, control design, simulation,
% and performance analysis with comprehensive logging and visualization.
%
% SYNTAX:
%   simulate_trajectory(trajectory_input)
%   simulate_trajectory(trajectory_input, Q)
%   simulate_trajectory(trajectory_input, Q, R)
%   simulate_trajectory(trajectory_input, Q, R, x0)
%   simulate_trajectory(trajectory_input, Q, R, x0, options)
%   results = simulate_trajectory(...)
%
% INPUTS:
%   trajectory_input - EITHER:
%                      (1) Filename string (e.g., 'basic_maneuver.wpt')
%                          Searches in ./trajectories/ directory
%                      (2) Waypoint structure from load_waypoints()
%                          Allows programmatic waypoint generation
%   Q                - (optional) LQR state weight matrix [12x12]
%                      Default: Balanced tracking (see quadrotor_linear_6dof.m)
%   R                - (optional) LQR control weight matrix [4x4]
%                      Default: Moderate control effort penalty
%   x0               - (optional) Initial state vector [12x1]
%                      [x y z roll pitch yaw u v w p q r]'
%                      Default: Start at first waypoint, zero velocity/attitude
%   options          - (optional) Structure with fields:
%                      .verbose       - Print detailed progress (default: true)
%                      .save_results  - Save results to .mat file (default: true)
%                      .plot          - Show plots (default: true if no output)
%                      .dt            - Trajectory timestep (default: 0.01 s)
%                      .output_dir    - Where to save results (default: './results')
%                      .method        - Force trajectory method: 'auto', 'makima', 'minsnap' (default: 'auto')
%                      .params_plant  - Plant parameters if different from nominal
%                                       Primary use case: Monte Carlo simulations where
%                                       you want to test perturbed plant dynamics against
%                                       a nominal (fixed) controller design.
%
% OUTPUTS:
%   results - (optional) Structure containing simulation data:
%             .t, .x, .u_log         - Time, state, control histories
%             .trajectory            - Reference trajectory
%             .params                - Vehicle parameters
%             .metrics               - Performance metrics
%             .config                - Configuration used
%             .timestamp             - When simulation was run
%
% EXAMPLES:
%   % Simple: run with defaults, show plots
%   simulate_trajectory('basic_maneuver.wpt');
%
%   % Programmatic waypoint structure
%   wpt = load_waypoints('figure_eight.wpt');
%   simulate_trajectory(wpt);
%
%   % Custom tuning, capture results
%   Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
%   results = simulate_trajectory('hover_test.wpt', Q_aggressive);
%
%   % Start tilted with initial velocity
%   x0 = zeros(12,1);
%   x0(4) = deg2rad(5);   % 5° roll
%   x0(7) = 0.5;          % 0.5 m/s forward velocity
%   simulate_trajectory('basic_maneuver.wpt', [], [], x0);
%
%   % Batch mode for Monte Carlo with custom output location
%   opts.verbose = false;
%   opts.plot = false;
%   opts.save_results = true;
%   opts.output_dir = './experiments/trajectory_limits/results';
%   results = simulate_trajectory('basic_maneuver.wpt', [], [], [], opts);
%
% See also: quadrotor_linear_6dof, ode_simulate, compute_performance_metrics

% Author: Trey Copeland
% Date: 2025-10-09
    
    init_project();

    %% ========================================================================
    %  STEP 1: INPUT VALIDATION AND DEFAULTS
    %  ========================================================================
    
    % Validate and process trajectory input
    if ischar(trajectory_input) || isstring(trajectory_input)
        % It's a filename - will load later
        trajectory_file = char(trajectory_input);
        wpt = [];  % Will load in Step 4
        is_filename = true;
    elseif isstruct(trajectory_input)
        % It's already a waypoint structure
        if ~all(isfield(trajectory_input, {'time', 'position'}))
            error('Waypoint structure must have ''time'' and ''position'' fields');
        end
        wpt = trajectory_input;
        trajectory_file = '<programmatic>';  % For logging
        is_filename = false;
    else
        error('trajectory_input must be a filename string or waypoint structure');
    end
    
    % Handle optional arguments
    if nargin < 2 || isempty(Q)
        Q = [];  % Use defaults in quadrotor_linear_6dof
    end
    
    if nargin < 3 || isempty(R)
        R = [];  % Use defaults in quadrotor_linear_6dof
    end
    
    if nargin < 4
        x0 = [];  % Will set default later
    end
    
    % Set default options
    if nargin < 5 || isempty(options)
        options = struct();
    end
    
    defaults.verbose = true;
    defaults.save_results = true;
    defaults.plot = (nargout == 0);
    defaults.dt = 0.01;
    defaults.output_dir = './results';  % Default to standard results directory
    defaults.params_plant = [];  % Use same as controller by default
    defaults.method = 'auto';

    options = set_default_options(options, defaults);
    
    verbose = options.verbose;
    
    %% ========================================================================
    %  STEP 2: PRINT HEADER
    %  ========================================================================
    
    if verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('Quadrotor 6DOF LQR Control System\n');
        fprintf('Pure MATLAB Implementation\n');
        fprintf('================================================================\n');
    end
    
    %% ========================================================================
    %  STEP 3: LOAD VEHICLE MODEL AND DESIGN CONTROLLER
    %  ========================================================================
    
    if verbose
        fprintf('Step 1/6: Loading quadrotor model and designing controller...\n');
    end
    
    % Design controller with nominal parameters
    params = quadrotor_linear_6dof(Q, R, verbose);
    
    % Handle plant vs controller parameter mismatch for robustness analysis
    if isfield(options, 'params_plant') && ~isempty(options.params_plant)
        params_plant = options.params_plant;
        if verbose
            fprintf('  Using different plant parameters for robustness testing\n');
            fprintf('  Controller designed for nominal, plant uses perturbed parameters\n');
        end
    else
        params_plant = params;  % No mismatch - controller and plant match
        if verbose
            fprintf('  Vehicle: 500g quadrotor\n');
            fprintf('  Controller: LQR with optimal gains\n');
            fprintf('  Closed-loop poles: %.2f to %.2f (all stable)\n', ...
                    min(real(params.poles)), max(real(params.poles)));
        end
    end
    
    %% ========================================================================
    %  STEP 4: LOAD AND GENERATE TRAJECTORY
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 2/6: Loading trajectory...\n');
    end
    
    % Load waypoints if filename was provided
    if is_filename
        % Construct full path to trajectory file
        % Find project root (where init_project.m lives)
        init_path = which('init_project');
        if isempty(init_path)
            error('Cannot find project root. Run init_project() first.');
        end
        project_root = fileparts(init_path);
        
        % Build absolute path to trajectory
        trajectory_path = fullfile(project_root, 'trajectories', trajectory_file);
        
        % Validate file exists
        if ~exist(trajectory_path, 'file')
            error('Trajectory file not found: %s', trajectory_path);
        end
        
        % Load waypoints
        wpt = load_waypoints(trajectory_path);
        
        if verbose
            fprintf('  Loaded: %s\n', wpt.metadata.name);
        end
    else
        % Using programmatically generated waypoints
        if verbose && isfield(wpt, 'metadata') && isfield(wpt.metadata, 'name')
            fprintf('  Using programmatic waypoints: %s\n', wpt.metadata.name);
        else
            fprintf('  Using programmatically generated waypoints\n');
        end
    end
    
    if verbose
        fprintf('  Waypoints: %d\n', length(wpt.time));
        fprintf('  Duration: %.1f seconds\n', wpt.time(end) - wpt.time(1));
    end
    
    % Generate smooth trajectory using automatic method selection
    if verbose
        fprintf('\nStep 3/6: Generating trajectory...\n');
    end
    
    % SELECT METHOD BASED ON OPTIONS
    if strcmpi(options.method, 'makima')
        trajectory = generate_trajectory_interp(wpt, params, options.dt);
    elseif strcmpi(options.method, 'minsnap')
        trajectory = generate_trajectory_minsnap(wpt, params, options.dt);
    else
        % Auto-selection (default)
        trajectory = generate_trajectory_auto(wpt, params, options.dt);
    end
    
    trajectory.filename = trajectory_file;
    
    if verbose
        fprintf('  Method selected: %s\n', trajectory.method);
        fprintf('  Reason: %s\n', trajectory.method_reason);
        fprintf('  Max velocity: %.2f m/s\n', max(vecnorm(trajectory.velocity, 2, 2)));
        fprintf('  Max acceleration: %.2f m/s²\n', max(vecnorm(trajectory.acceleration, 2, 2)));
    end
    
    [feasible, warnings, violations] = check_trajectory_feasibility(trajectory, params);

    % Store in trajectory for later reporting
    trajectory.feasibility = struct(...
        'feasible', feasible, ...
        'warnings', {warnings}, ...
        'violations', violations);
    
    % Print warnings if verbose
    if options.verbose && ~feasible
        fprintf('\n⚠️⚠️⚠️ TRAJECTORY FEASIBILITY WARNINGS ⚠️⚠️⚠️\n');
        for i = 1:length(warnings)
            fprintf('%s\n', warnings{i});
        end
        fprintf('Simulation will proceed, but tracking may be poor.\n\n');
    end

    %% ========================================================================
    %  STEP 5: SETUP INITIAL CONDITIONS
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 4/6: Setting up initial conditions...\n');
    end
    
    % Handle initial state
    if ~isempty(x0)
        % User provided initial state
        assert(length(x0) == 12, 'x0 must be 12x1 state vector');
        
        x0_full = zeros(12, 1);
        x0_full(1:3) = wpt.position(1, :)';  % Position from waypoint (force consistency)
        x0_full(4:12) = x0(4:12);            % Attitude, velocity, omega from user
        
        % Warn if position was overridden
        if norm(x0(1:3) - wpt.position(1,:)') > 0.01
            if verbose
                fprintf('  WARNING: Initial position overridden to match waypoint start\n');
                fprintf('           Starting at [%.2f, %.2f, %.2f] m\n', ...
                        wpt.position(1,1), wpt.position(1,2), wpt.position(1,3));
            end
        end
    else
        % Default: start at waypoint with zero velocity/attitude
        x0_full = zeros(12, 1);
        x0_full(1:3) = wpt.position(1, :)';
    end
    
    if verbose
        fprintf('  Initial state:\n');
        fprintf('    Position:  [%.2f, %.2f, %.2f] m\n', x0_full(1), x0_full(2), x0_full(3));
        fprintf('    Attitude:  [%.2f, %.2f, %.2f] deg\n', ...
                rad2deg(x0_full(4)), rad2deg(x0_full(5)), rad2deg(x0_full(6)));
        fprintf('    Velocity:  [%.2f, %.2f, %.2f] m/s\n', x0_full(7), x0_full(8), x0_full(9));
    end
    
    %% ========================================================================
    %  STEP 6: RUN SIMULATION
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 5/6: Running ODE simulation...\n');
    end
    
    tspan = [trajectory.time(1), trajectory.time(end)];
    ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    
    tic;
    [t, x, u_log] = ode_simulate(x0_full, tspan, params_plant, trajectory, ode_options);
    sim_time = toc;
    
    if verbose
        fprintf('  Simulation complete: %.3f seconds (%.0f time steps)\n', ...
                sim_time, length(t));
    end
    
    %% ========================================================================
    %  STEP 7: COMPUTE PERFORMANCE METRICS
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 6/6: Computing performance metrics...\n');
    end
    
    metrics = compute_performance_metrics(t, x, trajectory, params, u_log);
    
    if verbose
        fprintf('  Position RMSE: %.4f m\n', metrics.tracking.rmse_position);
        fprintf('  Attitude RMSE: %.2f deg\n', rad2deg(metrics.tracking.rmse_attitude));
        fprintf('  Control effort: %.2f\n', metrics.control.total_effort);
        fprintf('  Overall success: %s\n', bool2str(metrics.success.overall));
    end
    
    %% ========================================================================
    %  STEP 8: PRINT PERFORMANCE SUMMARY
    %  ========================================================================
    
    if verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('PERFORMANCE SUMMARY\n');
        fprintf('================================================================\n');
        fprintf('Position Tracking:\n');
        fprintf('  RMSE:             %.4f m\n', metrics.tracking.rmse_position);
        fprintf('  Max error:        %.4f m\n', metrics.tracking.max_position_error);
        fprintf('  Time in bounds:   %.1f%% (within 10cm)\n', ...
                metrics.tracking.time_in_bounds * 100);
        fprintf('\n');
        fprintf('Attitude:\n');
        fprintf('  RMSE:             %.2f deg\n', rad2deg(metrics.tracking.rmse_attitude));
        fprintf('  Max roll:         %.2f deg\n', rad2deg(metrics.tracking.max_roll));
        fprintf('  Max pitch:        %.2f deg\n', rad2deg(metrics.tracking.max_pitch));
        fprintf('\n');
        fprintf('Control Effort:\n');
        fprintf('  Total effort:     %.2f\n', metrics.control.total_effort);
        fprintf('  Mean thrust:      %.2f N (hover: %.2f N)\n', ...
                metrics.control.mean_thrust, params.u_hover(1));
        fprintf('  Thrust sat.:      %.1f%% of time\n', ...
                metrics.control.thrust_saturation_pct);
        fprintf('  Torque sat.:      %.1f%% of time\n', ...
                metrics.control.torque_saturation_pct);
        fprintf('\n');
        fprintf('Success Criteria:\n');
        fprintf('  Completed:        %s\n', bool2str(metrics.success.completed));
        fprintf('  Tracking OK:      %s (RMSE < 0.5m)\n', ...
                bool2str(metrics.success.position_acceptable));
        fprintf('  Attitude safe:    %s (angles < 60deg)\n', ...
                bool2str(metrics.success.attitude_safe));
        fprintf('  Overall:          %s\n', bool2str(metrics.success.overall));
        fprintf('\n');
        fprintf('Summary Score: %.3f (lower is better)\n', metrics.summary_weighted);
        fprintf('================================================================\n');
        fprintf('\n');
    end
    
%% ========================================================================
    %  STEP 9: SAVE RESULTS
    %  ========================================================================
    
    results = struct();
    results.t = t;
    results.x = x;
    results.u_log = u_log;
    results.trajectory = trajectory;
    results.params = params;
    results.params_plant = params_plant;
    results.metrics = metrics;
    results.config = struct('Q', params.Q, 'R', params.R, ...
                           'x0', x0_full, 'dt', options.dt);
    results.timestamp = datetime('now');

    if options.save_results
        % Create output directory
        output_dir = options.output_dir;
        if ~exist(output_dir, 'dir')
            mkdir(output_dir);
        end
        
        % Generate label
        if is_filename
            [~, label, ~] = fileparts(trajectory_file);
        else
            if isfield(wpt, 'metadata') && isfield(wpt.metadata, 'name')
                label = matlab.lang.makeValidName(wpt.metadata.name);
            else
                label = 'programmatic_trajectory';
            end
        end
        
        % Create results subdirectory (matches MC structure)
        timestamp = datestr(results.timestamp, 'yyyymmdd_HHMMSS');
        run_dir = fullfile(output_dir, sprintf('%s_%s', label, timestamp));
        if ~exist(run_dir, 'dir')
            mkdir(run_dir);
            results.output_dir = run_dir;
        end       
        
        % Save .mat file IN THE SUBDIRECTORY
        DataManager.save_results(results, 'results', run_dir, ...
            struct('verbose', verbose, 'timestamp', false));
        
        % Write metrics report IN THE SUBDIRECTORY
        metrics_file = fullfile(run_dir, Constants.ANALYSIS_REPORT);
        write_unified_metrics_report(results, [], metrics_file);
        
        % Copy waypoint file to results directory for reproducibility
        if is_filename && exist(trajectory_path, 'file')
            wpt_dest = fullfile(run_dir, trajectory_file);
            copyfile(trajectory_path, wpt_dest);
            if verbose
                fprintf('  Waypoints: %s\n', wpt_dest);
            end
        end

        if verbose
            fprintf('Results saved: %s\n', run_dir);
            fprintf('  Data:    %s\n', fullfile(run_dir, Constants.SINGLE_RUN_DATA));
            fprintf('  Metrics: %s\n\n', metrics_file);
        end
    end
    
    %% ========================================================================
    %  STEP 10: GENERATE PLOTS
    %  ========================================================================
    
    if options.plot
        if verbose
            fprintf('Generating plots...\n');
        end
        
        % Create figure with subplots
        fig = figure('Position', [100, 100, 1400, 900], ...
                     'Name', sprintf('Trajectory Tracking: %s', trajectory_file));
        
        % Position tracking
        subplot(3, 3, 1);
        plot(t, x(:,1), 'b-', 'LineWidth', 1.5); hold on;
        plot(trajectory.time, trajectory.position(:,1), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('X Position (m)');
        title('X Position Tracking');
        legend('Actual', 'Reference', 'Location', 'best');
        
        subplot(3, 3, 2);
        plot(t, x(:,2), 'b-', 'LineWidth', 1.5); hold on;
        plot(trajectory.time, trajectory.position(:,2), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Y Position (m)');
        title('Y Position Tracking');
        legend('Actual', 'Reference', 'Location', 'best');
        
        subplot(3, 3, 3);
        plot(t, x(:,3), 'b-', 'LineWidth', 1.5); hold on;
        plot(trajectory.time, trajectory.position(:,3), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Z Position (m)');
        title('Z Position (Altitude) Tracking');
        legend('Actual', 'Reference', 'Location', 'best');
        
        % 3D Trajectory
        subplot(3, 3, 4);
        plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 2); hold on;
        plot3(trajectory.position(:,1), trajectory.position(:,2), ...
              trajectory.position(:,3), 'r--', 'LineWidth', 1);
        plot3(x(1,1), x(1,2), x(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(x(end,1), x(end,2), x(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('3D Trajectory');
        legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');
        view(45, 30);
        
        % Attitude angles
        subplot(3, 3, 5);
        plot(t, rad2deg(x(:,4)), 'LineWidth', 1.5); hold on;
        plot(t, rad2deg(x(:,5)), 'LineWidth', 1.5);
        plot(t, rad2deg(x(:,6)), 'LineWidth', 1.5);
        grid on;
        xlabel('Time (s)'); ylabel('Angle (deg)');
        title('Attitude Angles');
        legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
        
        % Velocities
        subplot(3, 3, 6);
        plot(t, x(:,7:9), 'LineWidth', 1.5);
        grid on;
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        title('Body Velocities');
        legend('u', 'v', 'w', 'Location', 'best');
        
        % Control inputs - Thrust
        subplot(3, 3, 7);
        plot(t, u_log(:,1), 'b-', 'LineWidth', 1.5); hold on;
        yline(params.u_hover(1), 'r--', 'Hover', 'LineWidth', 1);
        yline(params.u_min(1), 'k:', 'Min', 'LineWidth', 1);
        yline(params.u_max(1), 'k:', 'Max', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Thrust (N)');
        title('Thrust Command');
        ylim([0, params.u_max(1) * 1.1]);
        
        % Control inputs - Torques
        subplot(3, 3, 8);
        plot(t, u_log(:,2:4), 'LineWidth', 1.5); hold on;
        yline(params.u_min(2), 'k:', 'Min', 'LineWidth', 1);
        yline(params.u_max(2), 'k:', 'Max', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Torque (N·m)');
        title('Torque Commands');
        legend('\tau_x (roll)', '\tau_y (pitch)', '\tau_z (yaw)', ...
               'Location', 'best');
        
        % Position error
        subplot(3, 3, 9);
        x_ref_interp = interp1(trajectory.time, trajectory.position, t);
        pos_error = vecnorm(x(:,1:3) - x_ref_interp, 2, 2);
        plot(t, pos_error * 100, 'b-', 'LineWidth', 1.5); hold on;
        yline(10, 'r--', '10cm threshold', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Position Error (cm)');
        title('Position Tracking Error');
        
        sgtitle(sprintf('Trajectory Tracking Results: %s', trajectory_file), ...
                'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'none');
        
        if verbose
            fprintf('  Plots generated\n\n');
        end
    end    
end

function str = bool2str(value)
    if value
        str = 'YES';
    else
        str = 'NO';
    end
end