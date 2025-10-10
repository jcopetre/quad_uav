function results = simulate_quadrotor_pure(trajectory_file, Q, R, x0, options)
% SIMULATE_QUADROTOR_PURE - Main simulation script for quadrotor trajectory tracking
%
% Production-quality simulation framework for 6DOF quadrotor trajectory tracking
% using LQR control and nonlinear dynamics. Supports batch processing and Monte Carlo.
%
% SYNTAX:
%   simulate_quadrotor_pure(trajectory_file)
%   simulate_quadrotor_pure(trajectory_file, Q, R)
%   simulate_quadrotor_pure(trajectory_file, Q, R, x0)
%   simulate_quadrotor_pure(trajectory_file, Q, R, x0, options)
%   results = simulate_quadrotor_pure(...)
%
% INPUTS:
%   trajectory_file - Filename in ./trajectories/ (e.g., 'basic_maneuver.wpt')
%   Q               - (optional) LQR state weights (12x12), default provided
%   R               - (optional) LQR control weights (4x4), default provided
%   x0              - (optional) Initial state (12x1), default zeros
%                     NOTE: Position (1:3) will be overridden by waypoint start
%   options         - (optional) Struct with fields:
%                     .verbose      - Print progress (default: true)
%                     .save_results - Save to ./results/ (default: true)
%                     .plot         - Generate plots (default: true if no output)
%                     .dt           - Trajectory time step (default: 0.01)
%                     .params       - Pre-designed params (overrides Q, R)
%                                     Use for Monte Carlo with fixed controller
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
%   simulate_quadrotor_pure('basic_maneuver.wpt');
%
%   % Custom tuning, capture results
%   Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
%   results = simulate_quadrotor_pure('hover_test.wpt', Q_aggressive);
%
%   % Start tilted with initial velocity
%   x0 = zeros(12,1);
%   x0(4) = deg2rad(5);   % 5° roll
%   x0(7) = 0.5;          % 0.5 m/s forward velocity
%   simulate_quadrotor_pure('basic_maneuver.wpt', [], [], x0);
%
%   % Batch mode for Monte Carlo
%   opts.verbose = false;
%   opts.plot = false;
%   results = simulate_quadrotor_pure('basic_maneuver.wpt', [], [], [], opts);
%
% See also: quadrotor_linear_6dof, simulate_quadrotor, compute_performance_metrics

% Author: Trey Copeland
% Date: 2025-10-09
    
    init_project();

    %% ========================================================================
    %  STEP 1: INPUT VALIDATION AND DEFAULTS
    %  ========================================================================
    
    % Validate trajectory file
    assert(ischar(trajectory_file) || isstring(trajectory_file), ...
           'trajectory_file must be a string');
    
    % Set default options
    if nargin < 5 || isempty(options)
        options = struct();
    end
    
    if ~isfield(options, 'verbose'),      options.verbose = true; end
    if ~isfield(options, 'save_results'), options.save_results = true; end
    if ~isfield(options, 'plot'),         options.plot = (nargout == 0); end  % Auto: plot if no output
    if ~isfield(options, 'dt'),           options.dt = 0.01; end
    
    % Convenience flag for output
    verbose = options.verbose;
    
    %% ========================================================================
    %  STEP 2: DISPLAY HEADER
    %  ========================================================================
    
    if verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('       Quadrotor 6DOF LQR Control System\n');
        fprintf('       Pure MATLAB Implementation\n');
        fprintf('================================================================\n\n');
    end
    
    %% ========================================================================
    %  STEP 3: LOAD VEHICLE MODEL AND DESIGN LQR CONTROLLER
    %  ========================================================================
    
    if verbose
        fprintf('Step 1/6: Loading quadrotor model and designing controller...\n');
    end
    
    % Check if params provided via options (for Monte Carlo with fixed controller)
    if isfield(options, 'params') && ~isempty(options.params)
        % Use pre-designed params (e.g., fixed K with varied mass)
        params = options.params;
        if verbose
            fprintf('  Using provided params (mass=%.3f kg, K fixed)\n', params.m);
            fprintf('  Controller: Pre-designed (from options)\n');
        end
    else
        % Design new controller from Q, R
        if nargin < 2 || isempty(Q)
            Q = [];  % Use defaults in quadrotor_linear_6dof
        end
        
        if nargin < 3 || isempty(R)
            R = [];  % Use defaults in quadrotor_linear_6dof
        end
        
        % Design LQR controller with provided or default weights
        params = quadrotor_linear_6dof(Q, R, verbose);
        
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
    
    % Construct full path to trajectory file
    trajectory_path = fullfile('./trajectories', trajectory_file);
    
    % Validate file exists
    if ~exist(trajectory_path, 'file')
        error('Trajectory file not found: %s', trajectory_path);
    end
    
    % Load waypoints
    wpt = load_waypoints(trajectory_path);
    
    if verbose
        fprintf('  Loaded: %s\n', wpt.metadata.name);
        fprintf('  Waypoints: %d\n', length(wpt.time));
        fprintf('  Duration: %.1f seconds\n', wpt.time(end) - wpt.time(1));
    end
    
    % Generate smooth trajectory
    if verbose
        fprintf('\nStep 3/6: Preparing trajectory from waypoints...\n');
    end
    
    % Use waypoints directly as sparse trajectory
    % get_reference_state() will interpolate on-the-fly during simulation
    trajectory.time = wpt.time;
    trajectory.position = wpt.position;
    trajectory.velocity = zeros(length(wpt.time), 3);  % Zero velocity at waypoints
    trajectory.attitude = zeros(length(wpt.time), 3);  % Level flight
    trajectory.omega = zeros(length(wpt.time), 3);     % No angular velocity
    
    if verbose
        fprintf('  Trajectory: %d waypoints\n', length(wpt.time));
        fprintf('  Duration: %.1f seconds\n', wpt.time(end) - wpt.time(1));
        fprintf('  Note: Using sparse waypoints; interpolation done on-the-fly\n');
    end
    
    %% ========================================================================
    %  STEP 5: SETUP INITIAL CONDITIONS
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 4/6: Setting up initial conditions...\n');
    end
    
    % Handle initial state
    if nargin >= 4 && ~isempty(x0)
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
    [t, x, u_log] = simulate_quadrotor(x0_full, tspan, params, trajectory, ode_options);
    sim_time = toc;
    
    if verbose
        fprintf('  Simulation complete: %.3f seconds (%.0f time steps)\n', ...
                sim_time, length(t));
        fprintf('  Real-time factor: %.1fx\n', tspan(end) / sim_time);
    end
    
    %% ========================================================================
    %  STEP 7: COMPUTE PERFORMANCE METRICS
    %  ========================================================================
    
    if verbose
        fprintf('\nStep 6/6: Computing performance metrics...\n');
    end
    
    metrics = compute_performance_metrics(t, x, trajectory, params, u_log);
    
    if verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('                    PERFORMANCE SUMMARY\n');
        fprintf('================================================================\n');
        fprintf('Position Tracking:\n');
        fprintf('  RMSE:           %.4f m\n', metrics.tracking.rmse_position);
        fprintf('  Max error:      %.4f m\n', metrics.tracking.max_position_error);
        fprintf('  Time in bounds: %.1f%% (within 10cm)\n', metrics.tracking.time_in_bounds);
        fprintf('\nAttitude:\n');
        fprintf('  RMSE:           %.2f deg\n', rad2deg(metrics.tracking.rmse_attitude));
        fprintf('  Max roll:       %.2f deg\n', rad2deg(metrics.tracking.max_roll));
        fprintf('  Max pitch:      %.2f deg\n', rad2deg(metrics.tracking.max_pitch));
        fprintf('\nControl Effort:\n');
        fprintf('  Total effort:   %.2f\n', metrics.control.total_effort);
        fprintf('  Mean thrust:    %.2f N (hover: %.2f N)\n', ...
                metrics.control.mean_thrust, params.u_hover(1));
        fprintf('  Thrust sat.:    %.1f%% of time\n', metrics.control.thrust_saturation_pct);
        fprintf('  Torque sat.:    %.1f%% of time\n', metrics.control.torque_saturation_pct);
        fprintf('\nSuccess Criteria:\n');
        fprintf('  Completed:      %s\n', bool_to_str(metrics.success.completed));
        fprintf('  Tracking OK:    %s (RMSE < 0.5m)\n', bool_to_str(metrics.success.position_acceptable));
        fprintf('  Attitude safe:  %s (angles < 60deg)\n', bool_to_str(metrics.success.attitude_safe));
        fprintf('  Overall:        %s\n', bool_to_str(metrics.success.overall));
        fprintf('\nSummary Score:    %.3f (lower is better)\n', metrics.summary_weighted);
        fprintf('================================================================\n\n');
    end
    
    %% ========================================================================
    %  STEP 8: VISUALIZATION
    %  ========================================================================
    
    if options.plot
        if verbose
            fprintf('Generating plots...\n');
        end
        
        % Compute reference trajectory for plotting
        x_ref = zeros(length(t), 12);
        for i = 1:length(t)
            x_ref(i, :) = get_reference_state(t(i), trajectory);
        end
        
        % Create comprehensive figure
        fig = figure('Position', [50, 50, 1600, 1000], 'Name', 'Quadrotor Simulation Results');
        
        % 3D Trajectory (top left, spans 2 rows)
        subplot(3, 3, [1, 4]);
        plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 2); hold on;
        plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
              'ro-', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
        plot3(x(1,1), x(1,2), x(1,3), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
        plot3(x(end,1), x(end,2), x(end,3), 'md', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
        grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('3D Trajectory');
        legend('Actual', 'Waypoints', 'Start', 'End', 'Location', 'best');
        view(45, 30);
        
        % Position tracking X
        subplot(3, 3, 2);
        plot(t, x(:,1), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, x_ref(:,1), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('X (m)');
        title('X Position Tracking');
        legend('Actual', 'Reference', 'Location', 'best');
        
        % Position tracking Y
        subplot(3, 3, 3);
        plot(t, x(:,2), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, x_ref(:,2), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Y (m)');
        title('Y Position Tracking');
        legend('Actual', 'Reference', 'Location', 'best');
        
        % Position tracking Z
        subplot(3, 3, 5);
        plot(t, x(:,3), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, x_ref(:,3), 'r--', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Z (m)');
        title('Z Position (Altitude)');
        legend('Actual', 'Reference', 'Location', 'best');
        
        % Attitude angles
        subplot(3, 3, 6);
        plot(t, rad2deg(x(:,4)), 'r-', 'LineWidth', 1.5); hold on;
        plot(t, rad2deg(x(:,5)), 'g-', 'LineWidth', 1.5);
        plot(t, rad2deg(x(:,6)), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time (s)'); ylabel('Angle (deg)');
        title('Attitude Angles');
        legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', 'Location', 'best');
        
        % Linear velocities
        subplot(3, 3, 7);
        plot(t, x(:,7), 'r-', 'LineWidth', 1.5); hold on;
        plot(t, x(:,8), 'g-', 'LineWidth', 1.5);
        plot(t, x(:,9), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        title('Linear Velocities');
        legend('V_x', 'V_y', 'V_z', 'Location', 'best');
        
        % Control inputs - Thrust
        subplot(3, 3, 8);
        plot(t, u_log(:,1), 'b-', 'LineWidth', 1.5); hold on;
        yline(params.u_hover(1), 'r--', 'LineWidth', 1, 'DisplayName', 'Hover');
        yline(params.u_max(1), 'k:', 'LineWidth', 1, 'DisplayName', 'Max');
        grid on;
        xlabel('Time (s)'); ylabel('Thrust (N)');
        title('Thrust Command');
        legend('Location', 'best');
        
        % Control inputs - Torques
        subplot(3, 3, 9);
        plot(t, u_log(:,2), 'r-', 'LineWidth', 1.5); hold on;
        plot(t, u_log(:,3), 'g-', 'LineWidth', 1.5);
        plot(t, u_log(:,4), 'b-', 'LineWidth', 1.5);
        yline(params.u_max(2), 'k:', 'LineWidth', 1);
        yline(params.u_min(2), 'k:', 'LineWidth', 1);
        grid on;
        xlabel('Time (s)'); ylabel('Torque (N·m)');
        title('Control Torques');
        legend('\tau_\phi', '\tau_\theta', '\tau_\psi', 'Location', 'best');
        
        % Overall title
        sgtitle(sprintf('Quadrotor LQR Control - %s', wpt.metadata.name), ...
                'FontSize', 14, 'FontWeight', 'bold');
        
        if verbose
            fprintf('  Plot created successfully\n');
        end
    end
    
    %% ========================================================================
    %  STEP 9: SAVE RESULTS
    %  ========================================================================
    
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    
    if options.save_results
        if verbose
            fprintf('\nSaving results...\n');
        end
        
        % Create results directory if it doesn't exist
        results_dir = './results';
        if ~exist(results_dir, 'dir')
            mkdir(results_dir);
        end
        
        % Generate filename with timestamp  
        [~, traj_name, ~] = fileparts(trajectory_file);
        filename = sprintf('simulation_%s_%s.mat', traj_name, timestamp);
        filepath = fullfile(results_dir, filename);
        
        % Package results
        results_struct.t = t;
        results_struct.x = x;
        results_struct.u_log = u_log;
        results_struct.trajectory = trajectory;
        results_struct.params = params;
        results_struct.metrics = metrics;
        results_struct.config.trajectory_file = trajectory_file;
        results_struct.config.Q = params.Q;
        results_struct.config.R = params.R;
        results_struct.config.x0 = x0_full;
        results_struct.config.options = options;
        results_struct.timestamp = timestamp;
        results_struct.sim_time = sim_time;
        
        % Save to file
        save(filepath, '-struct', 'results_struct');
        
        if verbose
            fprintf('  Results saved to: %s\n', filepath);
            fprintf('  File size: %.2f KB\n', dir(filepath).bytes / 1024);
        end
    end
    
    %% ========================================================================
    %  STEP 10: RETURN RESULTS (if requested)
    %  ========================================================================
    
    if nargout > 0
        results.t = t;
        results.x = x;
        results.u_log = u_log;
        results.trajectory = trajectory;
        results.params = params;
        results.metrics = metrics;
        results.config.trajectory_file = trajectory_file;
        results.config.Q = params.Q;
        results.config.R = params.R;
        results.config.x0 = x0_full;
        results.config.options = options;
        results.timestamp = timestamp;
        results.sim_time = sim_time;
    end
    
    if verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf('                  SIMULATION COMPLETE!\n');
        fprintf('================================================================\n\n');
    end

end

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

function str = bool_to_str(bool_val)
    % BOOL_TO_STR - Convert boolean to YES/NO string
    if bool_val
        str = 'YES';
    else
        str = 'NO';
    end
end