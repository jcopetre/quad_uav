function metrics = compute_metrics(t, x, trajectory, params, u_log)
% COMPUTE_PERFORMANCE_METRICS - Standardized performance evaluation
%
% Computes comprehensive performance metrics for trajectory tracking.
% Used for both single simulations and Monte Carlo analysis.
%
% SYNTAX:
%   metrics = compute_metrics(t, x, trajectory, params)
%   metrics = compute_metrics(t, x, trajectory, params, u_log)
%
% INPUTS:
%   t          - Time vector (Nx1) [s]
%   x          - State history (Nx12) [states at each time]
%   trajectory - Reference trajectory structure
%   params     - Vehicle parameters structure
%   u_log      - (optional) Control input history (Nx4) [F, tau_x, tau_y, tau_z]
%
% OUTPUTS:
%   metrics - Structure containing:
%             .tracking      - Tracking error metrics
%             .control       - Control effort metrics (if u_log provided)
%             .stability     - Stability indicators
%             .success       - Success/failure flags
%             .summary       - Single-number aggregate score
%
% EXAMPLE:
%   [t, x] = ode45(...);
%   metrics = compute_metrics(t, x, trajectory, params);
%   fprintf('RMSE: %.4f m\n', metrics.tracking.rmse_position);

% Author: Trey Copeland
% Date: 2025-10-09

    %% Input validation
    assert(size(x, 2) == 12, 'State history must have 12 columns');
    assert(length(t) == size(x, 1), 'Time and state dimensions must match');
    
    %% Extract reference trajectory at simulation times
    n = length(t);
    x_ref = zeros(n, 12);
    for i = 1:n
        x_ref(i, :) = get_reference_state(t(i), trajectory);
    end
    
    %% Position Tracking Metrics
    pos_error = x(:, 1:3) - x_ref(:, 1:3);  % [x, y, z] error
    
    metrics.tracking.rmse_position = sqrt(mean(sum(pos_error.^2, 2)));  % Overall
    metrics.tracking.rmse_xyz = sqrt(mean(pos_error.^2, 1));  % [x, y, z] individual
    metrics.tracking.max_position_error = max(sqrt(sum(pos_error.^2, 2)));
    metrics.tracking.final_position_error = norm(pos_error(end, :));
    
    % Time-in-bounds (percentage of time within tolerance)
    tolerance = 0.1;  % 10 cm
    in_bounds = sqrt(sum(pos_error.^2, 2)) < tolerance;
    metrics.tracking.time_in_bounds = 100 * sum(in_bounds) / n;  % Percentage
    
    %% Attitude Tracking Metrics
    att_error = x(:, 4:6) - x_ref(:, 4:6);  % [phi, theta, psi] error (rad)
    
    metrics.tracking.rmse_attitude = sqrt(mean(sum(att_error.^2, 2)));  % Overall (rad)
    metrics.tracking.rmse_rpy = sqrt(mean(att_error.^2, 1));  % [roll, pitch, yaw] individual
    metrics.tracking.max_roll = max(abs(x(:, 4)));
    metrics.tracking.max_pitch = max(abs(x(:, 5)));
    metrics.tracking.max_yaw = max(abs(x(:, 6)));
    
    %% Velocity Tracking Metrics
    vel_error = x(:, 7:9) - x_ref(:, 7:9);  % [vx, vy, vz] error
    
    metrics.tracking.rmse_velocity = sqrt(mean(sum(vel_error.^2, 2)));
    metrics.tracking.max_velocity = max(sqrt(sum(x(:, 7:9).^2, 2)));
    
    %% Control Effort Metrics (if control log provided)
    if nargin >= 5 && ~isempty(u_log)
        assert(size(u_log, 1) == n, 'Control log length must match state history');
        
        % Total control effort: ∫(F² + τ²) dt
        % Units: N²·s + (N·m)²·s (mixed - NOT physically meaningful)
        % Note: Dominated by thrust term (>99%), used for trending only
        metrics.control.total_effort = trapz(t, sum(u_log.^2, 2));
        metrics.control.total_effort_units = 'N²·s + (N·m)²·s (mixed)';
        metrics.control.total_effort_note = 'For comparison only; dominated by thrust';
        
        % Add RMS values (physically meaningful)
        metrics.control.rms_thrust = rms(u_log(:,1));
        metrics.control.rms_roll_torque = rms(u_log(:,2));
        metrics.control.rms_pitch_torque = rms(u_log(:,3));
        metrics.control.rms_yaw_torque = rms(u_log(:,4));
        
        % Average control magnitudes
        metrics.control.mean_thrust = mean(u_log(:, 1));
        metrics.control.mean_torque = mean(sqrt(sum(u_log(:, 2:4).^2, 2)));
        
        % Peak control usage
        metrics.control.max_thrust = max(u_log(:, 1));
        metrics.control.max_torque = max(sqrt(sum(u_log(:, 2:4).^2, 2)));
        
        % Saturation analysis
        thrust_saturated = (u_log(:,1) >= params.u_max(1) * 0.99) | ...
                           (u_log(:,1) <= params.u_min(1) * 1.01);
        torque_saturated = any(abs(u_log(:,2:4)) >= abs(params.u_max(2:4))' * 0.99, 2);
        
        metrics.control.thrust_saturation_pct = 100 * sum(thrust_saturated) / n;
        metrics.control.torque_saturation_pct = 100 * sum(torque_saturated) / n;
        metrics.control.num_saturation_events = sum(thrust_saturated | torque_saturated);
    else
        metrics.control = struct();  % Empty if no control log
    end
    
    %% Stability Metrics
    % Settling time (time to reach 5% of final error)
    final_error = norm(pos_error(end, :));
    settling_threshold = max(0.05 * metrics.tracking.max_position_error, 0.02);  % 5% or 2cm
    settled = sqrt(sum(pos_error.^2, 2)) < settling_threshold;
    
    if any(settled)
        first_settled = find(settled, 1);
        % Check if it stays settled (no more than 10% violations)
        violations = sum(~settled(first_settled:end));
        if violations < 0.1 * (n - first_settled)
            metrics.stability.settling_time = t(first_settled);
        else
            metrics.stability.settling_time = NaN;  % Never truly settled
        end
    else
        metrics.stability.settling_time = NaN;
    end
    
    % Overshoot (max error relative to final trajectory point)
    ref_final = x_ref(end, 1:3);
    max_deviation = max(sqrt(sum((x(:,1:3) - ref_final).^2, 2)));
    metrics.stability.overshoot = max_deviation / norm(ref_final);
    
    %% Linearization Validity Metrics
    % The LQR controller is designed using small-angle linearization around hover.
    % Track violations of linearization assumptions to warn users when results
    % may be unreliable.
    
    % Small-angle approximation validity: |θ| < 15° (sin(θ) ≈ θ within 1%)
    SMALL_ANGLE_LIMIT = deg2rad(15);  % 15 degrees
    
    % Angular velocity limit for linearization: |ω| < 1 rad/s (~57 deg/s)
    ANGULAR_VEL_LIMIT = 1.0;  % rad/s
    
    % Check reference trajectory demands
    ref_attitudes = zeros(length(t), 3);
    for i = 1:length(t)
        x_ref_i = get_reference_state(t(i), trajectory);
        ref_attitudes(i, :) = x_ref_i(4:6);
    end
    
    metrics.linearity.max_ref_roll = max(abs(ref_attitudes(:,1)));
    metrics.linearity.max_ref_pitch = max(abs(ref_attitudes(:,2)));
    metrics.linearity.max_ref_attitude = max(abs(ref_attitudes(:,1:2)), [], 'all');
    
    metrics.linearity.max_actual_roll = max(abs(x(:,4)));
    metrics.linearity.max_actual_pitch = max(abs(x(:,5)));
    metrics.linearity.max_actual_attitude = max(abs(x(:,4:5)), [], 'all');
    
    metrics.linearity.max_angular_velocity = max(sqrt(sum(x(:,10:12).^2, 2)));
    
    % Violation flags
    metrics.linearity.ref_exceeds_small_angle = (metrics.linearity.max_ref_attitude > SMALL_ANGLE_LIMIT);
    metrics.linearity.actual_exceeds_small_angle = (metrics.linearity.max_actual_attitude > SMALL_ANGLE_LIMIT);
    metrics.linearity.angular_vel_excessive = (metrics.linearity.max_angular_velocity > ANGULAR_VEL_LIMIT);
    
    % Overall linearity violation flag
    metrics.linearity.violated = metrics.linearity.ref_exceeds_small_angle || ...
                                  metrics.linearity.actual_exceeds_small_angle || ...
                                  metrics.linearity.angular_vel_excessive;
    
    % Compute percentage of time in violation
    ref_attitude_magnitude = sqrt(sum(ref_attitudes(:,1:2).^2, 2));
    actual_attitude_magnitude = sqrt(sum(x(:,4:5).^2, 2));
    angular_vel_magnitude = sqrt(sum(x(:,10:12).^2, 2));
    
    metrics.linearity.time_ref_violated_pct = 100 * sum(ref_attitude_magnitude > SMALL_ANGLE_LIMIT) / length(t);
    metrics.linearity.time_actual_violated_pct = 100 * sum(actual_attitude_magnitude > SMALL_ANGLE_LIMIT) / length(t);
    metrics.linearity.time_ang_vel_violated_pct = 100 * sum(angular_vel_magnitude > ANGULAR_VEL_LIMIT) / length(t);
    
    % Severity assessment
    if metrics.linearity.violated
        if metrics.linearity.max_actual_attitude > deg2rad(30)
            metrics.linearity.severity = 'SEVERE';  % >30° - linearization completely invalid
        elseif metrics.linearity.max_actual_attitude > deg2rad(20)
            metrics.linearity.severity = 'MODERATE';  % 20-30° - results questionable
        else
            metrics.linearity.severity = 'MILD';  % 15-20° - marginal validity
        end
    else
        metrics.linearity.severity = 'NONE';
    end

    %% Success Criteria
    % Define success as meeting all critical thresholds
    metrics.success.completed = (t(end) >= trajectory.time(end) * 0.99);  % Reached end time
    metrics.success.position_acceptable = (metrics.tracking.rmse_position < 0.5);  % <50cm RMSE
    metrics.success.attitude_safe = (max(abs(x(:,4:6)), [], 'all') < deg2rad(60));  % <60° angles, max over all elements
    metrics.success.velocity_bounded = (max(sqrt(sum(x(:,7:9).^2, 2))) < 15);  % <15 m/s
    metrics.success.no_divergence = all(sqrt(sum(x(:,1:3).^2, 2)) < 100);  % Within 100m
    
    metrics.success.overall = all(structfun(@(x) x, metrics.success));
    
    %% Summary Scalar (for ranking/sorting)
    % Weighted combination of key metrics (lower is better)
    % Normalize each metric to [0, 1] range based on "acceptable" thresholds
    
    w_pos = 1.0;    % Position tracking weight
    w_att = 0.3;    % Attitude tracking weight
    w_ctrl = 0.2;   % Control effort weight
    
    pos_score = min(metrics.tracking.rmse_position / 0.5, 1.0);  % Normalize by 50cm
    att_score = min(metrics.tracking.rmse_attitude / deg2rad(10), 1.0);  % Normalize by 10°
    
    if isfield(metrics.control, 'total_effort') && ~isempty(metrics.control.total_effort)
        % Normalize by "reasonable" control effort (problem-dependent)
        nominal_effort = params.m * params.g * trajectory.time(end);  % Hovering for duration
        ctrl_score = min(metrics.control.total_effort / (2*nominal_effort), 1.0);
    else
        ctrl_score = 0;  % No penalty if control not logged
    end
    
    metrics.summary = w_pos * pos_score + w_att * att_score + w_ctrl * ctrl_score;
    metrics.summary_weighted = metrics.summary / (w_pos + w_att + w_ctrl);  % Normalize to [0,1]
end