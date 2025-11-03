function [feasible, warnings, violations] = check_trajectory_feasibility(trajectory, params)
% CHECK_TRAJECTORY_FEASIBILITY - Validate trajectory against vehicle/controller limits
%
% Checks if a generated trajectory is within the valid operating region for
% the LQR controller (small-angle linearization) and vehicle actuator limits.
%
% SYNTAX:
%   [feasible, warnings] = check_trajectory_feasibility(trajectory, params)
%
% INPUTS:
%   trajectory - Trajectory structure from generate_trajectory_*
%   params     - Vehicle parameters from quadrotor_linear_6dof
%
% OUTPUTS:
%   feasible - Boolean: true if trajectory is within all limits
%   warnings - Cell array of warning strings describing violations
%
% CHECKS:
%   1. Feedforward attitudes within small-angle approximation (±15°)
%   2. Velocities within reasonable bounds
%   3. Accelerations achievable with max tilt
%   4. Angular velocities within linearization validity
%
% EXAMPLE:
%   traj = generate_trajectory_interp(wpt, params);
%   [ok, warn] = check_trajectory_feasibility(traj, params);
%   if ~ok
%       for i = 1:length(warn)
%           fprintf('⚠ %s\n', warn{i});
%       end
%   end

% Author: Trey Copeland
% Date: 2025-10-10

    warnings = {};
    
    %% Linearization Validity Checks
    SMALL_ANGLE_LIMIT = deg2rad(15);  % 15° for 1% accuracy in sin approximation
    MODERATE_ANGLE_LIMIT = deg2rad(20);  % 20° - marginal validity
    
    % Check feedforward attitudes
    max_roll = max(abs(trajectory.attitude(:,1)));
    max_pitch = max(abs(trajectory.attitude(:,2)));
    max_attitude = max(max_roll, max_pitch);
    
    if max_attitude > SMALL_ANGLE_LIMIT
        pct_violated = 100 * sum(sqrt(sum(trajectory.attitude(:,1:2).^2, 2)) > SMALL_ANGLE_LIMIT) / length(trajectory.time);
        
        if max_attitude > MODERATE_ANGLE_LIMIT
            warnings{end+1} = sprintf(...
                'Feedforward attitudes exceed linearization validity (max: %.1f°, limit: %.1f°). Results may be unreliable.', ...
                rad2deg(max_attitude), rad2deg(SMALL_ANGLE_LIMIT));
        else
            warnings{end+1} = sprintf(...
                'Feedforward attitudes marginally exceed linearization limit (max: %.1f°, safe: <%.1f°). %.1f%% of trajectory affected.', ...
                rad2deg(max_attitude), rad2deg(SMALL_ANGLE_LIMIT), pct_violated);
        end
    end
    
    %% Velocity Checks
    max_velocity = max(sqrt(sum(trajectory.velocity.^2, 2)));
    VELOCITY_WARNING = 3.0;  % m/s - reasonable limit for small quadrotor
    
    if max_velocity > VELOCITY_WARNING
        warnings{end+1} = sprintf(...
            'High velocities detected (max: %.2f m/s). Consider slower maneuvers for better tracking.', ...
            max_velocity);
    end
    
    %% Acceleration Feasibility
    % Max horizontal acceleration from tilting at 30°:
    % a_max = g * tan(30°) ≈ 5.7 m/s²
    max_tilt_safe = deg2rad(30);
    max_accel_safe = params.g * tan(max_tilt_safe);
    
    max_horiz_accel = max(sqrt(sum(trajectory.acceleration(:,1:2).^2, 2)));
    
    if max_horiz_accel > max_accel_safe
        warnings{end+1} = sprintf(...
            'Trajectory demands excessive acceleration (%.2f m/s²) requiring >30° tilt. Max safe: %.2f m/s².', ...
            max_horiz_accel, max_accel_safe);
    end

    %% Yaw Rate and Acceleration Checks (NEW)
    dt = mean(diff(trajectory.time));
    yaw_rate = gradient(trajectory.yaw, dt);
    yaw_accel = gradient(yaw_rate, dt);
    
    % Physical limits
    max_yaw_torque = params.u_max(4);  % N·m
    Izz = params.Izz;  % kg·m²
    max_yaw_accel_physical = max_yaw_torque / Izz;  % rad/s²
    
    % Practical limits for LQR tracking
    YAW_RATE_WARNING = deg2rad(50);   % deg/s - above this, tracking degrades
    YAW_RATE_CRITICAL = deg2rad(100); % deg/s - likely failure
    YAW_ACCEL_LIMIT = 0.7 * max_yaw_accel_physical;  % 70% margin
    YAW_RATE_RMS_LIMIT = deg2rad(20);  % Sustained rate limit
    
    max_yaw_rate = max(abs(yaw_rate));
    max_yaw_accel = max(abs(yaw_accel));
    rms_yaw_rate = rms(yaw_rate);
    
    % Yaw rate checks
    if max_yaw_rate > YAW_RATE_CRITICAL
        warnings{end+1} = sprintf(...
            '🔴 CRITICAL: Yaw rate (%.1f°/s) far exceeds practical limit (%.1f°/s). Trajectory will likely fail.', ...
            rad2deg(max_yaw_rate), rad2deg(YAW_RATE_CRITICAL));
    elseif max_yaw_rate > YAW_RATE_WARNING
        warnings{end+1} = sprintf(...
            '⚠️ High yaw rate (%.1f°/s) exceeds recommended limit (%.1f°/s). May cause tracking issues.', ...
            rad2deg(max_yaw_rate), rad2deg(YAW_RATE_WARNING));
    end
    
    % Sustained yaw rate check
    if rms_yaw_rate > YAW_RATE_RMS_LIMIT
        warnings{end+1} = sprintf(...
            '⚠️ Sustained yaw rate high (%.1f°/s RMS). Recommended: <%.1f°/s for extended maneuvers.', ...
            rad2deg(rms_yaw_rate), rad2deg(YAW_RATE_RMS_LIMIT));
    end
    
    % Yaw acceleration checks
    if max_yaw_accel > max_yaw_accel_physical
        warnings{end+1} = sprintf(...
            '🔴 CRITICAL: Yaw acceleration (%.1f°/s²) exceeds physical limit (%.1f°/s²). Impossible to track.', ...
            rad2deg(max_yaw_accel), rad2deg(max_yaw_accel_physical));
    elseif max_yaw_accel > YAW_ACCEL_LIMIT
        warnings{end+1} = sprintf(...
            '⚠️ Yaw acceleration (%.1f°/s²) high. Recommended: <%.1f°/s² for control margin.', ...
            rad2deg(max_yaw_accel), rad2deg(YAW_ACCEL_LIMIT));
    end
    
    %% Angular Velocity Checks
    if isfield(trajectory, 'omega')
        max_omega = max(sqrt(sum(trajectory.omega.^2, 2)));
        OMEGA_LIMIT = 1.0;  % rad/s (~57 deg/s)
        
        if max_omega > OMEGA_LIMIT
            warnings{end+1} = sprintf(...
                'High angular velocities in reference (%.2f rad/s = %.1f°/s). Linearization may be invalid.', ...
                max_omega, rad2deg(max_omega));
        end
    end
    
    %% Duration Check
    duration = trajectory.time(end) - trajectory.time(1);
    total_distance = 0;
    for i = 2:size(trajectory.position, 1)
        total_distance = total_distance + norm(trajectory.position(i,:) - trajectory.position(i-1,:));
    end
    avg_speed = total_distance / duration;
    
    if avg_speed > 2.0
        warnings{end+1} = sprintf(...
            'Trajectory is aggressive (avg speed: %.2f m/s over %.1f s). Consider longer duration.', ...
            avg_speed, duration);
    end
    
    %% Package Violations Structure
    violations = struct();
    violations.max_attitude = rad2deg(max_attitude);
    violations.max_velocity = max_velocity;
    violations.max_horiz_accel = max_horiz_accel;
    violations.max_yaw_rate = rad2deg(max_yaw_rate);
    violations.rms_yaw_rate = rad2deg(rms_yaw_rate);
    violations.max_yaw_accel = rad2deg(max_yaw_accel);
    violations.avg_speed = avg_speed;
    
    % Limits for reference
    violations.limits = struct();
    violations.limits.small_angle = rad2deg(SMALL_ANGLE_LIMIT);
    violations.limits.velocity_warning = VELOCITY_WARNING;
    violations.limits.accel_safe = max_accel_safe;
    violations.limits.yaw_rate_warning = rad2deg(YAW_RATE_WARNING);
    violations.limits.yaw_rate_critical = rad2deg(YAW_RATE_CRITICAL);
    violations.limits.yaw_accel_physical = rad2deg(max_yaw_accel_physical);
    violations.limits.yaw_accel_recommended = rad2deg(YAW_ACCEL_LIMIT);

    %% Overall Feasibility
    feasible = isempty(warnings);
    
    %% Summary
    if ~feasible
        warnings{end+1} = sprintf(...
            'Recommendation: Increase waypoint spacing, reduce velocities, or use gentler maneuvers.');
    end

end
