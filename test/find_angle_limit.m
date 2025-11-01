% FIND_ANGLE_LIMIT - Empirical analysis of LQR controller operating limits
%
% MOTIVATION:
%   The LQR controller in quadrotor_linear_6dof.m is designed using a 
%   linearized model with small-angle assumptions (sin(θ) ≈ θ, cos(θ) ≈ 1).
%   This script empirically determines:
%     1. At what attitude angles does controller performance degrade?
%     2. What trajectory aggressiveness is safe for the linear controller?
%     3. Is the limitation attitude angles OR something else?
%
% METHOD:
%   - Generate the same trajectory at different "scale factors"
%   - Larger scale = same path geometry, but slower motion (lower acceleration)
%   - Compare tracking performance across scales
%   - Identify the transition point between success and failure
%
% SCALE FACTOR EXPLAINED:
%   Scale = 0.5x → Distances halved, time halved → SAME velocity, HIGHER accel
%   Scale = 1.0x → Original trajectory (baseline)
%   Scale = 2.0x → Distances doubled, time doubled → SAME velocity, LOWER accel
%
%   KEY INSIGHT: Velocity stays constant, but required attitude angles change
%   because attitude ≈ acceleration / g
%
% OUTPUT INTERPRETATION:
%   - "Success" = Meets all criteria (RMSE < 0.5m, angles < 60°, stable)
%   - "Max Roll" = Peak attitude angle during flight (from actual tracking)
%   - "Torque Sat" = % of time control torques hit saturation limits
%   - "RMSE" = Root mean square position tracking error
%
% EXPECTED RESULT:
%   If the linear region hypothesis is correct:
%     → Higher acceleration → larger angles → worse tracking
%     → Performance degrades as angles increase
%
%   If something else is the issue:
%     → Pattern may be different (e.g., velocity limits, saturation, etc.)
%
% SURPRISING FINDING FROM YOUR OUTPUT:
%   The system FAILS at LOWER accelerations (gentler trajectories)!
%   This suggests the issue is NOT attitude angle limits.
%   Possible explanations:
%     1. Trajectory duration exceeds simulation time limit
%     2. Success criteria too strict for slow maneuvers
%     3. Integration errors accumulate over long durations
%     4. Something else in the success metric
%
% USAGE:
%   >> find_angle_limit
%   (Runs automatically, displays results and analysis)


% Author: Trey Copeland
% Date: 2025-10-10

setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

fprintf('=================================================================\n');
fprintf('  EMPIRICAL ANALYSIS: LQR Controller Operating Limits\n');
fprintf('=================================================================\n\n');

fprintf('HYPOTHESIS: Controller fails when attitude angles exceed linear region\n');
fprintf('METHOD:     Scale trajectory → vary acceleration → vary attitude angles\n');
fprintf('PREDICTION: Aggressive (high-accel) trajectories fail, gentle ones succeed\n\n');

fprintf('Testing different trajectory aggressiveness levels...\n\n');

%% Test Setup
% Test different scales of the same trajectory
% Scale factor affects both distance and speed proportionally
% Result: Velocity constant, acceleration varies inversely with scale

scales = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 1.5, 2.0, 2.5];
results = struct();

fprintf('Scale Factor Effect:\n');
fprintf('  0.5x → Path compressed in space & time → Higher accel (2x baseline)\n');
fprintf('  1.0x → Baseline trajectory\n');
fprintf('  2.0x → Path stretched in space & time → Lower accel (0.5x baseline)\n\n');

fprintf('Starting tests...\n');
fprintf('-----------------------------------------------------------------\n\n');

%% Run Scaled Tests
for i = 1:length(scales)
    scale = scales(i);
    
    fprintf('Test %d/%d: Scale factor %.1fx\n', i, length(scales), scale);
    fprintf('  (Expect: ');
    if scale < 1.0
        fprintf('higher acceleration → larger angles → possible failure)\n');
    elseif scale > 1.0
        fprintf('lower acceleration → smaller angles → should succeed)\n');
    else
        fprintf('baseline case)\n');
    end
    
    % Create scaled waypoints
    % Time scales with distance to maintain constant velocity
    waypoints_matrix = [
        0,       0,        0,        1,   0;      % Start at 1m altitude
        3*scale, 0,        0,        1,   0;      % Move forward
        6*scale, 2*scale,  0,        1,   0;      % Move right
        9*scale, 2*scale,  2*scale,  1,   0;      % Move forward again
        12*scale, 0,       0,        1,   0;      % Return
    ];
    
    % Generate trajectory
    traj = generate_trajectory_interp(waypoints_matrix, params, 0.01);
    
    % IMPORTANT: Zero out feedforward attitudes to isolate controller behavior
    % This forces LQR to generate ALL attitude commands (no kinematic hints)
    traj.attitude(:, 1:2) = 0;
    
    % Simulate
    x0 = zeros(12, 1);
    x0(3) = 1;  % Start at 1m altitude
    tspan = [0, 12*scale];  % Limit to 15 sec max
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    
    try
        [t, x, u_log] = ode_simulate(x0, tspan, params, traj, options);
        
        % Compute metrics
        metrics = compute_metrics(t, x, traj, params, u_log);
        
        % Store results
        results(i).scale = scale;
        results(i).duration = tspan(2);
        results(i).max_velocity = max(sqrt(sum(traj.velocity.^2, 2)));
        results(i).max_accel = max(sqrt(sum(traj.acceleration.^2, 2)));
        results(i).rmse = metrics.tracking.rmse_position;
        results(i).max_roll = rad2deg(metrics.tracking.max_roll);
        results(i).max_pitch = rad2deg(metrics.tracking.max_pitch);
        results(i).success = metrics.success.overall;
        results(i).torque_sat = metrics.control.torque_saturation_pct;
        results(i).thrust_sat = metrics.control.thrust_saturation_pct;
        
        fprintf('  TRAJECTORY DEMANDS:\n');
        fprintf('    Max velocity:    %.3f m/s\n', results(i).max_velocity);
        fprintf('    Max accel:       %.3f m/s²\n', results(i).max_accel);
        fprintf('    Duration:        %.1f seconds\n', results(i).duration);
        fprintf('  TRACKING PERFORMANCE:\n');
        fprintf('    Position RMSE:   %.4f m\n', results(i).rmse);
        fprintf('    Max roll:        %.2f deg\n', results(i).max_roll);
        fprintf('    Max pitch:       %.2f deg\n', results(i).max_pitch);
        fprintf('  CONTROL EFFORT:\n');
        fprintf('    Torque sat:      %.1f%%\n', results(i).torque_sat);
        fprintf('    Thrust sat:      %.1f%%\n', results(i).thrust_sat);
        fprintf('  OVERALL SUCCESS:   %s\n\n', iif(results(i).success, '✓ YES', '✗ NO'));
        
    catch ME
        fprintf('  ✗ SIMULATION FAILED: %s\n\n', ME.message);
        results(i).success = false;
        results(i).rmse = inf;
    end
end

%% Summary Table
fprintf('=================================================================\n');
fprintf('  SUMMARY TABLE\n');
fprintf('=================================================================\n\n');
fprintf('Scale   Duration   Max Vel   Max Accel   RMSE      Max Roll   Torque Sat   Success\n');
fprintf('-----   --------   -------   ---------   ------    --------   ----------   -------\n');

for i = 1:length(scales)
    if isfinite(results(i).rmse)
        fprintf('%.2fx    %4.1f s    %.2f m/s   %.2f m/s²   %.4f m  %5.1f°      %4.1f%%       %s\n', ...
                results(i).scale, ...
                results(i).duration, ...
                results(i).max_velocity, ...
                results(i).max_accel, ...
                results(i).rmse, ...
                results(i).max_roll, ...
                results(i).torque_sat, ...
                iif(results(i).success, '✓', '✗'));
    end
end

%% Analysis
fprintf('\n=================================================================\n');
fprintf('  ANALYSIS & INTERPRETATION\n');
fprintf('=================================================================\n\n');

% Find transition point
success_scales = scales([results.success]);
fail_scales = scales(~[results.success]);

if ~isempty(success_scales) && ~isempty(fail_scales)
    max_success_scale = max(success_scales);
    min_fail_scale = min(fail_scales);
    
    fprintf('TRANSITION POINT IDENTIFIED:\n');
    fprintf('  ✓ System succeeds up to %.1fx scale\n', max_success_scale);
    fprintf('  ✗ System fails at %.1fx scale and above\n\n', min_fail_scale);
    
    idx_success = find(scales == max_success_scale);
    idx_fail = find(scales == min_fail_scale);
    
    fprintf('COMPARISON AT TRANSITION:\n');
    fprintf('  Last Success (%.1fx):\n', max_success_scale);
    fprintf('    Velocity:     %.2f m/s\n', results(idx_success).max_velocity);
    fprintf('    Acceleration: %.2f m/s²\n', results(idx_success).max_accel);
    fprintf('    Max Roll:     %.1f°\n', results(idx_success).max_roll);
    fprintf('    Duration:     %.1f seconds\n', results(idx_success).duration);
    fprintf('    RMSE:         %.4f m\n', results(idx_success).rmse);
    fprintf('\n');
    fprintf('  First Failure (%.1fx):\n', min_fail_scale);
    fprintf('    Velocity:     %.2f m/s\n', results(idx_fail).max_velocity);
    fprintf('    Acceleration: %.2f m/s²\n', results(idx_fail).max_accel);
    fprintf('    Max Roll:     %.1f°\n', results(idx_fail).max_roll);
    fprintf('    Duration:     %.1f seconds\n', results(idx_fail).duration);
    fprintf('    RMSE:         %.4f m\n\n', results(idx_fail).rmse);
    
    % Diagnostic analysis
    fprintf('DIAGNOSTIC OBSERVATIONS:\n');
    
    % Check if pattern matches hypothesis
    if results(idx_fail).max_accel < results(idx_success).max_accel
        fprintf('  ⚠ SURPRISING: System fails at LOWER acceleration!\n');
        fprintf('    → Hypothesis REJECTED: Not an attitude angle limit\n');
        fprintf('    → Smaller angles (%.1f°) fail, larger angles (%.1f°) succeed\n', ...
                results(idx_fail).max_roll, results(idx_success).max_roll);
        fprintf('\n  POSSIBLE EXPLANATIONS:\n');
        fprintf('    1. Trajectory duration limit (%.1fs vs %.1fs)\n', ...
                results(idx_fail).duration, results(idx_success).duration);
        fprintf('    2. Success criteria issue (check compute_performance_metrics)\n');
        fprintf('    3. Numerical integration errors over long duration\n');
        fprintf('    4. Simulation time limit truncating trajectory\n');
        
        % Check if duration is the issue
        if results(idx_fail).duration >= 15
            fprintf('\n  → LIKELY CAUSE: Simulation time limit (15s cap)\n');
            fprintf('    Slower trajectories get truncated before completion\n');
        end
    else
        fprintf('  ✓ Pattern matches hypothesis\n');
        fprintf('    → Higher acceleration → larger angles → failure\n');
        fprintf('    → Linear region limit ≈ %.1f° roll angle\n', results(idx_success).max_roll);
    end
    
else
    if isempty(fail_scales)
        fprintf('✓ ALL TESTS PASSED\n');
        fprintf('  System handles all tested scales successfully\n');
        fprintf('  Maximum tested: %.1fx scale, %.1f° roll\n', ...
                max(scales), max([results.max_roll]));
    else
        fprintf('✗ ALL TESTS FAILED\n');
        fprintf('  System cannot handle any tested scale\n');
        fprintf('  Check basic controller functionality\n');
    end
end

fprintf('\n=================================================================\n\n');

%% Helper function
function out = iif(condition, true_val, false_val)
    if condition
        out = true_val;
    else
        out = false_val;
    end
end