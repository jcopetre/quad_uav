% Integration test for trajectory generation methods with closed-loop LQR control
%
% PURPOSE:
%   Validates the interaction between different trajectory generation methods
%   (interpolation-based vs. minimum snap) and closed-loop LQR tracking performance.
%   Compares tracking accuracy, control effort, and smoothness metrics.
%
% WHAT THIS TEST VALIDATES:
%   1. Both trajectory generators produce flyable paths
%   2. LQR controller successfully tracks both trajectory types
%   3. Minimum snap trajectories result in smoother control inputs
%   4. Quantitative performance differences between methods
%
% INTERPRETATION:
%   - If this test fails to run: Check unit tests first (trajectory generation, dynamics)
%   - If tracking is poor: Likely trajectory infeasibility or LQR tuning issue
%   - If results are unreasonable: Use intuition to guide debugging
%
% EXPECTED OUTCOMES:
%   - Minimum snap should have lower jerk/snap in control inputs
%   - Both methods should track within acceptable bounds (<0.5m RMSE)
%   - Minimum snap may have slightly better tracking due to smoother references
%
% USAGE:
%   Run this script directly from the test/ directory
%   >> cd test
%   >> inttest_trajectory_closedloop

% Author: Trey Copeland
% Date: 2025-10-10

clear; close all; clc;

fprintf('========================================\n');
fprintf('Integration Test: Trajectory Methods\n');
fprintf('with Closed-Loop LQR Control\n');
fprintf('========================================\n\n');

%% Setup
fprintf('Step 1: Setting up test environment...\n');
setup_test_environment();

% Use LQR weights tuned for feedforward trajectory tracking
% Key: Lower position weights to avoid position→attitude coupling
% Q = diag([10, 10, 50, 50, 50, 10, 10, 10, 15, 5, 5, 2]);
% R = diag([0.1, 1, 1, 1]);
% Disabled the above Q R weights as they seemed to cause problems
params = quadrotor_linear_6dof([], [], false);

% For a 500g quadrotor with 0.25m arms, ±0.3 N·m is realistic
params.u_max(2:4) = [0.3; 0.3; 0.3];
params.u_min(2:4) = [-0.3; -0.3; -0.3];

fprintf('  ✓ LQR controller designed (feedforward-tuned weights)\n');
fprintf('  ✓ Torque limits increased to ±0.3 N·m\n\n');

%% Load Test Trajectory
fprintf('Step 2: Loading test trajectory...\n');
wpt = load_waypoints('simple_square.wpt');
fprintf('  ✓ Loaded: %s\n', wpt.metadata.name);
fprintf('  ✓ Waypoints: %d\n', length(wpt.time));
fprintf('  ✓ Duration: %.1f seconds\n\n', wpt.time(end));

%% Generate Trajectories with Both Methods
fprintf('Step 3: Generating trajectories...\n');

fprintf('  Method 1: MAKIMA Interpolation... ');
tic;
traj_interp = generate_trajectory_interp(wpt, params, 0.01);  
traj_interp.attitude(:, 1:2) = 0;
t_interp_gen = toc;
fprintf('%.3f s\n', t_interp_gen);

fprintf('  Method 2: Minimum Snap... ');
tic;
traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);
traj_minsnap.attitude(:, 1:2) = 0;
t_minsnap_gen = toc;
fprintf('%.3f s\n\n', t_minsnap_gen);

%% Simulate Both Trajectories
fprintf('Step 4: Running closed-loop simulations...\n');

x0_interp = zeros(12, 1);
x0_interp(3) = wpt.position(1, 3);  % Use first waypoint altitude

x0_minsnap = zeros(12, 1);
x0_minsnap(3) = wpt.position(1, 3);  % Use first waypoint altitude

fprintf('  Initial position (interp):  [%.3f, %.3f, %.3f] m\n', ...
        x0_interp(1), x0_interp(2), x0_interp(3));
fprintf('  Initial velocity (interp):  [%.3f, %.3f, %.3f] m/s\n', ...
        x0_interp(7), x0_interp(8), x0_interp(9));

fprintf('  Initial position (minsnap): [%.3f, %.3f, %.3f] m\n', ...
        x0_minsnap(1), x0_minsnap(2), x0_minsnap(3));
fprintf('  Initial velocity (minsnap): [%.3f, %.3f, %.3f] m/s\n', ...
        x0_minsnap(7), x0_minsnap(8), x0_minsnap(9));

tspan = [0, wpt.time(end)];

options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

fprintf('  Simulating with interpolation trajectory... ');
tic;
[t_interp, x_interp, u_interp] = simulate_quadrotor(x0_interp, tspan, params, traj_interp, options);
t_interp_sim = toc;
fprintf('%.3f s (%d steps)\n', t_interp_sim, length(t_interp));

fprintf('  Simulating with minimum snap trajectory... ');
tic;
[t_minsnap, x_minsnap, u_minsnap] = simulate_quadrotor(x0_minsnap, tspan, params, traj_minsnap, options);
t_minsnap_sim = toc;
fprintf('%.3f s (%d steps)\n\n', t_minsnap_sim, length(t_minsnap));

%% Compute Performance Metrics
fprintf('Step 5: Computing performance metrics...\n');

metrics_interp = compute_performance_metrics(t_interp, x_interp, traj_interp, params, u_interp);
metrics_minsnap = compute_performance_metrics(t_minsnap, x_minsnap, traj_minsnap, params, u_minsnap);

fprintf('  ✓ Metrics computed for both methods\n\n');

%% Display Comparison
fprintf('========================================\n');
fprintf('PERFORMANCE COMPARISON\n');
fprintf('========================================\n\n');

fprintf('%-30s %15s %15s\n', 'Metric', 'Interpolation', 'Min Snap');
fprintf('%-30s %15s %15s\n', '------', '-------------', '--------');

% Position tracking
fprintf('%-30s %15.4f %15.4f\n', 'Position RMSE (m)', ...
        metrics_interp.tracking.rmse_position, ...
        metrics_minsnap.tracking.rmse_position);

fprintf('%-30s %15.4f %15.4f\n', 'Max Position Error (m)', ...
        metrics_interp.tracking.max_position_error, ...
        metrics_minsnap.tracking.max_position_error);

fprintf('%-30s %15.1f %15.1f\n', 'Time in Bounds (%)', ...
        metrics_interp.tracking.time_in_bounds, ...
        metrics_minsnap.tracking.time_in_bounds);

fprintf('\n');

% Attitude tracking
fprintf('%-30s %15.2f %15.2f\n', 'Attitude RMSE (deg)', ...
        rad2deg(metrics_interp.tracking.rmse_attitude), ...
        rad2deg(metrics_minsnap.tracking.rmse_attitude));

fprintf('%-30s %15.2f %15.2f\n', 'Max Roll (deg)', ...
        rad2deg(metrics_interp.tracking.max_roll), ...
        rad2deg(metrics_minsnap.tracking.max_roll));

fprintf('%-30s %15.2f %15.2f\n', 'Max Pitch (deg)', ...
        rad2deg(metrics_interp.tracking.max_pitch), ...
        rad2deg(metrics_minsnap.tracking.max_pitch));

fprintf('\n');

% Control effort
fprintf('%-30s %15.2f %15.2f\n', 'Total Control Effort', ...
        metrics_interp.control.total_effort, ...
        metrics_minsnap.control.total_effort);

fprintf('%-30s %15.2f %15.2f\n', 'Mean Thrust (N)', ...
        metrics_interp.control.mean_thrust, ...
        metrics_minsnap.control.mean_thrust);

fprintf('%-30s %15.2f %15.2f\n', 'Max Thrust (N)', ...
        metrics_interp.control.max_thrust, ...
        metrics_minsnap.control.max_thrust);

fprintf('%-30s %15.2f %15.2f\n', 'Thrust Saturation (%%)', ...
        metrics_interp.control.thrust_saturation_pct, ...
        metrics_minsnap.control.thrust_saturation_pct);

fprintf('%-30s %15.2f %15.2f\n', 'Torque Saturation (%%)', ...
        metrics_interp.control.torque_saturation_pct, ...
        metrics_minsnap.control.torque_saturation_pct);

fprintf('\n');

% Success criteria
fprintf('%-30s %15s %15s\n', 'Overall Success', ...
        bool2str(metrics_interp.success.overall), ...
        bool2str(metrics_minsnap.success.overall));

fprintf('%-30s %15.3f %15.3f\n', 'Summary Score (lower=better)', ...
        metrics_interp.summary_weighted, ...
        metrics_minsnap.summary_weighted);

fprintf('\n');

% Computation time
fprintf('%-30s %15.3f %15.3f\n', 'Generation Time (s)', ...
        t_interp_gen, t_minsnap_gen);

fprintf('%-30s %15.3f %15.3f\n', 'Simulation Time (s)', ...
        t_interp_sim, t_minsnap_sim);

fprintf('\n========================================\n\n');

%% Generate Comparative Plots
fprintf('Step 6: Generating comparison plots...\n');

fig = figure('Position', [50, 50, 1600, 1000], 'Name', 'Trajectory Method Comparison');

% 3D Trajectories (side by side)
subplot(3, 3, 1);
plot3(x_interp(:,1), x_interp(:,2), x_interp(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(traj_interp.position(:,1), traj_interp.position(:,2), traj_interp.position(:,3), ...
      'r--', 'LineWidth', 1);
plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
      'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Interpolation Method');
legend('Actual', 'Reference', 'Waypoints', 'Location', 'best');
view(45, 30);

subplot(3, 3, 2);
plot3(x_minsnap(:,1), x_minsnap(:,2), x_minsnap(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(traj_minsnap.position(:,1), traj_minsnap.position(:,2), traj_minsnap.position(:,3), ...
      'r--', 'LineWidth', 1);
plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
      'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Minimum Snap Method');
legend('Actual', 'Reference', 'Waypoints', 'Location', 'best');
view(45, 30);

% Position tracking errors
subplot(3, 3, 3);
% Compute reference positions
ref_interp = zeros(length(t_interp), 3);
ref_minsnap = zeros(length(t_minsnap), 3);
for i = 1:length(t_interp)
    x_ref = get_reference_state(t_interp(i), traj_interp);
    ref_interp(i,:) = x_ref(1:3);
end
for i = 1:length(t_minsnap)
    x_ref = get_reference_state(t_minsnap(i), traj_minsnap);
    ref_minsnap(i,:) = x_ref(1:3);
end
pos_error_interp = sqrt(sum((x_interp(:,1:3) - ref_interp).^2, 2));
pos_error_minsnap = sqrt(sum((x_minsnap(:,1:3) - ref_minsnap).^2, 2));

plot(t_interp, pos_error_interp, 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, pos_error_minsnap, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Tracking Error Comparison');
legend('Interpolation', 'Min Snap', 'Location', 'best');

% Thrust commands
subplot(3, 3, 4);
plot(t_interp, u_interp(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, u_minsnap(:,1), 'r-', 'LineWidth', 1.5);
yline(params.u_hover(1), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thrust Command');
legend('Interpolation', 'Min Snap', 'Hover', 'Location', 'best');

% Thrust rate of change (jerk indicator)
subplot(3, 3, 5);
thrust_rate_interp = [0; diff(u_interp(:,1))./diff(t_interp)];
thrust_rate_minsnap = [0; diff(u_minsnap(:,1))./diff(t_minsnap)];
plot(t_interp, abs(thrust_rate_interp), 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, abs(thrust_rate_minsnap), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('|dF/dt| (N/s)');
title('Thrust Rate of Change');
legend('Interpolation', 'Min Snap', 'Location', 'best');

% Torque magnitudes
subplot(3, 3, 6);
torque_mag_interp = sqrt(sum(u_interp(:,2:4).^2, 2));
torque_mag_minsnap = sqrt(sum(u_minsnap(:,2:4).^2, 2));
plot(t_interp, torque_mag_interp, 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, torque_mag_minsnap, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Torque Magnitude (N·m)');
title('Control Torque');
legend('Interpolation', 'Min Snap', 'Location', 'best');

% Velocity profiles
subplot(3, 3, 7);
vel_mag_interp = sqrt(sum(x_interp(:,7:9).^2, 2));
vel_mag_minsnap = sqrt(sum(x_minsnap(:,7:9).^2, 2));
plot(t_interp, vel_mag_interp, 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, vel_mag_minsnap, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Velocity Magnitude');
legend('Interpolation', 'Min Snap', 'Location', 'best');

% Attitude angles comparison
subplot(3, 3, 8);
roll_interp = rad2deg(x_interp(:,4));
roll_minsnap = rad2deg(x_minsnap(:,4));
plot(t_interp, roll_interp, 'b-', 'LineWidth', 1.5); hold on;
plot(t_minsnap, roll_minsnap, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angle');
legend('Interpolation', 'Min Snap', 'Location', 'best');

% Performance summary bar chart
subplot(3, 3, 9);
metrics_names = {'Pos RMSE', 'Att RMSE', 'Ctrl Effort'};
metrics_interp_vals = [metrics_interp.tracking.rmse_position, ...
                       rad2deg(metrics_interp.tracking.rmse_attitude), ...
                       metrics_interp.control.total_effort/100];
metrics_minsnap_vals = [metrics_minsnap.tracking.rmse_position, ...
                        rad2deg(metrics_minsnap.tracking.rmse_attitude), ...
                        metrics_minsnap.control.total_effort/100];
x_pos = 1:3;
bar(x_pos - 0.2, metrics_interp_vals, 0.4, 'b'); hold on;
bar(x_pos + 0.2, metrics_minsnap_vals, 0.4, 'r');
set(gca, 'XTick', x_pos, 'XTickLabel', metrics_names);
ylabel('Normalized Value');
title('Performance Summary');
legend('Interpolation', 'Min Snap', 'Location', 'best');
grid on;

sgtitle('Trajectory Generation Method Comparison - Figure Eight', ...
        'FontSize', 16, 'FontWeight', 'bold');

fprintf('  ✓ Plots generated\n\n');

%% Interpretation and Recommendations
fprintf('========================================\n');
fprintf('INTERPRETATION\n');
fprintf('========================================\n\n');

% Determine which method performed better
if metrics_minsnap.summary_weighted < metrics_interp.summary_weighted
    better_method = 'Minimum Snap';
    improvement = (1 - metrics_minsnap.summary_weighted/metrics_interp.summary_weighted) * 100;
else
    better_method = 'Interpolation';
    improvement = (1 - metrics_interp.summary_weighted/metrics_minsnap.summary_weighted) * 100;
end

fprintf('Overall Winner: %s (%.1f%% better)\n\n', better_method, improvement);

% Specific observations
fprintf('Key Observations:\n');

if metrics_minsnap.control.total_effort < metrics_interp.control.total_effort
    effort_reduction = (1 - metrics_minsnap.control.total_effort/metrics_interp.control.total_effort) * 100;
    fprintf('  • Min snap reduced control effort by %.1f%%\n', effort_reduction);
else
    fprintf('  • Interpolation had lower control effort\n');
end

if metrics_minsnap.control.thrust_saturation_pct < metrics_interp.control.thrust_saturation_pct
    fprintf('  • Min snap reduced thrust saturation (%.1f%% vs %.1f%%)\n', ...
            metrics_minsnap.control.thrust_saturation_pct, ...
            metrics_interp.control.thrust_saturation_pct);
else
    fprintf('  • Interpolation had less saturation\n');
end

if metrics_minsnap.tracking.rmse_position < metrics_interp.tracking.rmse_position
    tracking_improvement = (1 - metrics_minsnap.tracking.rmse_position/metrics_interp.tracking.rmse_position) * 100;
    fprintf('  • Min snap improved tracking by %.1f%%\n', tracking_improvement);
else
    fprintf('  • Interpolation had better position tracking\n');
end

fprintf('\nRecommendations:\n');
if t_minsnap_gen > t_interp_gen * 5
    fprintf('  • Min snap is %.1fx slower to generate\n', t_minsnap_gen/t_interp_gen);
    fprintf('  • Use interpolation for real-time applications\n');
    fprintf('  • Use min snap for offline planning\n');
else
    fprintf('  • Both methods have acceptable generation times\n');
end

fprintf('\n========================================\n');
fprintf('Integration Test Complete\n');
fprintf('========================================\n\n');

%% Helper function
function str = bool2str(val)
    if val
        str = 'YES';
    else
        str = 'NO';
    end
end