% quick_test_simulation with GENERATED trajectory
% This tests if the trajectory generators are the problem

clear; close all; clc;

setup_test_environment();

fprintf('========================================\n');
fprintf('Quadrotor Quick Test - Generated Traj\n');
fprintf('========================================\n\n');

%% Step 1: Load vehicle model and LQR controller
fprintf('Step 1: Loading quadrotor model...\n');

% Same weights as original quick_test_sim
Q = diag([200 200 200 10 10 1 20 20 20 1 1 0.1]);
R = diag([0.1 1 1 1]);

params = quadrotor_linear_6dof(Q, R, false);
fprintf('  Using aggressive position tracking gains\n');

%% Step 2: Create waypoints and generate trajectory
fprintf('Step 2: Creating waypoints and generating trajectory...\n');

% Same waypoints as original quick_test_sim, but in waypoint structure
waypoints_matrix = [
    0,   0,  0,  0,   0;      % time, x, y, z, yaw
    3,   0,  0,  1,   0;
    7,   2,  0,  1,   0;
    11,  2,  2,  1,   0;
    15,  2,  2,  0.5, 0
];

% Generate trajectory using interpolation (no feedforward attitudes)
fprintf('  Generating with MAKIMA interpolation...\n');
trajectory_gen = generate_trajectory_interp(waypoints_matrix, params, 0.01);

% Override: Force zero feedforward attitudes (like original quick_test)
fprintf('  Disabling feedforward attitudes...\n');
trajectory_gen.attitude(:, 1:2) = 0;

fprintf('  Generated trajectory: %d points\n', length(trajectory_gen.time));

%% Step 3: Initial conditions
fprintf('Step 3: Setting initial conditions...\n');
x0 = zeros(12, 1);  % Start at origin, stationary
fprintf('  Starting at origin, level, stationary\n');

%% Step 4: Run simulation
fprintf('Step 4: Running ODE simulation...\n');
tspan = [0, trajectory_gen.time(end)];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

tic;
[t, x, u_log] = ode_simulate(x0, tspan, params, trajectory_gen, options);
sim_time = toc;

fprintf('  Simulation complete: %.3f seconds (%.0f time steps)\n', ...
        sim_time, length(t));

%% Step 5: Compute tracking error
fprintf('Step 5: Computing tracking error...\n');

% Compute metrics
metrics = compute_metrics(t, x, trajectory_gen, params, u_log);

fprintf('  Position RMSE: %.4f m\n', metrics.tracking.rmse_position);

%% Step 6: Create plots
fprintf('Step 6: Generating plots...\n');

% Get reference trajectory for plotting
x_ref = zeros(length(t), 12);
for i = 1:length(t)
    x_ref(i, :) = get_reference_state(t(i), trajectory_gen);
end

figure('Position', [100, 100, 1400, 900]);

% Position tracking - X (top left)
subplot(2, 3, 1);
plot(t, x(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, x_ref(:,1), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position Tracking');
legend('Actual', 'Reference', 'Location', 'best');

% Position tracking - Y (top middle)
subplot(2, 3, 2);
plot(t, x(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t, x_ref(:,2), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position Tracking');
legend('Actual', 'Reference', 'Location', 'best');

% Position tracking - Z (top right)
subplot(2, 3, 3);
plot(t, x(:,3), 'b-', 'LineWidth', 2); hold on;
plot(t, x_ref(:,3), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Z Position (Altitude) Tracking');
legend('Actual', 'Reference', 'Location', 'best');

% 3D Trajectory (bottom left)
subplot(2, 3, 4);
plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(waypoints_matrix(:,2), waypoints_matrix(:,3), waypoints_matrix(:,4), ...
      'ro-', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);
plot3(x(1,1), x(1,2), x(1,3), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot3(x(end,1), x(end,2), x(end,3), 'md', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'LineWidth', 2);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory');
legend('Actual Path', 'Waypoints', 'Start', 'End', 'Location', 'best');
axis equal;
view(45, 30);

% Attitude (bottom middle)
subplot(2, 3, 5);
plot(t, rad2deg(x(:,4)), 'r-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x(:,5)), 'g-', 'LineWidth', 1.5);
plot(t, rad2deg(x(:,6)), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude Angles');
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', 'Location', 'best');

% Velocities (bottom right)
subplot(2, 3, 6);
plot(t, x(:,7), 'r-', 'LineWidth', 1.5); hold on;
plot(t, x(:,8), 'g-', 'LineWidth', 1.5);
plot(t, x(:,9), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Linear Velocities');
legend('V_x', 'V_y', 'V_z', 'Location', 'best');

sgtitle('Quadrotor LQR Control - Generated Trajectory Test', 'FontSize', 16, 'FontWeight', 'bold');

%% Step 7: Output Performance Metrics
fprintf('\n========================================\n');
fprintf('RESULTS COMPARISON\n');
fprintf('========================================\n');

fprintf('\nGenerated Trajectory (this test):\n');
fprintf('  Position RMSE:     %.4f m (%.1f%% in bounds)\n', ...
        metrics.tracking.rmse_position, metrics.tracking.time_in_bounds);
fprintf('  Attitude RMSE:     %.2f deg\n', rad2deg(metrics.tracking.rmse_attitude));
fprintf('  Max roll/pitch:    %.2f / %.2f deg\n', ...
        rad2deg(metrics.tracking.max_roll), rad2deg(metrics.tracking.max_pitch));
fprintf('  Overall success:   %s\n', iif(metrics.success.overall, 'YES', 'NO'));
fprintf('  Summary score:     %.3f\n', metrics.summary_weighted);

fprintf('\nOriginal quick_test_sim (manual trajectory):\n');
fprintf('  Position RMSE:     0.1586 m\n');
fprintf('  Attitude RMSE:     2.06 deg\n');
fprintf('  Max roll/pitch:    5.88 / 5.99 deg\n');
fprintf('  Overall success:   YES\n');
fprintf('  Summary score:     0.386\n');

fprintf('\n========================================\n');

function out = iif(condition, true_val, false_val)
    if condition
        out = true_val;
    else
        out = false_val;
    end
end