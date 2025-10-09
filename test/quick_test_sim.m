% quick_test_simulation.m
% Quick test to see the quadrotor control system work
%
% This is a minimal script to validate the complete control loop

clear; close all; clc;

fprintf('========================================\n');
fprintf('Quadrotor Quick Test Simulation\n');
fprintf('========================================\n\n');

%% Step 1: Load vehicle model and LQR controller
fprintf('Step 1: Loading quadrotor model...\n');

% More aggressive position tracking
Q = diag([200 200 200 10 10 1 20 20 20 1 1 0.1]);  % Higher position weights
R = diag([0.1 1 1 1]);  % Standard control effort

params = quadrotor_linear_6dof(Q, R);
fprintf('  Using aggressive position tracking gains\n');

%% Step 2: Define a simple test trajectory
fprintf('Step 2: Creating test trajectory...\n');

% Simple climb and hover trajectory (SLOWER transitions)
trajectory.time = [0; 3; 7; 11; 15]';  % More time between waypoints
trajectory.position = [
    0,  0,  0;      % Start at origin
    0,  0,  1;      % Climb to 1m (3 sec)
    2,  0,  1;      % Move forward (4 sec)
    2,  2,  1;      % Move right (4 sec)
    2,  2,  0.5     % Descend slightly (4 sec)
];
trajectory.velocity = [
    0,  0,  0;
    0,  0,  0.33;   % Slower climb
    0.5, 0,  0;     % Slower forward
    0,  0.5, 0;     % Slower right
    0,  0, -0.125   % Slower descent
];
trajectory.attitude = zeros(5, 3);  % Level flight
trajectory.omega = zeros(5, 3);     % No rotation

fprintf('  Trajectory: %d waypoints, %.1f seconds\n', ...
        length(trajectory.time), trajectory.time(end));

%% Step 3: Initial conditions
fprintf('Step 3: Setting initial conditions...\n');
x0 = zeros(12, 1);  % Start at origin, stationary
fprintf('  Starting at origin, level, stationary\n');

%% Step 4: Run simulation
fprintf('Step 4: Running ODE simulation...\n');
tspan = [0, trajectory.time(end)];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

tic;
[t, x] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, trajectory), ...
               tspan, x0, options);
sim_time = toc;

fprintf('  Simulation complete: %.3f seconds (%.0f time steps)\n', ...
        sim_time, length(t));

%% Step 5: Extract reference trajectory
fprintf('Step 5: Computing tracking error...\n');
x_ref = zeros(length(t), 12);
for i = 1:length(t)
    x_ref(i, :) = get_reference_state(t(i), trajectory);
end

% Compute tracking errors
pos_error = x(:, 1:3) - x_ref(:, 1:3);
rmse_pos = sqrt(mean(pos_error.^2, 1));

fprintf('  Position RMSE: X=%.4fm, Y=%.4fm, Z=%.4fm\n', ...
        rmse_pos(1), rmse_pos(2), rmse_pos(3));

%% Step 6: Create plots
fprintf('Step 6: Generating plots...\n');

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
plot3(trajectory.position(:,1), trajectory.position(:,2), trajectory.position(:,3), ...
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

sgtitle('Quadrotor LQR Control - Quick Test', 'FontSize', 16, 'FontWeight', 'bold');

%% Summary
fprintf('\n========================================\n');
fprintf('SIMULATION COMPLETE!\n');
fprintf('========================================\n');
fprintf('Total time:        %.1f seconds\n', t(end));
fprintf('Position RMSE:     %.4f m\n', norm(rmse_pos));
fprintf('Max altitude:      %.3f m\n', max(x(:,3)));
fprintf('Max roll angle:    %.2f deg\n', max(abs(rad2deg(x(:,4)))));
fprintf('Max pitch angle:   %.2f deg\n', max(abs(rad2deg(x(:,5)))));
fprintf('========================================\n\n');

fprintf('SUCCESS! The quadrotor is flying! 🚁\n\n');