setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

% Load and visualize
wpt = load_waypoints('./trajectories/figure_eight.wpt');

% Generate with both methods
traj_interp = generate_trajectory_interp(wpt, params, 0.01);
traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);

% Plot comparison
figure('Position', [100 100 1400 600]);

subplot(1, 2, 1);
plot3(traj_interp.position(:,1), traj_interp.position(:,2), traj_interp.position(:,3), ...
      'b-', 'LineWidth', 2);
hold on;
plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
      'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Mixed Interpolation');
view(45, 30);

subplot(1, 2, 2);
plot3(traj_minsnap.position(:,1), traj_minsnap.position(:,2), traj_minsnap.position(:,3), ...
      'b-', 'LineWidth', 2);
hold on;
plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
      'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Minimum Snap');
view(45, 30);

sgtitle('Figure-Eight Trajectory Comparison', 'FontSize', 16);