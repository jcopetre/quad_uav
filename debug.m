% Load waypoints
wpt = load_waypoints('./trajectories/basic_maneuver.wpt');
params = quadrotor_linear_6dof([], [], false);

% Generate trajectory
traj = generate_trajectory(wpt, params, 0.01);

% Check for problems
fprintf('Trajectory Diagnostics:\n');
fprintf('  Time points: %d\n', length(traj.time));
fprintf('  Max acceleration: %.2f m/s²\n', max(sqrt(sum(traj.acceleration.^2, 2))));
fprintf('  Max roll: %.2f deg\n', rad2deg(max(abs(traj.attitude(:,1)))));
fprintf('  Max pitch: %.2f deg\n', rad2deg(max(abs(traj.attitude(:,2)))));
fprintf('  Max yaw: %.2f deg\n', rad2deg(max(abs(traj.attitude(:,3)))));

% Check for NaN or Inf
fprintf('  Contains NaN: %s\n', mat2str(any(isnan(traj.attitude(:)))));
fprintf('  Contains Inf: %s\n', mat2str(any(isinf(traj.attitude(:)))));

% Plot attitude to see what's wrong
figure;
subplot(3,1,1); plot(traj.time, rad2deg(traj.attitude(:,1))); 
title('Roll'); ylabel('deg'); grid on;
subplot(3,1,2); plot(traj.time, rad2deg(traj.attitude(:,2))); 
title('Pitch'); ylabel('deg'); grid on;
subplot(3,1,3); plot(traj.time, rad2deg(traj.attitude(:,3))); 
title('Yaw'); ylabel('deg'); grid on; xlabel('Time (s)');