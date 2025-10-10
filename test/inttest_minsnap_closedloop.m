% Test minimum snap with LQR controller
setup_test_environment();
params = quadrotor_linear_6dof([], [], false);

% Load a complex trajectory
wpt = load_waypoints('./trajectories/figure_eight.wpt');

% Generate with BOTH methods
traj_pchip = generate_trajectory(wpt, params, 0.01);
traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);

% Simulate both
x0 = zeros(12, 1);
tspan = [0, wpt.time(end)];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

[t_pchip, x_pchip] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, traj_pchip), ...
                            tspan, x0, options);

[t_minsnap, x_minsnap] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, traj_minsnap), ...
                                tspan, x0, options);

% Compare tracking performance
% Plot side-by-side comparison
% Compute RMSE for position, attitude