% Quadrotor 6DOF Simulation with LQR Control
% Simplified - No duplicate model definitions

clear; clc; close all;

% Add vehicle directory to path
addpath('./vehicle');

fprintf('===================================\n');
fprintf('Quadrotor 6DOF LQR Control System\n');
fprintf('===================================\n\n');

%% Step 1: Load Vehicle Model (Single Source of Truth)
fprintf('Step 1/5: Loading quadrotor model...\n');

% Option 1: Use default Q and R weights
params = quadrotor_linear_6dof();

% Option 2: Custom weights (uncomment to use)
% Q_custom = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);
% R_custom = diag([0.2 2 2 2]);
% params = quadrotor_linear_6dof(Q_custom, R_custom);

% Save for Simulink to load
save('./vehicle/quadrotor_params.mat', 'params');

%% Step 2: Generate Trajectory
fprintf('\nStep 2/5: Generating optimal trajectory...\n');

% Waypoints [x, y, z, yaw, time]
waypoints = [
    0,   0,   0,   0,   0;
    2,   0,   1,   0,   3;
    2,   2,   1,   pi/2, 6;
    0,   2,   1.5, pi,   9;
    0,   0,   1,   0,   12;
    0,   0,   0,   0,   15;
];

dt = 0.01;
t_vec = 0:dt:waypoints(end,5);
n_points = length(t_vec);

% Preallocate
pos = zeros(n_points, 3);
vel = zeros(n_points, 3);
acc = zeros(n_points, 3);
yaw = zeros(n_points, 1);
yaw_rate = zeros(n_points, 1);

% Generate smooth trajectory using 5th order polynomial
for i = 1:size(waypoints,1)-1
    t_start = waypoints(i,5);
    t_end = waypoints(i+1,5);
    mask = (t_vec >= t_start) & (t_vec <= t_end);
    t_seg = t_vec(mask);
    
    tau = (t_seg - t_start) / (t_end - t_start);
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    s_dot = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / (t_end - t_start);
    s_ddot = (60*tau - 180*tau.^2 + 120*tau.^3) / (t_end - t_start)^2;
    
    p0 = waypoints(i,1:3)';
    p1 = waypoints(i+1,1:3)';
    
    for axis = 1:3
        pos(mask, axis) = p0(axis) + (p1(axis) - p0(axis)) * s;
        vel(mask, axis) = (p1(axis) - p0(axis)) * s_dot;
        acc(mask, axis) = (p1(axis) - p0(axis)) * s_ddot;
    end
    
    yaw0 = waypoints(i,4);
    yaw1 = waypoints(i+1,4);
    dyaw = yaw1 - yaw0;
    if dyaw > pi, dyaw = dyaw - 2*pi; end
    if dyaw < -pi, dyaw = dyaw + 2*pi; end
    
    yaw(mask) = yaw0 + dyaw * tau;
    yaw_rate(mask) = dyaw / (t_end - t_start);
end

trajectory.time = t_vec';
trajectory.pos = pos;
trajectory.vel = vel;
trajectory.acc = acc;
trajectory.yaw = yaw;
trajectory.yaw_rate = yaw_rate;
trajectory.dt = dt;
trajectory.phi_d = -acc(:,2) / params.g;
trajectory.theta_d = acc(:,1) / params.g;

save('./vehicle/trajectory.mat', 'trajectory');
fprintf('  Generated %d points over %.1f seconds\n', n_points, t_vec(end));

%% Step 3: Check Simulink Model
fprintf('\nStep 3/5: Checking Simulink model...\n');

model_name = 'quadrotor_6dof_lqr';  % Just the model name (no path)
model_path = './vehicle/quadrotor_6dof_lqr';  % Full path for loading

if ~bdIsLoaded(model_name)
    if exist([model_path '.slx'], 'file')
        fprintf('  Loading model from %s.slx...\n', model_path);
        load_system(model_path);
    else
        error('Model not found: %s.slx\nRun generate_vehicle_slx() first.', model_path);
    end
else
    fprintf('  Model already loaded\n');
end

%% Step 4: Run Simulation
fprintf('\nStep 4/5: Running Simulink simulation...\n');

set_param(model_name, 'StopTime', num2str(trajectory.time(end)));
set_param(model_name, 'FixedStep', num2str(dt));
set_param(model_name, 'Solver', 'FixedStepAuto');

tic;
sim_out = sim(model_name);
sim_time = toc;

fprintf('  Simulation completed in %.2f seconds\n', sim_time);

% Extract results
time = sim_out.tout;
states = sim_out.states;
controls = sim_out.controls;
trajectory_ref = sim_out.trajectory_ref;

%% Step 5: Analyze Results
fprintf('\nStep 5/5: Analyzing results...\n');

pos_actual = states(:, 1:3);
att_actual = states(:, 4:6);
pos_ref = trajectory_ref(:, 1:3);
att_ref = [trajectory_ref(:, 4:5), trajectory.yaw];

% Performance metrics
pos_error = pos_actual - pos_ref;
att_error = (att_actual - att_ref) * 180/pi;

rmse_pos = sqrt(mean(pos_error.^2));
rmse_att = sqrt(mean(att_error.^2));
max_pos_error = max(abs(pos_error));
max_att_error = max(abs(att_error));

fprintf('\n=== PERFORMANCE METRICS ===\n');
fprintf('Position RMSE:  X=%.4fm  Y=%.4fm  Z=%.4fm\n', rmse_pos);
fprintf('Position Max:   X=%.4fm  Y=%.4fm  Z=%.4fm\n', max_pos_error);
fprintf('Attitude RMSE:  φ=%.2f°  θ=%.2f°  ψ=%.2f°\n', rmse_att);
fprintf('Attitude Max:   φ=%.2f°  θ=%.2f°  ψ=%.2f°\n', max_att_error);

% Plot results
figure('Position', [50 50 1400 900]);

subplot(3,3,1);
plot3(pos_ref(:,1), pos_ref(:,2), pos_ref(:,3), 'b--', 'LineWidth', 2); 
hold on;
plot3(pos_actual(:,1), pos_actual(:,2), pos_actual(:,3), 'r-', 'LineWidth', 1.5);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory'); legend('Reference', 'Actual');
axis equal; view(45, 30);

for i = 1:3
    subplot(3,3,i+1);
    plot(time, pos_ref(:,i), 'b--', 'LineWidth', 2); hold on;
    plot(time, pos_actual(:,i), 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel(['Position ' char('X'+i-1) ' (m)']);
    title([char('X'+i-1) ' Tracking']); legend('Ref', 'Actual');
end

labels = {'\phi', '\theta', '\psi'};
for i = 1:3
    subplot(3,3,i+4);
    plot(time, att_ref(:,i)*180/pi, 'b--', 'LineWidth', 2); hold on;
    plot(time, att_actual(:,i)*180/pi, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel([labels{i} ' (°)']);
    title([labels{i} ' Tracking']); legend('Ref', 'Actual');
end

subplot(3,3,8);
plot(time, controls(:,1), 'LineWidth', 1.5); hold on;
yline(params.m * params.g, 'k--', 'Hover');
grid on; xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thrust'); legend('Actual', 'Hover');

subplot(3,3,9);
plot(time, controls(:,2:4), 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Torque (N·m)');
title('Control Torques'); legend('\tau_\phi', '\tau_\theta', '\tau_\psi');

sgtitle('Quadrotor 6DOF - LQR Control');

fprintf('\n=== SIMULATION COMPLETE ===\n\n');