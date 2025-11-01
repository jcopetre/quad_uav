% BASIC_SIMULATION - Simple quadrotor simulation examples
%
% Demonstrates basic usage of simulate_quadrotor_pure with various
% configurations: default settings, custom tuning, and batch mode.

clear; clc; close all;
init_project();

%% Example 1: Basic simulation with defaults
fprintf('Example 1: Basic simulation\n');
fprintf('----------------------------\n');
simulate_quadrotor_pure('basic_maneuver.wpt');

%% Example 2: Custom LQR tuning
fprintf('\nExample 2: Custom aggressive tuning\n');
fprintf('------------------------------------\n');
Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R_aggressive = diag([0.01 0.5 0.5 0.5]);
simulate_quadrotor_pure('basic_maneuver.wpt', Q_aggressive, R_aggressive);

%% Example 3: Batch mode (no plots)
fprintf('\nExample 3: Batch mode\n');
fprintf('----------------------\n');
opts.verbose = false;
opts.plot = false;
results = simulate_quadrotor_pure('basic_maneuver.wpt', [], [], [], opts);
fprintf('Simulation complete. Position RMSE: %.4f m\n', ...
        results.metrics.tracking.rmse_position);