clear, clc, close all

% Basic test simply reproduces quick_test_sim.m
% Basic test
simulate_quadrotor_pure('basic_maneuver.wpt');

% Basic test with manual Q and R settings
% Q = diag([200 200 200 10 10 1 20 20 20 1 1 0.1]);
% R = diag([0.1 1 1 1]);
% simulate_quadrotor_pure('basic_maneuver.wpt', Q, R);

% Custom tuning
% Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
% simulate_quadrotor_pure('basic_maneuver.wpt', Q_aggressive);

% Batch mode (no output)
% opts.verbose = false;
% opts.plot = false;
% results = simulate_quadrotor_pure('basic_maneuver.wpt', [], [], [], opts);

