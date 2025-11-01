function [t, x, u_log] = ode_simulate(x0, tspan, params, trajectory, options)
% SIMULATE_QUADROTOR - Run closed-loop simulation with control logging
%
% Simulates quadrotor trajectory tracking using LQR control and nonlinear
% 6DOF dynamics. Returns state history and control input history.
%
% SYNTAX:
%   [t, x, u_log] = ode_simulate(x0, tspan, params, trajectory)
%   [t, x, u_log] = ode_simulate(x0, tspan, params, trajectory, options)
%
% INPUTS:
%   x0         - Initial state vector (12x1)
%   tspan      - Time span [t_start, t_end] (1x2) [s]
%   params     - Parameter structure from quadrotor_linear_6dof
%   trajectory - Reference trajectory structure
%   options    - (optional) ODE solver options from odeset
%
% OUTPUTS:
%   t     - Time vector (Nx1) [s]
%   x     - State history (Nx12) [states at each time]
%   u_log - Control input history (Nx4) [F, tau_phi, tau_theta, tau_psi]
%
% NOTES:
%   Control inputs are recomputed after simulation completes. This is exact
%   for deterministic controllers (like LQR) and avoids complex logging during
%   ODE integration.
%
% EXAMPLE:
%   params = quadrotor_linear_6dof();
%   wpt = load_waypoints('./trajectories/basic_maneuver.wpt');
%   traj = generate_trajectory(wpt, params);
%   x0 = zeros(12, 1);
%   tspan = [0, traj.time(end)];
%   
%   [t, x, u_log] = ode_simulate(x0, tspan, params, traj);
%   
%   % Analyze results
%   metrics = compute_metrics(t, x, traj, params, u_log);
%
% See also: ode45, quadrotor_closed_loop_dynamics, compute_performance_metrics

% Author: Trey Copeland
% Date: 2025-10-09

    %% Input validation
    assert(length(x0) == 12, 'Initial state must be 12x1');
    assert(length(tspan) == 2, 'Time span must be [t_start, t_end]');
    assert(isstruct(params), 'params must be a structure');
    assert(isstruct(trajectory), 'trajectory must be a structure');
    
    %% Set default ODE options if not provided
    if nargin < 5 || isempty(options)
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    end
    
    %% Run ODE simulation
    [t, x] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, trajectory), ...
                   tspan, x0, options);
    
    %% Recompute control inputs for logging
    % Since LQR is deterministic (u = f(x, x_ref)), recomputation is exact
    n = length(t);
    u_log = zeros(n, 4);
    
    for i = 1:n
        % Get reference state at this time
        x_ref = get_reference_state(t(i), trajectory);
        
        % Compute control input (deterministic)
        u_log(i, :) = compute_lqr_control(x(i, :)', x_ref, params);
    end

end