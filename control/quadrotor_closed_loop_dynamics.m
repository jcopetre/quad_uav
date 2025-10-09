function x_dot = quadrotor_closed_loop_dynamics(t, x, params, trajectory)
% QUADROTOR_CLOSED_LOOP_DYNAMICS - Closed-loop system dynamics for ODE solver
%
% Wrapper function that combines reference tracking, LQR control, and nonlinear
% dynamics. Called repeatedly by ode45 during simulation.
%
% SYNTAX:
%   x_dot = quadrotor_closed_loop_dynamics(t, x, params, trajectory)
%
% INPUTS:
%   t          - Current time (scalar) [s]
%   x          - Current state vector (12x1)
%   params     - Parameter structure from quadrotor_linear_6dof containing:
%                .K, .u_hover, physical parameters, etc.
%   trajectory - Trajectory structure with reference data
%
% OUTPUTS:
%   x_dot - State derivative (12x1) for ODE solver
%
% CONTROL LOOP:
%   1. Lookup reference state x_ref(t) from trajectory
%   2. Compute control input u using LQR: u = u0 - K*(x - x_ref)
%   3. Evaluate nonlinear dynamics: x_dot = f(x, u)
%   4. Return x_dot to ODE solver
%
% USAGE WITH ODE45:
%   [t, x] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, traj), ...
%                  tspan, x0);
%
% EXAMPLE:
%   params = quadrotor_linear_6dof();
%   
%   % Simple hover trajectory
%   trajectory.time = [0; 10];
%   trajectory.position = [0 0 0; 0 0 1];
%   trajectory.velocity = zeros(2, 3);
%   trajectory.attitude = zeros(2, 3);
%   trajectory.omega = zeros(2, 3);
%   
%   % Simulate
%   x0 = zeros(12, 1);  % Start at origin
%   tspan = [0 10];
%   [t, x] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, trajectory), ...
%                  tspan, x0);
%
% See also: ode45, get_reference_state, compute_lqr_control, quadrotor_dynamics_pure

% Author: Trey Copeland
% Date: 2025-01-08

%% Step 1: Get reference state at current time
x_ref = get_reference_state(t, trajectory);

%% Step 2: Compute LQR control input
u = compute_lqr_control(x, x_ref, params);

%% Step 3: Evaluate nonlinear dynamics
x_dot = quadrotor_dynamics_pure(x, u, params);

%% Optional: Store control for post-processing
% (If needed later, can use global variable or nested function)
% This would require restructuring to return u alongside x_dot

end
