function u = compute_lqr_control(x, x_ref, params)
% COMPUTE_LQR_CONTROL - Apply LQR control law with saturation
%
% Computes control input using linear quadratic regulator feedback around
% a reference trajectory with actuator saturation limits.
%
% SYNTAX:
%   u = compute_lqr_control(x, x_ref, params)
%
% INPUTS:
%   x      - Current state vector (12x1)
%   x_ref  - Reference state vector (12x1)
%   params - Parameter structure containing:
%            .K       - LQR feedback gain matrix (4x12)
%            .u_hover - Nominal hover control (4x1)
%            .u_max   - Maximum control limits (4x1) [optional]
%            .u_min   - Minimum control limits (4x1) [optional]
%
% OUTPUTS:
%   u - Control vector (4x1): [F, tau_phi, tau_theta, tau_psi]'
%
% CONTROL LAW:
%   e = x - x_ref                    (tracking error)
%   u = u_hover - K*e                (LQR feedback)
%   u = saturate(u, u_min, u_max)    (actuator limits)
%
% DEFAULT SATURATION LIMITS:
%   Thrust:  [0, 2*m*g] N
%   Torques: [-0.1, +0.1] N·m
%
% EXAMPLE:
%   params = quadrotor_linear_6dof();
%   x = [0; 0; 1; 0.1; 0; 0; 0; 0; 0; 0; 0; 0];  % Slight roll at 1m altitude
%   x_ref = zeros(12, 1);  % Want to be at origin, level
%   u = compute_lqr_control(x, x_ref, params);
%
% See also: quadrotor_linear_6dof, quadrotor_closed_loop_dynamics

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Validate inputs
assert(length(x) == 12, 'State vector must be 12x1');
assert(length(x_ref) == 12, 'Reference state must be 12x1');
assert(isfield(params, 'K'), 'params must contain K matrix');
assert(isfield(params, 'u_hover'), 'params must contain u_hover');

%% Compute tracking error
e = x - x_ref;

%% Apply LQR control law
% u = u_nominal - K*e
% The negative sign is because LQR minimizes cost, so feedback opposes error
u = params.u_hover - params.K * e;

%% Apply saturation limits
% Default limits if not provided
if ~isfield(params, 'u_max') || isempty(params.u_max)
    % Thrust: 0 to 2x hover thrust (can't pull, max 2g acceleration)
    % Torques: symmetric limits based on typical quadrotor
    params.u_max = [2 * params.m * params.g;  % Max thrust
                    0.1;                       % Max roll torque
                    0.1;                       % Max pitch torque
                    0.1];                      % Max yaw torque
end

if ~isfield(params, 'u_min') || isempty(params.u_min)
    % Thrust: non-negative (can't pull)
    % Torques: negative of max
    params.u_min = [0;      % Min thrust
                    -0.1;   % Min roll torque
                    -0.1;   % Min pitch torque
                    -0.1];  % Min yaw torque
end

% Saturate each control input
u = min(max(u, params.u_min), params.u_max);

end
