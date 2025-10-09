function x_ref = get_reference_state(t, trajectory)
% GET_REFERENCE_STATE - Lookup reference state from trajectory at given time
%
% Interpolates trajectory data to find the desired state at time t.
% Handles boundary conditions (before start, after end).
%
% SYNTAX:
%   x_ref = get_reference_state(t, trajectory)
%
% INPUTS:
%   t          - Current time (scalar) [s]
%   trajectory - Trajectory structure containing:
%                .time     - Time vector (Nx1)
%                .position - Position [x, y, z] (Nx3) [m]
%                .velocity - Velocity [vx, vy, vz] (Nx3) [m/s]
%                .attitude - Euler angles [phi, theta, psi] (Nx3) [rad]
%                .omega    - Angular velocity [p, q, r] (Nx3) [rad/s]
%
% OUTPUTS:
%   x_ref - Reference state vector (12x1):
%           [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]'
%
% INTERPOLATION:
%   Uses linear interpolation (interp1) for smooth reference tracking.
%   Extrapolation holds the first/last value (boundary clamp).
%
% EXAMPLE:
%   trajectory.time = [0; 1; 2];
%   trajectory.position = [0 0 0; 1 0 1; 2 0 1];
%   trajectory.velocity = zeros(3, 3);
%   trajectory.attitude = zeros(3, 3);
%   trajectory.omega = zeros(3, 3);
%   x_ref = get_reference_state(1.5, trajectory);
%
% See also: interp1, generate_trajectory

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Validate inputs
assert(isscalar(t), 'Time must be a scalar');
assert(isstruct(trajectory), 'Trajectory must be a structure');

% Check required fields
required_fields = {'time', 'position', 'velocity', 'attitude', 'omega'};
for i = 1:length(required_fields)
    assert(isfield(trajectory, required_fields{i}), ...
           sprintf('Trajectory must contain field: %s', required_fields{i}));
end

%% Handle boundary conditions
t_start = trajectory.time(1);
t_end = trajectory.time(end);

% Clamp time to trajectory bounds
t_clamped = max(t_start, min(t_end, t));

%% Interpolate trajectory data
% Position [x, y, z]
pos = interp1(trajectory.time, trajectory.position, t_clamped, 'linear');

% Velocity [vx, vy, vz]
vel = interp1(trajectory.time, trajectory.velocity, t_clamped, 'linear');

% Attitude [phi, theta, psi]
att = interp1(trajectory.time, trajectory.attitude, t_clamped, 'linear');

% Angular velocity [p, q, r]
omega = interp1(trajectory.time, trajectory.omega, t_clamped, 'linear');

%% Assemble reference state vector
x_ref = [pos(:); att(:); vel(:); omega(:)];

% Ensure column vector
x_ref = x_ref(:);

%% Validate output
assert(length(x_ref) == 12, 'Reference state must be 12x1');
assert(all(isfinite(x_ref)), 'Reference state contains non-finite values');

end
