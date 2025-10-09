function x_dot = quadrotor_dynamics_pure(x, u, params)
% QUADROTOR_DYNAMICS_PURE - Nonlinear 6DOF quadrotor dynamics
%
% Computes state derivatives for a rigid-body quadrotor using full nonlinear
% equations of motion. No simplifications or small-angle approximations.
%
% SYNTAX:
%   x_dot = quadrotor_dynamics_pure(x, u, params)
%
% INPUTS:
%   x      - State vector (12x1): [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]'
%            Position (m), Euler angles (rad), velocities (m/s), angular rates (rad/s)
%   u      - Control vector (4x1): [F, tau_phi, tau_theta, tau_psi]'
%            Thrust (N), torques (N·m)
%   params - Parameter structure from quadrotor_linear_6dof containing:
%            .m, .g, .Ixx, .Iyy, .Izz
%
% OUTPUTS:
%   x_dot - State derivative (12x1): time derivative of state vector
%
% STATE VECTOR DEFINITION:
%   x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]'
%   Indices: 1:3   = position (inertial frame)
%            4:6   = Euler angles (roll, pitch, yaw)
%            7:9   = linear velocity (inertial frame)
%            10:12 = angular velocity (body frame)
%
% DYNAMICS:
%   Position:    ṙ = v
%   Orientation: Euler rate = f(angular velocity, angles)
%   Linear:      m*v̇ = R*[0; 0; F] - [0; 0; m*g]
%   Angular:     I*ω̇ = τ - ω × (I*ω)
%
% COORDINATE FRAMES:
%   Inertial: NED (North-East-Down) - +z is DOWN
%   Body: Fixed to quadrotor, origin at center of mass, +z is DOWN (thrust opposes this)
%
% ROTATION CONVENTION:
%   ZYX Euler angles (yaw-pitch-roll)
%   R_b2i = Rz(ψ) * Ry(θ) * Rx(φ)
%
% EXAMPLE:
%   params = quadrotor_linear_6dof();
%   x0 = zeros(12, 1);  % Stationary at origin
%   u_hover = [params.m * params.g; 0; 0; 0];
%   x_dot = quadrotor_dynamics_pure(x0, u_hover, params);
%
% See also: quadrotor_linear_6dof, quadrotor_closed_loop_dynamics

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Extract states
% Position (inertial frame)
% x_pos = x(1);  % Not needed for dynamics
% y_pos = x(2);
% z_pos = x(3);

% Euler angles (roll, pitch, yaw)
phi   = x(4);
theta = x(5);
psi   = x(6);

% Linear velocities (inertial frame)
v = x(7:9);  % [vx; vy; vz]

% Angular velocities (body frame)
omega = x(10:12);  % [p; q; r]
p = omega(1);
q = omega(2);
r = omega(3);

%% Extract control inputs
F       = u(1);      % Total thrust (N)
tau_phi = u(2);      % Roll torque (N·m)
tau_theta = u(3);    % Pitch torque (N·m)
tau_psi = u(4);      % Yaw torque (N·m)

tau = [tau_phi; tau_theta; tau_psi];  % Torque vector

%% Extract parameters
m   = params.m;
g   = params.g;
Ixx = params.Ixx;
Iyy = params.Iyy;
Izz = params.Izz;

% Inertia matrix
I = diag([Ixx, Iyy, Izz]);

%% Rotation matrix: Body → Inertial (ZYX Euler)
% R_b2i transforms vectors from body frame to inertial frame
% Standard aerospace rotation sequence: R = Rz(ψ) * Ry(θ) * Rx(φ)

% Individual rotation matrices
Rx = [1,         0,          0;
      0,  cos(phi),  -sin(phi);
      0,  sin(phi),   cos(phi)];

Ry = [ cos(theta), 0, sin(theta);
              0,    1,         0;
      -sin(theta), 0, cos(theta)];

Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
           0,         0,    1];

% Combined rotation matrix
R_b2i = Rz * Ry * Rx;

%% Translational dynamics (Newton's 2nd law)
% Forces in inertial frame (NED convention: +z is DOWN):
%   - Thrust: R_b2i * [0; 0; -F]  (thrust opposes body +z, which points down)
%   - Gravity: [0; 0; m*g]        (downward in inertial frame, +z direction)

F_thrust_inertial = R_b2i * [0; 0; -F];  % Negative because thrust opposes +z (down)
F_gravity = [0; 0; m * g];  % Gravity acts downward (+z direction in NED)

% Linear acceleration: a = F_net / m
a = (F_thrust_inertial + F_gravity) / m;

%% Rotational dynamics (Euler's equation)
% Torque equation in body frame:
%   I*ω̇ = τ - ω × (I*ω)
%
% This accounts for:
%   - Applied torques (τ)
%   - Gyroscopic effects (ω × (I*ω))

omega_dot = I \ (tau - cross(omega, I * omega));

%% Euler angle kinematics
% Relationship between Euler angle rates and body angular velocities:
%
%   [φ̇]   [1  sin(φ)tan(θ)  cos(φ)tan(θ)] [p]
%   [θ̇] = [0     cos(φ)       -sin(φ)   ] [q]
%   [ψ̇]   [0  sin(φ)sec(θ)  cos(φ)sec(θ)] [r]
%
% This is the inverse of the angular velocity transformation matrix

% Precompute trig functions
c_phi = cos(phi);
s_phi = sin(phi);
c_theta = cos(theta);
s_theta = sin(theta);
t_theta = tan(theta);  % tan(θ)

% Check for gimbal lock (θ = ±90°)
if abs(c_theta) < 1e-6
    warning('Gimbal lock detected: theta = %.2f deg', rad2deg(theta));
    % Use small epsilon to prevent division by zero
    c_theta = sign(c_theta) * 1e-6;
end

% Euler angle rate transformation matrix
W = [1,  s_phi * t_theta,  c_phi * t_theta;
     0,  c_phi,           -s_phi;
     0,  s_phi / c_theta,  c_phi / c_theta];

euler_dot = W * omega;

%% Assemble state derivative
x_dot = zeros(12, 1);

% Position derivatives = velocities
x_dot(1:3) = v;

% Euler angle derivatives
x_dot(4:6) = euler_dot;

% Velocity derivatives = accelerations
x_dot(7:9) = a;

% Angular velocity derivatives
x_dot(10:12) = omega_dot;

end