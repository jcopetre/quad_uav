function params = quadrotor_linear_6dof(Q, R, verbose)
% QUADROTOR_LINEAR_6DOF - 6DOF linearized quadrotor model and LQR controller design
%
% Defines physical parameters, builds linearized state-space model around hover,
% and computes optimal LQR feedback gains.
%
% SYNTAX:
%   params = quadrotor_linear_6dof()
%   params = quadrotor_linear_6dof(Q, R)
%   params = quadrotor_linear_6dof(Q, R, verbose)
%
% INPUTS:
%   Q       - (optional) State weighting matrix (12x12), default provided
%   R       - (optional) Control weighting matrix (4x4), default provided
%   verbose - (optional) Display summary output (true/false), default true
%
% OUTPUTS:
%   params - Structure containing:
%            .m, .g, .L, .Ixx, .Iyy, .Izz  - Physical parameters
%            .A, .B, .C, .D                 - State-space matrices
%            .Q, .R                         - LQR weighting matrices
%            .K                             - LQR feedback gain (4x12)
%            .S                             - Solution to Riccati equation
%            .poles                         - Closed-loop eigenvalues
%            .u_hover                       - Nominal hover control (4x1)
%
% LINEARIZATION:
%   Model linearized around hover equilibrium:
%     - All states = 0
%     - Thrust = m*g (hover)
%     - Small angle approximation (sin(θ) ≈ θ, cos(θ) ≈ 1)
%
% STATE VECTOR:
%   x = [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]'
%     where: position (m), attitude (rad), velocity (m/s), angular rate (rad/s)
%
% CONTROL VECTOR:
%   u = [F, tau_phi, tau_theta, tau_psi]'
%     where: total thrust (N), torques (N·m)
%
% EXAMPLES:
%   % Default LQR weights with output
%   params = quadrotor_linear_6dof();
%
%   % Custom aggressive tracking, quiet mode
%   Q = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
%   R = diag([0.01 0.5 0.5 0.5]);
%   params = quadrotor_linear_6dof(Q, R, false);
%
% See also: lqr, ss

% Author: Trey Copeland
% Date: 2025-01-08

%% Input validation and default values
if nargin < 1 || isempty(Q)
    % Default state weights: prioritize position and altitude
    Q = diag([100 100 100 10 10 1 10 10 10 1 1 0.1]);
end

if nargin < 2 || isempty(R)
    % Default control weights: moderate thrust penalty, higher torque penalty
    R = diag([0.1 1 1 1]);
end

if nargin < 3
    verbose = true;  % Display output by default
end

% Validate dimensions
assert(isequal(size(Q), [12, 12]), 'Q must be 12x12 matrix');
assert(isequal(size(R), [4, 4]), 'R must be 4x4 matrix');

%% Physical Parameters
% Standard small quadrotor parameters (500g class)
params.m = 0.5;           % Mass (kg)
params.g = 9.81;          % Gravitational acceleration (m/s^2)
params.L = 0.25;          % Arm length from center to rotor (m)
params.Ixx = 0.0075;      % Moment of inertia about x-axis (kg·m^2)
params.Iyy = 0.0075;      % Moment of inertia about y-axis (kg·m^2)
params.Izz = 0.013;       % Moment of inertia about z-axis (kg·m^2)

%% Linearized State-Space Model
% Linearization around hover equilibrium
% 
% State vector: x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]'
% Control: u = [F, τ_φ, τ_θ, τ_ψ]'
%
% Linearized dynamics: ẋ = Ax + Bu

% Initialize state matrix (12x12)
A = zeros(12, 12);

% Position derivatives = velocities
A(1, 7) = 1;   % ẋ = x_dot
A(2, 8) = 1;   % ẏ = y_dot
A(3, 9) = 1;   % ż = z_dot

% Attitude derivatives = angular velocities (small angle assumption)
A(4, 10) = 1;  % φ̇ = p
A(5, 11) = 1;  % θ̇ = q
A(6, 12) = 1;  % ψ̇ = r

% Linear acceleration coupling (from tilted thrust)
% At hover with small angles: ẍ ≈ -g*θ, ÿ ≈ g*φ
A(7, 5) = -params.g;   % ẍ coupled to pitch (θ)
A(8, 4) = params.g;    % ÿ coupled to roll (φ)

% Note: z acceleration, angular accelerations handled by control input B matrix

% Initialize input matrix (12x4)
B = zeros(12, 4);

% Thrust affects vertical acceleration
B(9, 1) = 1 / params.m;   % z̈ = F/m

% Torques affect angular accelerations
B(10, 2) = 1 / params.Ixx;   % ṗ = τ_φ/Ixx
B(11, 3) = 1 / params.Iyy;   % q̇ = τ_θ/Iyy
B(12, 4) = 1 / params.Izz;   % ṙ = τ_ψ/Izz

% Output matrix: assume all states are measured
C = eye(12);

% Feedthrough matrix: no direct input to output
D = zeros(12, 4);

% Store matrices
params.A = A;
params.B = B;
params.C = C;
params.D = D;

%% LQR Controller Design
% Store weighting matrices
params.Q = Q;
params.R = R;

% Create state-space system
sys = ss(A, B, C, D);

% Compute optimal LQR feedback gain
% Minimizes: J = ∫(x'Qx + u'Ru)dt
% Control law: u = u_hover - K*(x - x_ref)
try
    [K, S, poles] = lqr(sys, Q, R);
    params.K = K;
    params.S = S;
    params.poles = poles;
catch ME
    error('LQR computation failed: %s', ME.message);
end

% Nominal hover control input
params.u_hover = [params.m * params.g; 0; 0; 0];

%% Display summary
if verbose
    fprintf('Quadrotor Linear 6DOF Model\n');
    fprintf('============================\n');
    fprintf('Physical Properties:\n');
    fprintf('  Mass:        %.3f kg\n', params.m);
    fprintf('  Arm length:  %.3f m\n', params.L);
    fprintf('  Inertia:     Ixx=%.4f, Iyy=%.4f, Izz=%.4f kg·m²\n', ...
            params.Ixx, params.Iyy, params.Izz);
    fprintf('\nLQR Controller:\n');
    fprintf('  Feedback gain K: [4x12]\n');
    fprintf('  Closed-loop poles (real part): %.2f to %.2f\n', ...
            min(real(poles)), max(real(poles)));
    fprintf('  All poles stable: %s\n', ...
            iif(all(real(poles) < 0), 'YES', 'NO'));
    fprintf('============================\n\n');
end

end

%% Helper function for conditional output
function out = iif(condition, true_val, false_val)
    % Inline if-then-else helper
    if condition
        out = true_val;
    else
        out = false_val;
    end
end
% QUADROTOR_LINEAR_6DOF - Single source of truth for quadrotor model
%
% Inputs:
%   Q - State weighting matrix (12x12), optional
%   R - Control weighting matrix (4x4), optional
%
% Outputs:
%   params - Structure containing all model parameters and LQR gains
%
% Usage:
%   params = quadrotor_linear_6dof();              % Use default Q, R
%   params = quadrotor_linear_6dof(Q_custom, R_custom);  % Custom weights

    %% Physical Parameters
    params.m = 0.5;           % Mass (kg)
    params.g = 9.81;          % Gravity (m/s^2)
    params.L = 0.25;          % Arm length (m)
    params.Ixx = 0.0075;      % Moment of inertia x-axis (kg*m^2)
    params.Iyy = 0.0075;      % Moment of inertia y-axis (kg*m^2)
    params.Izz = 0.013;       % Moment of inertia z-axis (kg*m^2)
    
    %% Linearized State-Space Model
    % State: x = [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]'
    % Input: u = [F, tau_phi, tau_theta, tau_psi]'
    
    A = zeros(12, 12);
    % Position derivatives = velocities
    A(1,7) = 1;   A(2,8) = 1;   A(3,9) = 1;
    % Attitude derivatives = angular velocities
    A(4,10) = 1;  A(5,11) = 1;  A(6,12) = 1;
    % Acceleration due to tilted thrust (linearized)
    A(7,5) = -params.g;   % x_ddot from theta
    A(8,4) = params.g;    % y_ddot from phi
    
    B = zeros(12, 4);
    B(9,1) = 1/params.m;                    % Thrust affects z
    B(10,2) = 1/params.Ixx;                 % Roll torque
    B(11,3) = 1/params.Iyy;                 % Pitch torque
    B(12,4) = 1/params.Izz;                 % Yaw torque
    
    C = eye(12);
    D = zeros(12, 4);
    
    params.A = A;
    params.B = B;
    params.C = C;
    params.D = D;
    
    %% LQR Controller Design
    % Default weights if not provided
    if nargin < 1 || isempty(Q)
        Q = diag([100 100 100 10 10 1 10 10 10 1 1 0.1]);
    end
    if nargin < 2 || isempty(R)
        R = diag([0.1 1 1 1]);
    end
    
    params.Q = Q;
    params.R = R;
    
    % Compute LQR gain
    sys = ss(A, B, C, D);
    [K, S, poles] = lqr(sys, Q, R);
    
    params.K = K;
    params.S = S;
    params.poles = poles;
    
    % Nominal hover control
    params.u_hover = [params.m * params.g; 0; 0; 0];
    
    % Display info
    fprintf('Quadrotor Linear 6DOF Model Initialized\n');
    fprintf('  Mass: %.2f kg, Arm length: %.2f m\n', params.m, params.L);
    fprintf('  LQR closed-loop poles (real): %.2f to %.2f\n', ...
            min(real(poles)), max(real(poles)));
    
end