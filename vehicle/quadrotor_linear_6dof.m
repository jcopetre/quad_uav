function params = quadrotor_linear_6dof(Q, R)
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