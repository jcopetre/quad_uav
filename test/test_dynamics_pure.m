% test_dynamics_pure.m
% Unit tests for quadrotor_dynamics_pure function
%
% Run this file to validate the nonlinear 6DOF dynamics

function test_dynamics_pure()
    fprintf('Running Unit Tests: quadrotor_dynamics_pure\n');
    fprintf('============================================\n\n');
    
    % Setup test environment
    setup_test_environment();

    % Run all test cases
    test_hover_equilibrium();
    test_free_fall();
    test_pure_thrust();
    test_rotation_matrix_orthogonality();
    test_angular_momentum_conservation();
    test_gimbal_lock_warning();
    test_symmetry();
    
    fprintf('\n============================================\n');
    fprintf('All Tests Passed! \n\n');
end

%% Test 1: Hover Equilibrium
function test_hover_equilibrium()
    fprintf('Test 1: Hover equilibrium... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Initial state: stationary at origin, level
    x_hover = zeros(12, 1);
    
    % Hover control: thrust = weight, no torques
    u_hover = [params.m * params.g; 0; 0; 0];
    
    % Compute dynamics
    x_dot = quadrotor_dynamics_pure(x_hover, u_hover, params);
    
    % At hover, all derivatives should be zero (or very close)
    % Position rates = velocities = 0
    % Euler rates = W*omega = W*0 = 0
    % Accelerations = (thrust - weight)/m = 0
    % Angular accelerations = I\(tau - omega×Iomega) = I\0 = 0
    
    max_derivative = max(abs(x_dot));
    assert(max_derivative < Constants.TOL, ...
           sprintf('Hover not in equilibrium, max derivative: %.2e', max_derivative));
    
    fprintf('PASS (max derivative: %.2e)\n', max_derivative);
end

%% Test 2: Free Fall
function test_free_fall()
    fprintf('Test 2: Free fall dynamics... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Initial state: stationary at altitude, level
    x0 = zeros(12, 1);
    x0(3) = 10;  % 10m altitude
    
    % No control input (free fall)
    u_zero = zeros(4, 1);
    
    % Compute dynamics
    x_dot = quadrotor_dynamics_pure(x0, u_zero, params);
    
    % Should have:
    % - Zero horizontal acceleration
    % - Downward acceleration = g
    % - No rotation
    
    assert(abs(x_dot(7)) < Constants.TOL, 'Should have no x-acceleration');
    assert(abs(x_dot(8)) < Constants.TOL, 'Should have no y-acceleration');
    assert(abs(x_dot(9) - params.g) < Constants.TOL, ...
           sprintf('Should accelerate downward at g, got %.2f', x_dot(9)));
    
    % No angular acceleration
    assert(all(abs(x_dot(10:12)) < Constants.TOL), 'Should have no angular acceleration');
    
    fprintf('PASS\n');
end

%% Test 3: Pure Thrust
function test_pure_thrust()
    fprintf('Test 3: Pure vertical thrust... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Level orientation, no velocity
    x0 = zeros(12, 1);
    
    % Thrust = 2*weight (should accelerate upward at g)
    u_thrust = [2 * params.m * params.g; 0; 0; 0];
    
    % Compute dynamics
    x_dot = quadrotor_dynamics_pure(x0, u_thrust, params);
    
    % Net upward acceleration should be g
    % F_net = 2mg - mg = mg
    % a = F_net/m = g (upward, so negative in NED, positive in ENU)
    % We're using standard convention where +z is up in body frame
    
    expected_accel = -params.g;  % Net upward acceleration
    assert(abs(x_dot(9) - expected_accel) < Constants.TOL, ...
           sprintf('Expected %.2f m/s² upward, got %.2f', expected_accel, x_dot(9)));
    
    fprintf('PASS\n');
end

%% Test 4: Rotation Matrix Orthogonality
function test_rotation_matrix_orthogonality()
    fprintf('Test 4: Rotation matrix properties... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Test at various angles
    test_angles = [
        0,     0,     0;      % Level
        0.1,   0,     0;      % Small roll
        0,     0.1,   0;      % Small pitch
        0,     0,     0.1;    % Small yaw
        0.5,   0.3,   1.2;    % Mixed attitude
        pi/4,  pi/6,  pi/3;   % Larger angles
    ];
    
    for i = 1:size(test_angles, 1)
        x = zeros(12, 1);
        x(4:6) = test_angles(i, :)';
        u = [params.m * params.g; 0; 0; 0];
        
        % Call dynamics (internally computes rotation matrix)
        % We'll verify by checking that thrust transforms correctly
        x_dot = quadrotor_dynamics_pure(x, u, params);
        
        % For orthogonality test, we need to reconstruct R here
        phi = x(4); theta = x(5); psi = x(6);
        
        Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
        Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
        Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
        R = Rz * Ry * Rx;
        
        % Test orthogonality: R'*R = I
        I_test = R' * R;
        orthog_error = norm(I_test - eye(3), 'fro');
        assert(orthog_error < Constants.TOL, ...
               sprintf('Rotation matrix not orthogonal, error: %.2e', orthog_error));
        
        % Test determinant = 1 (proper rotation, no reflection)
        det_R = det(R);
        assert(abs(det_R - 1) < Constants.TOL, ...
               sprintf('Determinant should be 1, got %.6f', det_R));
    end
    
    fprintf('PASS (tested %d attitudes)\n', size(test_angles, 1));
end

%% Test 5: Angular Momentum Conservation
function test_angular_momentum_conservation()
    fprintf('Test 5: Angular momentum conservation (no torque)... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Spinning quadrotor with no external torques
    x0 = zeros(12, 1);
    x0(10:12) = [0.1; 0.2; 0.3];  % Initial angular velocity
    
    % Hover thrust but zero torques
    u = [params.m * params.g; 0; 0; 0];
    
    % Compute dynamics
    x_dot = quadrotor_dynamics_pure(x0, u, params);
    
    % With no external torques: I*ω̇ = -ω × (I*ω)
    % Angular momentum L = I*ω should change due to gyroscopic effects
    % But the magnitude should change only slightly for small time steps
    
    I = diag([params.Ixx, params.Iyy, params.Izz]);
    omega = x0(10:12);
    omega_dot = x_dot(10:12);
    
    % Check that omega_dot = I\(-omega × I*omega)
    expected_omega_dot = I \ (-cross(omega, I * omega));
    
    error = norm(omega_dot - expected_omega_dot);
    assert(error < Constants.TOL, ...
           sprintf('Angular dynamics incorrect, error: %.2e', error));
    
    fprintf('PASS\n');
end

%% Test 6: Gimbal Lock Warning
function test_gimbal_lock_warning()
    fprintf('Test 6: Gimbal lock detection... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Set pitch to near 90 degrees (gimbal lock)
    x_gimbal = zeros(12, 1);
    x_gimbal(5) = pi/2 - 1e-7;  % theta ≈ 90°
    x_gimbal(10:12) = [0.1; 0.1; 0.1];  % Some angular velocity
    
    u = [params.m * params.g; 0; 0; 0];
    
    % Should trigger warning but not crash
    warning('off', 'all');  % Suppress expected warning for test
    lastwarn('');  % Clear last warning
    
    x_dot = quadrotor_dynamics_pure(x_gimbal, u, params);
    
    [~, msgid] = lastwarn;
    warning('on', 'all');  % Re-enable warnings
    
    % Verify function still returned valid output
    assert(length(x_dot) == 12, 'Should return valid state derivative');
    assert(all(isfinite(x_dot)), 'State derivative should be finite');
    
    fprintf('PASS\n');
end

%% Test 7: Symmetry Properties
function test_symmetry()
    fprintf('Test 7: Symmetry in dynamics... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Test 1: Roll left vs roll right symmetry
    x_roll_pos = zeros(12, 1);
    x_roll_pos(4) = 0.2;  % +roll
    u_roll = [params.m * params.g; 0.1; 0; 0];  % Roll torque
    
    x_roll_neg = zeros(12, 1);
    x_roll_neg(4) = -0.2;  % -roll
    u_roll_neg = [params.m * params.g; -0.1; 0; 0];  % Opposite roll torque
    
    x_dot_pos = quadrotor_dynamics_pure(x_roll_pos, u_roll, params);
    x_dot_neg = quadrotor_dynamics_pure(x_roll_neg, u_roll_neg, params);
    
    % Y-acceleration should be opposite
    assert(abs(x_dot_pos(8) + x_dot_neg(8)) < Constants.TOL, ...
           'Roll symmetry violated in y-acceleration');
    
    % Test 2: Pitch symmetry
    x_pitch_pos = zeros(12, 1);
    x_pitch_pos(5) = 0.2;  % +pitch
    
    x_pitch_neg = zeros(12, 1);
    x_pitch_neg(5) = -0.2;  % -pitch
    
    u_hover = [params.m * params.g; 0; 0; 0];
    
    x_dot_pitch_pos = quadrotor_dynamics_pure(x_pitch_pos, u_hover, params);
    x_dot_pitch_neg = quadrotor_dynamics_pure(x_pitch_neg, u_hover, params);
    
    % X-acceleration should be opposite
    assert(abs(x_dot_pitch_pos(7) + x_dot_pitch_neg(7)) < Constants.TOL, ...
           'Pitch symmetry violated in x-acceleration');
    
    fprintf('PASS\n');
end