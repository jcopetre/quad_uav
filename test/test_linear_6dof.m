% test_quadrotor_linear_6dof.m
% Unit tests for quadrotor_linear_6dof function
%
% Run this file to validate the vehicle model and LQR design

function test_quadrotor_linear_6dof()
    fprintf('Running Unit Tests: quadrotor_linear_6dof\n');
    fprintf('==========================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_default_parameters();
    test_custom_weights();
    test_state_space_dimensions();
    test_lqr_stability();
    test_hover_equilibrium();
    test_controllability_observability();
    test_invalid_inputs();
    
    fprintf('\n==========================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: Default Parameters
function test_default_parameters()
    fprintf('Test 1: Default parameters... ');
    
    params = quadrotor_linear_6dof([], [], false);  % Quiet mode
    
    % Check physical parameters exist and are positive
    assert(params.m > 0, 'Mass must be positive');
    assert(params.g > 0, 'Gravity must be positive');
    assert(params.L > 0, 'Arm length must be positive');
    assert(params.Ixx > 0 && params.Iyy > 0 && params.Izz > 0, ...
           'Inertias must be positive');
    
    % Check matrices exist and have correct dimensions
    assert(isequal(size(params.A), [12, 12]), 'A matrix size wrong');
    assert(isequal(size(params.B), [12, 4]), 'B matrix size wrong');
    assert(isequal(size(params.K), [4, 12]), 'K gain size wrong');
    
    fprintf('PASS\n');
end

%% Test 2: Custom Weights
function test_custom_weights()
    fprintf('Test 2: Custom Q and R weights... ');
    
    Q_custom = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
    R_custom = diag([0.5 2 2 2]);
    
    params = quadrotor_linear_6dof(Q_custom, R_custom, false);  % Quiet mode
    
    % Verify custom weights were stored
    assert(isequal(params.Q, Q_custom), 'Q not stored correctly');
    assert(isequal(params.R, R_custom), 'R not stored correctly');
    
    % LQR should still compute
    assert(~isempty(params.K), 'LQR gain not computed');
    
    fprintf('PASS\n');
end

%% Test 3: State-Space Dimensions
function test_state_space_dimensions()
    fprintf('Test 3: State-space matrix dimensions... ');
    
    params = quadrotor_linear_6dof([], [], false);  % Quiet mode
    
    % Check all dimensions match
    [n_a, m_a] = size(params.A);
    [n_b, m_b] = size(params.B);
    [n_c, m_c] = size(params.C);
    [n_d, m_d] = size(params.D);
    
    assert(n_a == 12 && m_a == 12, 'A dimensions incorrect');
    assert(n_b == 12 && m_b == 4, 'B dimensions incorrect');
    assert(n_c == 12 && m_c == 12, 'C dimensions incorrect');
    assert(n_d == 12 && m_d == 4, 'D dimensions incorrect');
    
    fprintf('PASS\n');
end

%% Test 4: LQR Stability
function test_lqr_stability()
    fprintf('Test 4: LQR closed-loop stability... ');
    
    params = quadrotor_linear_6dof([], [], false);  % Quiet mode
    
    % Closed-loop system: A_cl = A - B*K
    A_cl = params.A - params.B * params.K;
    
    % Check all eigenvalues have negative real part
    eigs_cl = eig(A_cl);
    assert(all(real(eigs_cl) < 0), 'System not stable!');
    
    % Verify stored poles match (with tolerance for numerical precision)
    eigs_sorted = sort(eigs_cl);
    poles_sorted = sort(params.poles);
    max_diff = max(abs(eigs_sorted - poles_sorted));
    assert(max_diff < 1e-10, ...
           sprintf('Stored poles differ by %.2e', max_diff));
    
    fprintf('PASS (max real part: %.2f)\n', max(real(eigs_cl)));
end

%% Test 5: Hover Equilibrium
function test_hover_equilibrium()
    fprintf('Test 5: Hover equilibrium check... ');
    
    params = quadrotor_linear_6dof([], [], false);  % Quiet mode
    
    % At hover: all states zero, thrust = m*g
    x_hover = zeros(12, 1);
    u_hover = params.u_hover;
    
    % Check hover thrust is correct
    assert(abs(u_hover(1) - params.m * params.g) < 1e-10, ...
           'Hover thrust incorrect');
    assert(all(abs(u_hover(2:4)) < 1e-10), ...
           'Hover torques should be zero');
    
    % Linearized dynamics at hover should give zero derivative
    x_dot = params.A * x_hover + params.B * u_hover;
    
    % Only z-acceleration should be non-zero (and should be zero due to thrust-gravity balance)
    % Actually, in linearized model around hover, x_dot should be exactly zero
    assert(all(abs(x_dot) < 1e-10), ...
           'Hover is not an equilibrium point');
    
    fprintf('PASS\n');
end

%% Test 6: Controllability and Observability
function test_controllability_observability()
    fprintf('Test 6: Controllability and observability... ');
    
    params = quadrotor_linear_6dof([], [], false);  % Quiet mode
    
    % Check controllability
    Co = ctrb(params.A, params.B);
    rank_Co = rank(Co);
    assert(rank_Co == 12, ...
           sprintf('System not fully controllable (rank=%d)', rank_Co));
    
    % Check observability
    Ob = obsv(params.A, params.C);
    rank_Ob = rank(Ob);
    assert(rank_Ob == 12, ...
           sprintf('System not fully observable (rank=%d)', rank_Ob));
    
    fprintf('PASS\n');
end

%% Test 7: Invalid Inputs
function test_invalid_inputs()
    fprintf('Test 7: Invalid input handling... ');
    
    % Test wrong Q size
    try
        quadrotor_linear_6dof(eye(10), eye(4), false);
        error('Should have thrown error for wrong Q size');
    catch ME
        assert(contains(ME.message, '12x12'), 'Wrong error message');
    end
    
    % Test wrong R size
    try
        quadrotor_linear_6dof(eye(12), eye(3), false);
        error('Should have thrown error for wrong R size');
    catch ME
        assert(contains(ME.message, '4x4'), 'Wrong error message');
    end
    
    fprintf('PASS\n');
end