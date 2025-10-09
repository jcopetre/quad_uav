% test_control_loop.m
% Unit tests for control loop functions
%
% Tests: compute_lqr_control, get_reference_state, quadrotor_closed_loop_dynamics

function test_control_loop()
    fprintf('Running Unit Tests: Control Loop\n');
    fprintf('==================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_lqr_zero_error();
    test_lqr_saturation();
    test_lqr_tracking_error();
    test_reference_lookup();
    test_reference_interpolation();
    test_reference_boundary();
    test_closed_loop_hover();
    test_closed_loop_integration();
    
    fprintf('\n==================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: LQR with Zero Error
function test_lqr_zero_error()
    fprintf('Test 1: LQR with zero tracking error... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % At reference state
    x = zeros(12, 1);
    x_ref = zeros(12, 1);
    
    u = compute_lqr_control(x, x_ref, params);
    
    % Should return hover control (no correction needed)
    error = norm(u - params.u_hover);
    assert(error < Constants.TOL, ...
           sprintf('Zero error should give u_hover, error: %.2e', error));
    
    fprintf('PASS\n');
end

%% Test 2: LQR Saturation
function test_lqr_saturation()
    fprintf('Test 2: LQR control saturation... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Large error that would cause saturation
    x = zeros(12, 1);
    x(3) = 10;  % 10m altitude error
    x_ref = zeros(12, 1);
    
    u = compute_lqr_control(x, x_ref, params);
    
    % Thrust should be saturated
    u_max = 2 * params.m * params.g;
    assert(u(1) >= 0 && u(1) <= u_max, 'Thrust should be within limits');
    
    % Torques should be within limits
    assert(all(abs(u(2:4)) <= 0.1), 'Torques should be within limits');
    
    fprintf('PASS\n');
end

%% Test 3: LQR Proportional Response
function test_lqr_tracking_error()
    fprintf('Test 3: LQR proportional response... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Small altitude error (NED: positive z = below reference)
    x = zeros(12, 1);
    x(3) = 0.1;  % 10cm below reference (positive in NED)
    x_ref = zeros(12, 1);
    
    u = compute_lqr_control(x, x_ref, params);
    
    % Thrust should be greater than hover (want to ascend back to reference)
    % In NED: being at positive z means we're too far down, need more thrust
    assert(u(1) > params.u_hover(1), ...
           sprintf('Should increase thrust to ascend, got %.4f vs hover %.4f', ...
                   u(1), params.u_hover(1)));
    
    % Small error version
    x_small = zeros(12, 1);
    x_small(3) = 0.01;  % 1cm error (still below reference)
    u_small = compute_lqr_control(x_small, x_ref, params);
    
    % Smaller error should give smaller correction
    correction_large = abs(u(1) - params.u_hover(1));
    correction_small = abs(u_small(1) - params.u_hover(1));
    assert(correction_small < correction_large, ...
           'Smaller error should give smaller correction');
    
    fprintf('PASS\n');
end

%% Test 4: Reference Lookup at Waypoint
function test_reference_lookup()
    fprintf('Test 4: Reference state lookup... ');
    
    % Create simple trajectory
    traj.time = [0; 1; 2];
    traj.position = [0 0 0; 1 0 1; 2 0 1];
    traj.velocity = [0 0 0; 1 0 1; 1 0 0];
    traj.attitude = [0 0 0; 0 0.1 0; 0 0 0];
    traj.omega = zeros(3, 3);
    
    % Lookup at exact waypoint
    x_ref = get_reference_state(1.0, traj);
    
    % Should match waypoint exactly
    expected = [1; 0; 1; 0; 0.1; 0; 1; 0; 1; 0; 0; 0];
    error = norm(x_ref - expected);
    assert(error < Constants.TOL, ...
           sprintf('Should match waypoint, error: %.2e', error));
    
    fprintf('PASS\n');
end

%% Test 5: Reference Interpolation
function test_reference_interpolation()
    fprintf('Test 5: Reference interpolation... ');
    
    % Create trajectory
    traj.time = [0; 2];
    traj.position = [0 0 0; 2 0 0];
    traj.velocity = zeros(2, 3);
    traj.attitude = zeros(2, 3);
    traj.omega = zeros(2, 3);
    
    % Interpolate at midpoint
    x_ref = get_reference_state(1.0, traj);
    
    % Position should be halfway
    assert(abs(x_ref(1) - 1.0) < Constants.TOL, 'Should interpolate position');
    
    fprintf('PASS\n');
end

%% Test 6: Reference Boundary Conditions
function test_reference_boundary()
    fprintf('Test 6: Reference boundary handling... ');
    
    % Create trajectory
    traj.time = [0; 1];
    traj.position = [0 0 0; 1 0 0];
    traj.velocity = zeros(2, 3);
    traj.attitude = zeros(2, 3);
    traj.omega = zeros(2, 3);
    
    % Before start
    x_ref_before = get_reference_state(-1.0, traj);
    assert(abs(x_ref_before(1) - 0) < Constants.TOL, 'Should clamp to start');
    
    % After end
    x_ref_after = get_reference_state(2.0, traj);
    assert(abs(x_ref_after(1) - 1) < Constants.TOL, 'Should clamp to end');
    
    fprintf('PASS\n');
end

%% Test 7: Closed-Loop Hover
function test_closed_loop_hover()
    fprintf('Test 7: Closed-loop hover dynamics... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Hover trajectory
    traj.time = [0; 10];
    traj.position = [0 0 0; 0 0 0];
    traj.velocity = zeros(2, 3);
    traj.attitude = zeros(2, 3);
    traj.omega = zeros(2, 3);
    
    % At hover state
    x = zeros(12, 1);
    
    % Evaluate dynamics
    x_dot = quadrotor_closed_loop_dynamics(0, x, params, traj);
    
    % Should be in equilibrium
    max_derivative = max(abs(x_dot));
    assert(max_derivative < Constants.TOL, ...
           sprintf('Hover should be equilibrium, max derivative: %.2e', max_derivative));
    
    fprintf('PASS\n');
end

%% Test 8: Closed-Loop Integration Test
function test_closed_loop_integration()
    fprintf('Test 8: Closed-loop ODE integration... ');
    
    params = quadrotor_linear_6dof([], [], false);
    
    % Debug: Check K matrix signs
    fprintf('\n    K(1,3) = %.4f (thrust response to z error)\n', params.K(1,3));
    
    % Simple hover trajectory (stay at origin)
    traj.time = [0; 2];
    traj.position = [0 0 0; 0 0 0];  % Stay at origin
    traj.velocity = zeros(2, 3);
    traj.attitude = zeros(2, 3);
    traj.omega = zeros(2, 3);
    
    % Start slightly below origin (negative z in NED = above ground)
    x0 = zeros(12, 1);
    x0(3) = -0.5;  % Start 0.5m above ground (negative in NED)
    
    % Simulate short duration
    tspan = [0 0.5];
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    
    [t, x] = ode45(@(t,x) quadrotor_closed_loop_dynamics(t, x, params, traj), ...
                   tspan, x0, options);
    
    % Debug output
    final_z = x(end, 3);
    initial_z = x0(3);
    
    fprintf('    Initial z=%.3f m, Final z=%.3f m\n', initial_z, final_z);
    fprintf('    Should move toward zero (reference)\n');
    
    % Should be stable
    assert(all(isfinite(x(:))), 'Solution should remain finite');
    
    % Should move toward reference (z=0)
    % Started at z=-0.5, should move toward z=0 (less negative)
    moved_toward_ref = abs(final_z - 0) < abs(initial_z - 0);
    assert(moved_toward_ref, ...
           sprintf('Should move toward ref (z=0), started at %.3f, ended at %.3f', ...
                   initial_z, final_z));
    
    fprintf('    PASS\n');
end
