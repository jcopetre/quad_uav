function trajectory = generate_trajectory_minsnap(waypoints, params, dt)
% GENERATE_TRAJECTORY_MINSNAP - Minimum snap trajectory optimization
%
% Generates a smooth trajectory by minimizing the integral of snap
% (4th derivative of position) subject to waypoint constraints.
%
% SYNTAX:
%   trajectory = generate_trajectory_minsnap(waypoints, params)
%   trajectory = generate_trajectory_minsnap(waypoints, params, dt)
%
% INPUTS:
%   waypoints - Structure containing:
%               .time     - Time vector (Nx1) [s]
%               .position - Position matrix (Nx3) [m]
%               .velocity - (optional) Velocity matrix (Nx3) [m/s]
%               .acceleration - (optional) Acceleration matrix (Nx3) [m/s²]
%               .yaw      - Yaw angles (Nx1) [rad], NaN = auto
%   params    - Parameter structure from quadrotor_linear_6dof
%   dt        - (optional) Time step [s], default 0.01
%
% OUTPUTS:
%   trajectory - Structure containing:
%                .time         - Time vector (Mx1)
%                .position     - Position [x,y,z] (Mx3)
%                .velocity     - Velocity (Mx3)
%                .acceleration - Acceleration (Mx3)
%                .jerk         - Jerk (Mx3)
%                .snap         - Snap (Mx3)
%                .attitude     - Euler angles [phi,theta,psi] (Mx3)
%                .omega        - Angular velocity (Mx3)
%
% THEORY:
%   Minimizes: J = Σᵢ ∫ₜᵢᵗⁱ⁺¹ ||d⁴r/dt⁴||² dt
%
%   Subject to boundary conditions at waypoints:
%     - Position (always enforced)
%     - Velocity (enforced if specified, else 0)
%     - Acceleration (enforced if specified, else 0)
%     - Jerk (set to 0 for smoothness)
%
%   Uses 7th-order polynomials for each segment, solved as QP problem.
%
% REFERENCES:
%   Mellinger, D., & Kumar, V. (2011). "Minimum snap trajectory 
%   generation and control for quadrotors." ICRA 2011.
%
% EXAMPLE:
%   wpt.time = [0; 2; 5]';
%   wpt.position = [0 0 0; 1 0 1; 2 0 1];
%   wpt.velocity = [0 0 0; 0.5 0 0; 0 0 0];  % Flythrough at middle
%   wpt.yaw = [0; NaN; 0];
%   
%   params = quadrotor_linear_6dof();
%   traj = generate_trajectory_minsnap(wpt, params);
%
% See also: generate_trajectory, load_waypoints, quadrotor_linear_6dof

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Input validation and defaults
if nargin < 3 || isempty(dt)
    dt = 0.01;
end

if isstruct(waypoints)
    wpt_time = waypoints.time;
    wpt_pos = waypoints.position;
    wpt_yaw = waypoints.yaw;
    
    % Check if velocity was explicitly provided
    if isfield(waypoints, 'velocity') && ~isempty(waypoints.velocity)
        wpt_vel = waypoints.velocity;
        vel_specified = true(size(wpt_pos, 1), 1);  % All specified
    else
        wpt_vel = zeros(size(wpt_pos));  % Default zeros
        vel_specified = false(size(wpt_pos, 1), 1);  % None specified
        vel_specified(1) = true;  % Start always specified (default 0)
        vel_specified(end) = true;  % End always specified (default 0)
    end
    
    % Check if acceleration was explicitly provided
    if isfield(waypoints, 'acceleration') && ~isempty(waypoints.acceleration)
        wpt_acc = waypoints.acceleration;
        acc_specified = true(size(wpt_pos, 1), 1);  % All specified
    else
        wpt_acc = zeros(size(wpt_pos));  % Default zeros
        acc_specified = false(size(wpt_pos, 1), 1);  % None specified
        acc_specified(1) = true;  % Start always specified (default 0)
        acc_specified(end) = true;  % End always specified (default 0)
    end
else
    error('Matrix input not supported for minsnap - use structure format');
end

% Validate inputs
assert(length(wpt_time) >= 2, 'Need at least 2 waypoints');
assert(all(diff(wpt_time) > 0), 'Waypoint times must be strictly increasing');
assert(size(wpt_pos, 1) == length(wpt_time), 'Position rows must match time length');
assert(size(wpt_vel, 1) == length(wpt_time), 'Velocity rows must match time length');
assert(size(wpt_acc, 1) == length(wpt_time), 'Acceleration rows must match time length');

N = length(wpt_time);
N_seg = N - 1;

fprintf('Generating minimum snap trajectory...\n');
fprintf('  Waypoints: %d\n', N);
fprintf('  Segments: %d\n', N_seg);

%% Segment times
T = diff(wpt_time);  % Duration of each segment

%% Solve minimum snap for each axis independently
% X, Y, Z are decoupled in the optimization

fprintf('  Solving QP for X axis... ');
tic;
[coeffs_x, cost_x] = solve_minsnap_1d(wpt_pos(:,1), wpt_vel(:,1), wpt_acc(:,1), T, vel_specified, acc_specified);
t_x = toc;
fprintf('%.3f s (cost: %.2e)\n', t_x, cost_x);

fprintf('  Solving QP for Y axis... ');
tic;
[coeffs_y, cost_y] = solve_minsnap_1d(wpt_pos(:,2), wpt_vel(:,2), wpt_acc(:,2), T, vel_specified, acc_specified);
t_y = toc;
fprintf('%.3f s (cost: %.2e)\n', t_y, cost_y);

fprintf('  Solving QP for Z axis... ');
tic;
[coeffs_z, cost_z] = solve_minsnap_1d(wpt_pos(:,3), wpt_vel(:,3), wpt_acc(:,3), T, vel_specified, acc_specified);
t_z = toc;
fprintf('%.3f s (cost: %.2e)\n', t_z, cost_z);

%% Evaluate trajectory at fine resolution
fprintf('  Evaluating trajectory at dt=%.3f s... ', dt);
tic;
trajectory = evaluate_minsnap_trajectory(coeffs_x, coeffs_y, coeffs_z, ...
                                         wpt_time, T, dt);
t_eval = toc;
fprintf('%.3f s (%d points)\n', t_eval, length(trajectory.time));

%% Generate yaw trajectory
fprintf('  Generating yaw trajectory... ');
tic;
trajectory.yaw = generate_yaw_trajectory(trajectory, wpt_time, wpt_yaw, dt);
t_yaw = toc;
fprintf('%.3f s\n', t_yaw);

%% Compute feedforward attitude from acceleration
fprintf('  Computing feedforward attitude... ');
tic;
trajectory = compute_feedforward_attitude(trajectory, params);
t_att = toc;
fprintf('%.3f s\n', t_att);

%% Assemble attitude vector
trajectory.attitude = [trajectory.roll, trajectory.pitch, trajectory.yaw];

%% Compute angular velocity from Euler rates
trajectory.omega = compute_angular_velocity(trajectory, dt);

%% Display summary
fprintf('\nTrajectory generation complete:\n');
fprintf('  Total computation time: %.3f s\n', t_x + t_y + t_z + t_eval + t_yaw + t_att);
fprintf('  Duration: %.2f seconds\n', trajectory.time(end) - trajectory.time(1));
fprintf('  Points: %d (dt = %.3f s)\n', length(trajectory.time), dt);
fprintf('  Max velocity: %.2f m/s\n', max(sqrt(sum(trajectory.velocity.^2, 2))));
fprintf('  Max acceleration: %.2f m/s²\n', max(sqrt(sum(trajectory.acceleration.^2, 2))));
fprintf('  Max jerk: %.2f m/s³\n', max(sqrt(sum(trajectory.jerk.^2, 2))));
fprintf('  Max snap: %.2f m/s⁴\n', max(sqrt(sum(trajectory.snap.^2, 2))));
fprintf('  Max roll: %.2f deg\n', max(abs(rad2deg(trajectory.roll))));
fprintf('  Max pitch: %.2f deg\n', max(abs(rad2deg(trajectory.pitch))));
fprintf('  Total snap cost: %.2e\n', cost_x + cost_y + cost_z);

end

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

function [coeffs, cost] = solve_minsnap_1d(pos, vel, acc, T, vel_specified, acc_specified)
% SOLVE_MINSNAP_1D - Solve minimum snap optimization for one axis
%
% INPUTS:
%   pos - Position at waypoints [N×1]
%   vel - Velocity at waypoints [N×1]
%   acc - Acceleration at waypoints [N×1]
%   T   - Segment durations [N_seg×1]
%   vel_specified - Boolean array [N×1] indicating which velocities are user-specified
%   acc_specified - Boolean array [N×1] indicating which accelerations are user-specified

    N = length(pos);
    N_seg = N - 1;
    n_coeffs = 8 * N_seg;
    
    %% Build Hessian matrix (snap cost)
    H = zeros(n_coeffs, n_coeffs);
    
    for seg = 1:N_seg
        H_seg = compute_snap_hessian(T(seg));
        idx = (seg-1)*8 + (1:8);
        H(idx, idx) = H_seg;
    end
    
    % Make H positive definite
    min_eig = min(eig(H));
    if min_eig < 1e-10
        H = H + 1e-10 * eye(n_coeffs);
    end
    
    %% Build constraint matrices
    [Aeq, beq] = build_minsnap_constraints(pos, vel, acc, T, N_seg, vel_specified, acc_specified);
    
    %% Solve QP
    if exist('quadprog', 'file')
        options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
        [c, cost] = quadprog(H, [], [], [], Aeq, beq, [], [], [], options);
    else
        warning('quadprog not found, using direct KKT solver');
        KKT = [H, Aeq'; Aeq, zeros(size(Aeq, 1))];
        rhs = [zeros(n_coeffs, 1); beq];
        sol = KKT \ rhs;
        c = sol(1:n_coeffs);
        cost = 0.5 * c' * H * c;
    end
    
    coeffs = reshape(c, 8, N_seg)';
end


function H = compute_snap_hessian(T)
% COMPUTE_SNAP_HESSIAN - Compute Hessian for snap cost integral
%
% For 7th order polynomial: p(t) = Σᵢ cᵢ*tⁱ
% Snap: d⁴p/dt⁴ = Σᵢ₌₄⁷ cᵢ * i!/(i-4)! * t^(i-4)
%
% Cost: ∫₀ᵀ (d⁴p/dt⁴)² dt
%
% INPUTS:
%   T - Segment duration
%
% OUTPUTS:
%   H - Hessian matrix (8×8)

    H = zeros(8, 8);
    
    % Only derivatives from 4th order and up contribute to snap
    for i = 4:7
        for j = 4:7
            % Coefficient for t^i in polynomial
            coeff_i = factorial(i) / factorial(i-4);
            % Coefficient for t^j in polynomial
            coeff_j = factorial(j) / factorial(j-4);
            
            % Integral: ∫₀ᵀ t^(i+j-8) dt = T^(i+j-7) / (i+j-7)
            power = i + j - 8;
            if power == -1
                integral_val = log(T);  % Special case
            else
                integral_val = T^(power+1) / (power+1);
            end
            
            H(i+1, j+1) = coeff_i * coeff_j * integral_val;
        end
    end
end


function [Aeq, beq] = build_minsnap_constraints(pos, vel, acc, T, N_seg, vel_specified, acc_specified)
% BUILD_MINSNAP_CONSTRAINTS - Build equality constraint matrices
%
% Minimum snap trajectory optimization following Mellinger & Kumar (2011).
%
% CRITICAL FIX: Interior waypoints must constrain BOTH:
%   1. The END of the segment arriving at the waypoint
%   2. The START of the segment departing from the waypoint
%
% Without both constraints, segments can "miss" waypoints while maintaining
% continuity, leading to high accelerations and poor tracking.
%
% REFERENCES:
%   Mellinger, D., & Kumar, V. (2011). "Minimum snap trajectory generation
%   and control for quadrotors." IEEE International Conference on Robotics
%   and Automation (ICRA), pp. 2520-2525.

    N = N_seg + 1;
    n_coeffs = 8 * N_seg;
    
    Aeq = [];
    beq = [];
    row = 1;

    %% START waypoint (seg 1, t=0): position, vel, acc, jerk
    idx = 0*8 + (1:8);
    Aeq(row, idx) = poly_coeffs(0, 0); beq(row) = pos(1); row = row + 1;
    Aeq(row, idx) = poly_coeffs(0, 1); beq(row) = vel(1); row = row + 1;
    Aeq(row, idx) = poly_coeffs(0, 2); beq(row) = acc(1); row = row + 1;
    Aeq(row, idx) = poly_coeffs(0, 3); beq(row) = 0; row = row + 1;

    %% INTERIOR waypoints: constrain BOTH segments at the waypoint
    for seg = 1:N_seg-1
        wp = seg + 1;  % Waypoint index (e.g., wp=2 for waypoint between seg 1 and 2)
        
        % Segment indices
        idx_end = (seg-1)*8 + (1:8);   % Segment ending at this waypoint
        idx_start = seg*8 + (1:8);      % Segment starting from this waypoint
        
        % Time values
        t_end = T(seg);    % End time of segment ending here
        t_start = 0;       % Start time of segment starting here (always 0 in local coords)
        
        %% Position constraints (ALWAYS enforced at waypoints)
        % Constrain end of arriving segment
        Aeq(row, idx_end) = poly_coeffs(t_end, 0);
        beq(row) = pos(wp);
        row = row + 1;
        
        % Constrain start of departing segment
        Aeq(row, idx_start) = poly_coeffs(t_start, 0);
        beq(row) = pos(wp);
        row = row + 1;
        
        %% Velocity constraints (only if user explicitly specified)
        if vel_specified(wp)
            % Constrain end of arriving segment
            Aeq(row, idx_end) = poly_coeffs(t_end, 1);
            beq(row) = vel(wp);
            row = row + 1;
            
            % Constrain start of departing segment
            Aeq(row, idx_start) = poly_coeffs(t_start, 1);
            beq(row) = vel(wp);
            row = row + 1;
        end
        
        %% Acceleration constraints (only if user explicitly specified)
        if acc_specified(wp)
            % Constrain end of arriving segment
            Aeq(row, idx_end) = poly_coeffs(t_end, 2);
            beq(row) = acc(wp);
            row = row + 1;
            
            % Constrain start of departing segment
            Aeq(row, idx_start) = poly_coeffs(t_start, 2);
            beq(row) = acc(wp);
            row = row + 1;
        end
    end

    %% END waypoint (last segment, t=T): position, vel, acc, jerk
    idx = (N_seg-1)*8 + (1:8);
    t_end = T(N_seg);
    Aeq(row, idx) = poly_coeffs(t_end, 0); beq(row) = pos(N); row = row + 1;
    Aeq(row, idx) = poly_coeffs(t_end, 1); beq(row) = vel(N); row = row + 1;
    Aeq(row, idx) = poly_coeffs(t_end, 2); beq(row) = acc(N); row = row + 1;
    Aeq(row, idx) = poly_coeffs(t_end, 3); beq(row) = 0; row = row + 1;

    %% Continuity at interior waypoints (for higher derivatives)
    % NOTE: Position/velocity/acceleration continuity is already enforced
    % by the waypoint constraints above. We only need continuity for
    % derivatives NOT explicitly constrained at waypoints.
    %
    % For interior waypoints WITHOUT explicit velocity constraints:
    %   - Position is constrained (waypoints above)
    %   - Velocity continuity needed
    %   - Acceleration continuity needed
    %   - Jerk continuity needed
    %   - Snap continuity needed
    %
    % For interior waypoints WITH explicit velocity constraints:
    %   - Position is constrained (waypoints above)
    %   - Velocity is constrained (waypoints above)
    %   - Acceleration continuity needed (if not constrained)
    %   - Jerk continuity needed
    %   - Snap continuity needed
    
    for seg = 1:N_seg-1
        wp = seg + 1;  % Waypoint index
        
        idx = (seg-1)*8 + (1:8);
        idx_next = seg*8 + (1:8);
        
        t = T(seg);
        t_next = 0;
        
        % Velocity continuity (only if NOT explicitly constrained)
        if ~vel_specified(wp)
            Aeq(row, idx) = poly_coeffs(t, 1);
            Aeq(row, idx_next) = -poly_coeffs(t_next, 1);
            beq(row) = 0;
            row = row + 1;
        end
        
        % Acceleration continuity (only if NOT explicitly constrained)
        if ~acc_specified(wp)
            Aeq(row, idx) = poly_coeffs(t, 2);
            Aeq(row, idx_next) = -poly_coeffs(t_next, 2);
            beq(row) = 0;
            row = row + 1;
        end
        
        % Jerk continuity (always needed for smoothness)
        Aeq(row, idx) = poly_coeffs(t, 3);
        Aeq(row, idx_next) = -poly_coeffs(t_next, 3);
        beq(row) = 0;
        row = row + 1;
        
        % Snap continuity (always needed for C⁴ continuity)
        Aeq(row, idx) = poly_coeffs(t, 4);
        Aeq(row, idx_next) = -poly_coeffs(t_next, 4);
        beq(row) = 0;
        row = row + 1;
    end
end


function c = poly_coeffs(t, deriv)
% POLY_COEFFS - Get polynomial coefficients for evaluation
%
% For polynomial: p(t) = c₀ + c₁t + c₂t² + ... + c₇t⁷
%
% INPUTS:
%   t     - Time value
%   deriv - Derivative order (0=position, 1=velocity, 2=accel, 3=jerk, 4=snap)
%
% OUTPUTS:
%   c - Coefficient vector [1×8]

    c = zeros(1, 8);
    
    switch deriv
        case 0  % Position: p(t)
            for i = 0:7
                c(i+1) = t^i;
            end
            
        case 1  % Velocity: dp/dt
            for i = 1:7
                c(i+1) = i * t^(i-1);
            end
            
        case 2  % Acceleration: d²p/dt²
            for i = 2:7
                c(i+1) = i * (i-1) * t^(i-2);
            end
           
        case 3  % Jerk: d³p/dt³
            for i = 3:7
                c(i+1) = i * (i-1) * (i-2) * t^(i-3);
            end
            
        case 4  % Snap: d⁴p/dt⁴
            for i = 4:7
                c(i+1) = i * (i-1) * (i-2) * (i-3) * t^(i-4);
            end
            
        otherwise
            error('Derivative order %d not supported', deriv);
    end
end


function trajectory = evaluate_minsnap_trajectory(coeffs_x, coeffs_y, coeffs_z, ...
                                                   wpt_time, T, dt)
% EVALUATE_MINSNAP_TRAJECTORY - Evaluate polynomial trajectory at fine resolution
%
% INPUTS:
%   coeffs_x, coeffs_y, coeffs_z - Polynomial coefficients [N_seg × 8]
%   wpt_time - Waypoint times [N×1]
%   T - Segment durations [N_seg×1]
%   dt - Time step
%
% OUTPUTS:
%   trajectory - Structure with position, velocity, accel, jerk, snap

    N_seg = size(coeffs_x, 1);
    
    % Create time vector
    t_start = wpt_time(1);
    t_end = wpt_time(end);
    time = (t_start:dt:t_end)';
    if time(end) < t_end
        time(end+1) = t_end;
    end
    
    n_points = length(time);
    
    % Preallocate
    trajectory.time = time;
    trajectory.position = zeros(n_points, 3);
    trajectory.velocity = zeros(n_points, 3);
    trajectory.acceleration = zeros(n_points, 3);
    trajectory.jerk = zeros(n_points, 3);
    trajectory.snap = zeros(n_points, 3);
    
    % Evaluate each time point
    for i = 1:n_points
        t_curr = time(i);
        
        % Find which segment this time belongs to
        seg = find(t_curr >= wpt_time(1:end-1) & t_curr <= wpt_time(2:end), 1);
        if isempty(seg)
            if t_curr < wpt_time(1)
                seg = 1;
            else
                seg = N_seg;
            end
        end
        
        % Time within segment
        t_seg = t_curr - wpt_time(seg);
        
        % Evaluate polynomial and derivatives
        c_x = coeffs_x(seg, :);
        c_y = coeffs_y(seg, :);
        c_z = coeffs_z(seg, :);
        
        trajectory.position(i, :) = [
            polyval_deriv(c_x, t_seg, 0), ...
            polyval_deriv(c_y, t_seg, 0), ...
            polyval_deriv(c_z, t_seg, 0)
        ];
        
        trajectory.velocity(i, :) = [
            polyval_deriv(c_x, t_seg, 1), ...
            polyval_deriv(c_y, t_seg, 1), ...
            polyval_deriv(c_z, t_seg, 1)
        ];
        
        trajectory.acceleration(i, :) = [
            polyval_deriv(c_x, t_seg, 2), ...
            polyval_deriv(c_y, t_seg, 2), ...
            polyval_deriv(c_z, t_seg, 2)
        ];
        
        trajectory.jerk(i, :) = [
            polyval_deriv(c_x, t_seg, 3), ...
            polyval_deriv(c_y, t_seg, 3), ...
            polyval_deriv(c_z, t_seg, 3)
        ];
        
        trajectory.snap(i, :) = [
            polyval_deriv(c_x, t_seg, 4), ...
            polyval_deriv(c_y, t_seg, 4), ...
            polyval_deriv(c_z, t_seg, 4)
        ];
    end
end


function val = polyval_deriv(coeffs, t, deriv)
% POLYVAL_DERIV - Evaluate polynomial or its derivative
%
% For polynomial with coefficients [c0, c1, ..., c7]
% p(t) = c0 + c1*t + c2*t² + ... + c7*t⁷
%
% INPUTS:
%   coeffs - Coefficient vector [1×8], ordered [c0, c1, ..., c7]
%   t - Time value
%   deriv - Derivative order
%
% OUTPUTS:
%   val - Evaluated value

    switch deriv
        case 0  % Position
            val = coeffs * [1; t; t^2; t^3; t^4; t^5; t^6; t^7];
            
        case 1  % Velocity
            val = coeffs * [0; 1; 2*t; 3*t^2; 4*t^3; 5*t^4; 6*t^5; 7*t^6];
            
        case 2  % Acceleration
            val = coeffs * [0; 0; 2; 6*t; 12*t^2; 20*t^3; 30*t^4; 42*t^5];
            
        case 3  % Jerk
            val = coeffs * [0; 0; 0; 6; 24*t; 60*t^2; 120*t^3; 210*t^4];
            
        case 4  % Snap
            val = coeffs * [0; 0; 0; 0; 24; 120*t; 360*t^2; 840*t^3];
            
        otherwise
            error('Derivative order %d not supported', deriv);
    end
end


function yaw = generate_yaw_trajectory(trajectory, wpt_time, wpt_yaw, dt)
% GENERATE_YAW_TRAJECTORY - Generate smooth yaw trajectory
%
% Uses minimum snap for yaw if explicit values given, or tracks velocity
% direction if auto (NaN).

    n_points = length(trajectory.time);
    yaw = zeros(n_points, 1);
    
    if all(isnan(wpt_yaw)) % All Constants.AUTO_YAW
        % All auto: calculate from velocity direction
        for i = 1:n_points
            vx = trajectory.velocity(i, 1);
            vy = trajectory.velocity(i, 2);
            speed = sqrt(vx^2 + vy^2);
            
            if speed > 0.1  % Moving
                yaw(i) = atan2(vy, vx);
            elseif i > 1
                yaw(i) = yaw(i-1);  % Maintain previous yaw
            else
                yaw(i) = 0;
            end
        end
        
    elseif all(~isnan(wpt_yaw))
        % All explicit: use minimum snap for yaw too
        % (Could implement full min-snap, but for now use simple interpolation)
        yaw = interp1(wpt_time, wpt_yaw, trajectory.time, 'pchip');
        
    else
        % Mixed: interpolate explicit, then fill auto sections
        explicit_idx = ~isnan(wpt_yaw);
        yaw = interp1(wpt_time(explicit_idx), wpt_yaw(explicit_idx), ...
                      trajectory.time, 'linear', 'extrap');
        
        % For sections between auto waypoints, use velocity direction
        for i = 1:length(wpt_time)-1
            if isnan(wpt_yaw(i)) && isnan(wpt_yaw(i+1))
                mask = trajectory.time >= wpt_time(i) & trajectory.time <= wpt_time(i+1);
                for j = find(mask)'
                    vx = trajectory.velocity(j, 1);
                    vy = trajectory.velocity(j, 2);
                    if sqrt(vx^2 + vy^2) > 0.1
                        yaw(j) = atan2(vy, vx);
                    end
                end
            end
        end
    end
end


function trajectory = compute_feedforward_attitude(trajectory, params)
% COMPUTE_FEEDFORWARD_ATTITUDE - Compute desired roll/pitch from acceleration
%
% Uses small-angle approximation:
%   theta ≈ asin(ax / g)
%   phi ≈ asin(-ay / (g * cos(theta)))

    n_points = length(trajectory.time);
    g = params.g;
    
    trajectory.roll = zeros(n_points, 1);
    trajectory.pitch = zeros(n_points, 1);
    
    for i = 1:n_points
        ax = trajectory.acceleration(i, 1);
        ay = trajectory.acceleration(i, 2);
        
        % Pitch from forward acceleration
        trajectory.pitch(i) = asin(max(-0.5, min(0.5, ax / g)));
        
        % Roll from lateral acceleration
        cos_theta = cos(trajectory.pitch(i));
        if abs(cos_theta) > 0.1
            trajectory.roll(i) = asin(max(-0.5, min(0.5, -ay / (g * cos_theta))));
        else
            trajectory.roll(i) = 0;
        end
    end

end


function omega = compute_angular_velocity(trajectory, dt)
% COMPUTE_ANGULAR_VELOCITY - Compute body rates for trajectory tracking
%
% CRITICAL FIX: The original implementation computed omega from Euler angle
% rate derivatives, which caused massive numerical errors (1800+ deg/s).
%
% ROOT CAUSE:
%   - Euler angles have discontinuities (wrap at ±180°)
%   - Numerical derivatives amplify these discontinuities
%   - Transformation W^-1 further amplifies errors near gimbal lock
%   - Result: Physically impossible angular velocities → 100% saturation
%
% NEW APPROACH:
%   For feedforward trajectories, omega should be minimal:
%   - Feedforward attitudes (roll, pitch) are kinematic - they specify
%     orientation needed for desired acceleration, NOT rotation rates
%   - Roll/pitch rates (p, q): Set to zero - let controller handle tilting
%   - Yaw rate (r): Derived from desired heading change for coordinated turns
%
% BENEFITS:
%   - Physically reasonable values (< 90 deg/s)
%   - No saturation from tracking impossible angular velocities
%   - Controller focuses on position/velocity tracking
%   - Attitude naturally follows from LQR feedback

    n_points = length(trajectory.time);
    omega = zeros(n_points, 3);
    
    % Compute desired yaw rate from trajectory
    % Use smooth derivative to avoid numerical issues
    yaw_rate = gradient(trajectory.yaw, dt);
    
    % Limit yaw rate to reasonable values (avoid wrap-around issues at ±π)
    max_yaw_rate = deg2rad(90);  % 90 deg/s is aggressive but achievable
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate));
    
    % Set only yaw rate (r component)
    omega(:, 3) = yaw_rate;
    
    % p and q remain zero - feedforward attitudes provide the reference
    % orientation, but the controller determines how fast to achieve it
    % based on the tracking error and LQR gains
end