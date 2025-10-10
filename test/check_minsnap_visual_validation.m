function quick_visual_validation()
% QUICK_VISUAL_VALIDATION - Essential visual checks for minimum snap
%
% Run this to quickly validate trajectory quality by eye

    fprintf('=== Quick Visual Validation ===\n\n');
    
    setup_test_environment();
    params = quadrotor_linear_6dof([], [], false);
    
    %% Test 1: Simple L-shape (easiest to validate)
    fprintf('Test 1: Simple L-shape trajectory\n');
    
    wpt.time = [0; 3; 6]';
    wpt.position = [
        0, 0, 1;    % Start
        3, 0, 1;    % Move right
        3, 3, 1     % Move forward
    ];
    wpt.yaw = [0; 0; pi/2];  % Turn to face forward
    
    traj = generate_trajectory_minsnap(wpt, params, 0.01);
    
    % Create validation figure
    figure('Position', [100 100 1600 1000]);
    
    %% Plot 1: 3D Path - MOST IMPORTANT
    subplot(2, 3, 1);
    plot3(traj.position(:,1), traj.position(:,2), traj.position(:,3), ...
          'b-', 'LineWidth', 2);
    hold on;
    
    % Mark waypoints
    plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
          'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % Mark start
    plot3(wpt.position(1,1), wpt.position(1,2), wpt.position(1,3), ...
          'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    
    % Add labels
    for i = 1:length(wpt.time)
        text(wpt.position(i,1), wpt.position(i,2), wpt.position(i,3), ...
             sprintf('  WP%d\n  t=%.1fs', i, wpt.time(i)), ...
             'FontSize', 10, 'FontWeight', 'bold');
    end
    
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title('3D Path - Should Smoothly Round Corners', 'FontSize', 14);
    legend('Path', 'Waypoints', 'Start', 'Location', 'best');
    view(45, 30);
    
    %% Plot 2: Velocity Magnitude - CHECK FOR SMOOTH BELL CURVES
    subplot(2, 3, 2);
    vel_mag = sqrt(sum(traj.velocity.^2, 2));
    plot(traj.time, vel_mag, 'b-', 'LineWidth', 2);
    hold on;
    
    % Mark waypoint times
    for i = 1:length(wpt.time)
        xline(wpt.time(i), 'r--', 'LineWidth', 1.5);
    end
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Speed (m/s)', 'FontSize', 12);
    title('Speed - Should Be Smooth, Zero at Waypoints', 'FontSize', 14);
    
    % Check velocities at waypoints
    fprintf('  Checking velocity at waypoints:\n');
    for i = 1:length(wpt.time)
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        v = norm(traj.velocity(idx, :));
        fprintf('    WP%d (t=%.1f): v=%.4f m/s ', i, wpt.time(i), v);
        if v < 0.1
            fprintf('✓\n');
        else
            fprintf('⚠ (should be ~0)\n');
        end
    end
    
    %% Plot 3: Acceleration Components - CHECK FOR CONTINUITY
    subplot(2, 3, 3);
    plot(traj.time, traj.acceleration(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.acceleration(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.acceleration(:,3), 'b-', 'LineWidth', 1.5);
    
    % Mark waypoints
    for i = 1:length(wpt.time)
        xline(wpt.time(i), 'k--', 'Alpha', 0.3);
    end
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Acceleration (m/s²)', 'FontSize', 12);
    title('Acceleration - Should Be Continuous (No Jumps)', 'FontSize', 14);
    legend('Ax', 'Ay', 'Az', 'Location', 'best');
    
    %% Plot 4: XY Plane View - CHECK CORNER ROUNDING
    subplot(2, 3, 4);
    plot(traj.position(:,1), traj.position(:,2), 'b-', 'LineWidth', 2);
    hold on;
    plot(wpt.position(:,1), wpt.position(:,2), ...
         'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % Draw expected sharp corner for comparison
    plot([0 3 3], [0 0 3], 'k--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('Top View - Corner Should Be Rounded', 'FontSize', 14);
    legend('Actual Path', 'Waypoints', 'Sharp Corner (reference)', 'Location', 'best');
    
    % Zoom in on corner
    xlim([1.5 3.5]); ylim([-0.5 3.5]);
    
    %% Plot 5: Jerk - CHECK FOR REASONABLE BOUNDS
    subplot(2, 3, 5);
    jerk_mag = sqrt(sum(traj.jerk.^2, 2));
    plot(traj.time, jerk_mag, 'b-', 'LineWidth', 2);
    
    % Mark waypoints
    for i = 1:length(wpt.time)
        xline(wpt.time(i), 'r--', 'LineWidth', 1.5);
    end
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Jerk Magnitude (m/s³)', 'FontSize', 12);
    title('Jerk - Should Be Bounded (<50 m/s³)', 'FontSize', 14);
    
    max_jerk = max(jerk_mag);
    fprintf('\n  Max jerk: %.2f m/s³ ', max_jerk);
    if max_jerk < 50
        fprintf('✓\n');
    else
        fprintf('⚠ (unusually high)\n');
    end
    
    %% Plot 6: Snap - CHECK FOR SMOOTHNESS
    subplot(2, 3, 6);
    snap_mag = sqrt(sum(traj.snap.^2, 2));
    plot(traj.time, snap_mag, 'b-', 'LineWidth', 2);
    
    % Mark waypoints
    for i = 1:length(wpt.time)
        xline(wpt.time(i), 'r--', 'LineWidth', 1.5);
    end
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Snap Magnitude (m/s⁴)', 'FontSize', 12);
    title('Snap - Should Be Relatively Smooth', 'FontSize', 14);
    
    snap_cost = sum(snap_mag.^2) * 0.01;  % Approximate integral
    fprintf('  Snap cost: %.2e\n', snap_cost);
    
    sgtitle('Quick Visual Validation - L-Shape Trajectory', ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    %% Summary Report
    fprintf('\n=== VALIDATION SUMMARY ===\n');
    fprintf('✓ Tests to check visually:\n');
    fprintf('  1. Path smoothly rounds the corner (not sharp 90°)\n');
    fprintf('  2. Speed drops to ~0 at waypoints\n');
    fprintf('  3. Acceleration is continuous (no sudden jumps)\n');
    fprintf('  4. Jerk is bounded (<50 m/s³)\n');
    fprintf('  5. All derivatives look smooth (no spikes)\n');
    fprintf('\n');
    
    %% Additional numerical checks
    fprintf('=== NUMERICAL CHECKS ===\n');
    
    % Check waypoint accuracy
    fprintf('Waypoint position errors:\n');
    for i = 1:length(wpt.time)
        [~, idx] = min(abs(traj.time - wpt.time(i)));
        pos_err = norm(traj.position(idx, :) - wpt.position(i, :));
        fprintf('  WP%d: %.4f m ', i, pos_err);
        if pos_err < 0.01
            fprintf('✓\n');
        else
            fprintf('⚠\n');
        end
    end
    
    % Check for NaN or Inf
    fprintf('\nData integrity:\n');
    has_nan = any(isnan(traj.position(:))) || any(isnan(traj.velocity(:))) || ...
              any(isnan(traj.acceleration(:)));
    has_inf = any(isinf(traj.position(:))) || any(isinf(traj.velocity(:))) || ...
              any(isinf(traj.acceleration(:)));
    
    fprintf('  NaN values: ');
    if ~has_nan
        fprintf('None ✓\n');
    else
        fprintf('FOUND ⚠\n');
    end
    
    fprintf('  Inf values: ');
    if ~has_inf
        fprintf('None ✓\n');
    else
        fprintf('FOUND ⚠\n');
    end
    
    fprintf('\n=== END VALIDATION ===\n\n');
end