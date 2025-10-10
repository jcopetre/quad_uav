function test_corner_rounding_vs_time()
% TEST_CORNER_ROUNDING_VS_TIME - Show how corner radius depends on time allocation

    fprintf('=== Testing Corner Rounding vs Time Allocation ===\n\n');
    
    setup_test_environment();
    params = quadrotor_linear_6dof([], [], false);
    
    % Same waypoints, different timing
    test_times = [2, 4, 6, 10];  % Seconds per segment
    
    figure('Position', [100 100 1600 800]);
    
    for idx = 1:length(test_times)
        seg_time = test_times(idx);
        
        % L-shape with variable timing
        wpt.time = [0; seg_time; 2*seg_time]';
        wpt.position = [0 0 1; 3 0 1; 3 3 1];
        wpt.yaw = [0; 0; pi/2];
        
        fprintf('Generating with segment time = %d seconds...\n', seg_time);
        traj = generate_trajectory_minsnap(wpt, params, 0.01);
        
        % Plot XY path
        subplot(2, 4, idx);
        plot(traj.position(:,1), traj.position(:,2), 'b-', 'LineWidth', 3);
        hold on;
        plot(wpt.position(:,1), wpt.position(:,2), 'ro', ...
             'MarkerSize', 12, 'MarkerFaceColor', 'r');
        
        % Sharp corner reference
        plot([0 3 3], [0 0 3], 'k--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
        
        grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title(sprintf('T=%ds per segment', seg_time));
        xlim([2 4]); ylim([-1 1]);
        
        % Measure actual corner radius
        % Find point closest to corner (3, 0)
        [~, corner_idx] = min(sqrt((traj.position(:,1)-3).^2 + ...
                                   (traj.position(:,2)-0).^2));
        
        % Look at points around corner
        window = 20;
        idx_start = max(1, corner_idx - window);
        idx_end = min(length(traj.time), corner_idx + window);
        
        corner_region_x = traj.position(idx_start:idx_end, 1);
        corner_region_y = traj.position(idx_start:idx_end, 2);
        
        % Estimate corner radius (distance from corner waypoint)
        max_deviation = max(sqrt((corner_region_x - 3).^2 + corner_region_y.^2));
        
        fprintf('  Estimated corner rounding: %.3f m\n', max_deviation);
        
        % Plot velocity magnitude
        subplot(2, 4, idx + 4);
        vel_mag = sqrt(sum(traj.velocity.^2, 2));
        plot(traj.time, vel_mag, 'b-', 'LineWidth', 2);
        grid on;
        xlabel('Time (s)'); ylabel('Speed (m/s)');
        title(sprintf('Max speed: %.2f m/s', max(vel_mag)));
        
        % Mark waypoint times
        for i = 1:3
            xline(wpt.time(i), 'r--', 'LineWidth', 1);
        end
    end
    
    sgtitle('Corner Rounding vs Time Allocation - Longer Time = Smoother Corner', ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    fprintf('\n=== KEY INSIGHT ===\n');
    fprintf('As segment time increases, corner rounding improves!\n');
    fprintf('With short times (2-3s), corner CANNOT be rounded much.\n');
    fprintf('With longer times (6-10s), corner becomes nicely rounded.\n');
    fprintf('==================\n\n');
end