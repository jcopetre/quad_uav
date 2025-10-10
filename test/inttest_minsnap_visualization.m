function test_minsnap_visualization()
% TEST_MINSNAP_VISUALIZATION - Visual validation of minimum snap trajectories
%
% Creates plots for manual inspection of trajectory quality

    fprintf('Generating visualization tests...\n\n');
    
    setup_test_environment();
    params = quadrotor_linear_6dof([], [], false);
    
    %% Test Case 1: Simple Trajectory
    fprintf('Visualizing: Simple trajectory\n');
    wpt1.time = [0; 2; 4; 6]';
    wpt1.position = [0 0 0; 2 0 1; 4 2 1; 6 2 0];
    wpt1.yaw = zeros(4, 1);
    
    traj1 = generate_trajectory_minsnap(wpt1, params, 0.01);
    
    figure('Position', [100 100 1400 900]);
    plot_trajectory_analysis(traj1, wpt1, 'Simple Trajectory');
    
    %% Test Case 2: Aggressive Racing
    fprintf('Visualizing: Aggressive racing\n');
    wpt2.time = [0; 1; 2; 3; 4]';
    wpt2.position = [0 0 0; 5 0 2; 10 3 2; 10 6 1; 5 6 0];
    wpt2.velocity = [0 0 0; 3 0 1; 3 2 0; 0 2 -1; 0 0 0];
    wpt2.yaw = zeros(5, 1);
    
    traj2 = generate_trajectory_minsnap(wpt2, params, 0.01);
    
    figure('Position', [100 100 1400 900]);
    plot_trajectory_analysis(traj2, wpt2, 'Aggressive Racing');
    
    %% Test Case 3: Figure Eight
    fprintf('Visualizing: Figure eight\n');
    n = 13;
    theta = linspace(0, 2*pi, n);
    wpt3.time = linspace(0, 20, n)';
    wpt3.position = [sin(theta')*3, sin(2*theta')*1.5, ones(n,1)];
    wpt3.yaw = nan(n, 1);
    
    traj3 = generate_trajectory_minsnap(wpt3, params, 0.01);
    
    figure('Position', [100 100 1400 900]);
    plot_trajectory_analysis(traj3, wpt3, 'Figure Eight');
    
    fprintf('\nVisualization complete!\n');
end

function plot_trajectory_analysis(traj, wpt, title_str)
% PLOT_TRAJECTORY_ANALYSIS - Create comprehensive trajectory plots

    % 3D Trajectory
    subplot(2, 3, 1);
    plot3(traj.position(:,1), traj.position(:,2), traj.position(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(wpt.position(:,1), wpt.position(:,2), wpt.position(:,3), ...
          'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot3(traj.position(1,1), traj.position(1,2), traj.position(1,3), ...
          'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Trajectory');
    legend('Path', 'Waypoints', 'Start');
    view(45, 30);
    
    % Position vs Time
    subplot(2, 3, 2);
    plot(traj.time, traj.position(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.position(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.position(:,3), 'b-', 'LineWidth', 1.5);
    plot(wpt.time, wpt.position(:,1), 'ro', 'MarkerSize', 8);
    plot(wpt.time, wpt.position(:,2), 'go', 'MarkerSize', 8);
    plot(wpt.time, wpt.position(:,3), 'bo', 'MarkerSize', 8);
    grid on;
    xlabel('Time (s)'); ylabel('Position (m)');
    title('Position vs Time');
    legend('X', 'Y', 'Z');
    
    % Velocity vs Time
    subplot(2, 3, 3);
    plot(traj.time, traj.velocity(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.velocity(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.velocity(:,3), 'b-', 'LineWidth', 1.5);
    if isfield(wpt, 'velocity')
        plot(wpt.time, wpt.velocity(:,1), 'ro', 'MarkerSize', 8);
        plot(wpt.time, wpt.velocity(:,2), 'go', 'MarkerSize', 8);
        plot(wpt.time, wpt.velocity(:,3), 'bo', 'MarkerSize', 8);
    end
    grid on;
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    title('Velocity vs Time');
    legend('Vx', 'Vy', 'Vz');
    
    % Acceleration vs Time
    subplot(2, 3, 4);
    plot(traj.time, traj.acceleration(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.acceleration(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.acceleration(:,3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    title('Acceleration vs Time');
    legend('Ax', 'Ay', 'Az');
    
    % Jerk vs Time
    subplot(2, 3, 5);
    plot(traj.time, traj.jerk(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.jerk(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.jerk(:,3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Jerk (m/s³)');
    title('Jerk vs Time');
    legend('Jx', 'Jy', 'Jz');
    
    % Snap vs Time
    subplot(2, 3, 6);
    plot(traj.time, traj.snap(:,1), 'r-', 'LineWidth', 1.5); hold on;
    plot(traj.time, traj.snap(:,2), 'g-', 'LineWidth', 1.5);
    plot(traj.time, traj.snap(:,3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Snap (m/s⁴)');
    title('Snap vs Time');
    legend('Sx', 'Sy', 'Sz');
    
    sgtitle(title_str, 'FontSize', 16, 'FontWeight', 'bold');
end