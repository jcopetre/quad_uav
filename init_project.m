function init_project()
% INIT_PROJECT Initialize quadrotor LQR control framework
%
% Adds all necessary directories to the MATLAB path for the quadrotor
% simulation and control framework.
%
% USAGE:
%   init_project
%   init_project()
%
% DIRECTORY STRUCTURE:
%   src/simulation/   - Core simulation functions
%   src/monte_carlo/  - Monte Carlo robustness analysis
%   src/trajectory/   - Trajectory generation and loading
%   src/output/       - Figure and report generation
%   src/config/       - Configuration management (future)
%   src/utils/        - Utility functions and helpers
%   control/          - Control law implementations
%   dynamics/         - Vehicle dynamics models
%   vehicle/          - Vehicle parameters and definitions
%   trajectories/     - Waypoint files (*.wpt)
%   examples/         - Example scripts and workflows
%   test/             - Unit and integration tests
%
% NOTES:
%   - Run this function at the start of each MATLAB session
%   - Or add to startup.m for automatic initialization
%   - Use 'savepath' to make changes permanent (optional)
%   - Safe to call multiple times - returns silently if already initialized
%
% See also: run_study, examples/robustness_study

    %% Get project root directory
    project_root = fileparts(mfilename('fullpath'));
    
    %% Define all required paths (relative to project root)
    required_paths = {
        '';                                 % Project root
        fullfile('src', 'simulation');      % Core simulation
        fullfile('src', 'monte_carlo');     % Monte Carlo analysis
        fullfile('src', 'trajectory');      % Trajectory generation
        fullfile('src', 'output');          % Output generation
        fullfile('src', 'config');          % Configuration (future)
        fullfile('src', 'data');            % Data management
        fullfile('src', 'utils');           % Utilities
        'control';                          % Control laws
        'dynamics';                         % Dynamics models
        'vehicle';                          % Vehicle parameters
        'trajectories';                     % Waypoint files
        'configs';                          % Config files (future)
        'examples';                         % Example scripts
        'test';                             % Unit tests
    };
    
    %% Define critical files to verify (relative to project root)
    critical_files = {
        fullfile('src', 'simulation', 'simulate_trajectory.m');
        fullfile('src', 'simulation', 'ode_simulate.m');
        fullfile('src', 'monte_carlo', 'simulate_monte_carlo.m');
        fullfile('src', 'trajectory', 'load_waypoints.m');
        fullfile('src', 'utils', 'set_default_options.m');
        fullfile('vehicle', 'quadrotor_linear_6dof.m');
        fullfile('dynamics', 'quadrotor_dynamics_pure.m');
        fullfile('src', 'data', 'DataManager.m');
    };
    
    %% Quick check: are all critical files accessible?
    % If yes, paths are already set - return silently
    all_on_path = true;
    for i = 1:length(critical_files)
        % Extract function name from file path
        [~, func_name, ~] = fileparts(critical_files{i});
        
        % Check if on path
        if isempty(which(func_name))
            all_on_path = false;
            break;
        end
    end
    
    if all_on_path
        % Paths already initialized, return silently
        return;
    end
    
    %% Add all required paths
    for i = 1:length(required_paths)
        path_to_add = fullfile(project_root, required_paths{i});
        if exist(path_to_add, 'dir') || i == 1  % Allow project root even if it's a file location
            addpath(path_to_add);
        end
    end
    
    %% Verify critical files and collect any missing ones
    missing_files = {};
    for i = 1:length(critical_files)
        file_path = fullfile(project_root, critical_files{i});
        if ~exist(file_path, 'file')
            missing_files{end+1} = critical_files{i}; %#ok<AGROW>
        end
    end
    
    %% Report status
    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('QUADROTOR LQR CONTROL FRAMEWORK\n');
    fprintf('================================================================\n');
    fprintf('Project root: %s\n', project_root);
    fprintf('\n');
    
    if isempty(missing_files)
        fprintf('✓ All paths initialized successfully\n');
        fprintf('✓ Critical files verified (%d/%d)\n', length(critical_files), length(critical_files));
        fprintf('\n💡 Call savepath to make these changes permanent\n');
    else
        fprintf('⚠ Warning: %d/%d critical files not found:\n', ...
                length(missing_files), length(critical_files));
        for i = 1:length(missing_files)
            fprintf('  ✗ %s\n', missing_files{i});
        end
        fprintf('\n');
        fprintf('This may indicate:\n');
        fprintf('  1. Project restructuring incomplete\n');
        fprintf('  2. Missing dependencies\n');
        fprintf('  3. Incorrect working directory\n');
        fprintf('\nCurrent directory: %s\n', pwd);
    end
    
    fprintf('\n');
    fprintf('Usage:\n');
    fprintf('  run_study()                      - Run complete study\n');
    fprintf('  examples/robustness_study.m      - Robustness analysis example\n');
    fprintf('  simulate_trajectory(...)         - Single trajectory simulation\n');
    fprintf('================================================================\n');
    fprintf('\n');
    
end