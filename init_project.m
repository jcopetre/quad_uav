function init_project(verbose)
% INIT_PROJECT - Initialize project environment by adding directories to MATLAB path
%
% Run this once at the start of a MATLAB session to configure paths for the
% quadrotor simulation project. All simulation and test scripts will then work.
% Automatically detects if paths are already configured and stays quiet.
%
% SYNTAX:
%   init_project()
%   init_project(verbose)
%
% INPUTS:
%   verbose - (optional) Force verbose output even if no changes made
%             true:  Always print status
%             false: Only print if paths need to be added (default)
%
% EXAMPLE:
%   >> init_project()              % Quiet if already initialized
%   >> init_project(true)          % Force verbose output
%   >> simulate_quadrotor_pure('basic_maneuver.wpt')
%   >> test_linear_6dof
%
% NOTE: This only needs to be run once per MATLAB session. Paths are not
% permanently saved unless you use 'savepath' after running this function.

% Author: Trey Copeland
% Date: 2025-10-10

    %% Parse inputs
    if nargin < 1
        verbose = false;  % Default: quiet mode
    end
    
    %% Determine project root
    % Assumes this file is in the project root directory
    project_root = fileparts(mfilename('fullpath'));
    
    %% Define directories to add
    dirs_to_add = {
        'vehicle';
        'trajectories';
        'control';
        'dynamics';
        'test';
        'utilities';
        'data'
    };
    
    %% Check which directories need to be added
    dirs_needed = {};
    dirs_missing = {};
    
    for i = 1:length(dirs_to_add)
        dir_path = fullfile(project_root, dirs_to_add{i});
        
        if exist(dir_path, 'dir')
            % Directory exists - check if it's on the path
            if ~contains(path, dir_path)
                dirs_needed{end+1} = dirs_to_add{i}; %#ok<AGROW>
            end
        else
            dirs_missing{end+1} = dirs_to_add{i}; %#ok<AGROW>
        end
    end
    
    %% Determine if we need to print anything
    already_initialized = isempty(dirs_needed) && isempty(dirs_missing);
    should_print = verbose || ~already_initialized;
    
    %% Print header if needed
    if should_print
        fprintf('===================================\n');
        fprintf('Quadrotor Project Initialization\n');
        fprintf('===================================\n');
        fprintf('Project root: %s\n\n', project_root);
    end
    
    %% Add directories to path and report
    if ~isempty(dirs_needed)
        if should_print
            fprintf('Adding directories to MATLAB path:\n');
        end
        
        for i = 1:length(dirs_needed)
            dir_path = fullfile(project_root, dirs_needed{i});
            addpath(dir_path);
            
            if should_print
                fprintf('  ✓ %s\n', dirs_needed{i});
            end
        end
        
        if should_print
            fprintf('\n');
        end
    elseif already_initialized && should_print
        fprintf('All directories already on path.\n\n');
    end
    
    %% Warn about missing directories
    if ~isempty(dirs_missing) && should_print
        fprintf('WARNING: Missing directories:\n');
        for i = 1:length(dirs_missing)
            fprintf('  ✗ %s\n', dirs_missing{i});
        end
        fprintf('\n');
    end
    
    %% Print footer if needed
    if should_print
        fprintf('===================================\n');
        fprintf('Initialization complete!\n');
        fprintf('===================================\n\n');
        
        if ~already_initialized
            fprintf('You can now run:\n');
            fprintf('  - simulate_quadrotor_pure(''basic_maneuver.wpt'')\n');
            fprintf('  - test_linear_6dof()\n');
            fprintf('  - Any other project scripts\n\n');
            fprintf('To make paths permanent, run: savepath\n\n');
        end
    end

end