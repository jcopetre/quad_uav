function init_project()
% INIT_PROJECT - Initialize project environment by adding directories to MATLAB path
%
% Run this once at the start of a MATLAB session to configure paths for the
% quadrotor simulation project. All simulation and test scripts will then work.
%
% SYNTAX:
%   init_project()
%
% EXAMPLE:
%   >> init_project()
%   >> simulate_quadrotor_pure('basic_maneuver.wpt')
%   >> test_linear_6dof
%
% NOTE: This only needs to be run once per MATLAB session. Paths are not
% permanently saved unless you use 'savepath' after running this function.


% Author: Trey Copeland
% Date: 2025-10-10

%% Determine project root
% Assumes this file is in the project root directory
project_root = fileparts(mfilename('fullpath'));

fprintf('===================================\n');
fprintf('Quadrotor Project Initialization\n');
fprintf('===================================\n');
fprintf('Project root: %s\n\n', project_root);

%% Define directories to add
dirs_to_add = {
    'vehicle';
    'trajectories';
    'control';
    'dynamics';
    'test';
    'utilities';
};

%% Add directories to path
fprintf('Adding directories to MATLAB path:\n');
for i = 1:length(dirs_to_add)
    dir_path = fullfile(project_root, dirs_to_add{i});
    
    if exist(dir_path, 'dir')
        addpath(dir_path);
        fprintf('  ✓ %s\n', dirs_to_add{i});
    else
        fprintf('  ✗ %s (not found)\n', dirs_to_add{i});
    end
end

fprintf('\n===================================\n');
fprintf('Initialization complete!\n');
fprintf('===================================\n\n');

fprintf('You can now run:\n');
fprintf('  - simulate_quadrotor_pure(''basic_maneuver.wpt'')\n');
fprintf('  - test_linear_6dof()\n');
fprintf('  - Any other project scripts\n\n');

fprintf('To make paths permanent, run: savepath\n\n');

end
