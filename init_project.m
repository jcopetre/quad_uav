function init_project()
% INIT_PROJECT - Initialize project environment by adding directories to MATLAB path
%
% Run this once at the start of a MATLAB session to configure paths for the
% quadrotor simulation project. All simulation and test scripts will then work.
% Automatically detects if paths are already configured and stays quiet.
%%
% NOTE: This only needs to be run once per MATLAB session. Paths are not
% permanently saved unless you use 'savepath' after running this function.

    project_root = fileparts(mfilename('fullpath'));
    
    % Add directories to path
    addpath(genpath(fullfile(project_root, 'vehicle')));
    addpath(genpath(fullfile(project_root, 'control')));
    addpath(genpath(fullfile(project_root, 'dynamics')));
    addpath(genpath(fullfile(project_root, 'trajectories')));
    addpath(genpath(fullfile(project_root, 'utilities')));
    addpath(genpath(fullfile(project_root, 'test')));
    addpath(genpath(fullfile(project_root, 'examples'))); 
    addpath(project_root);  % For Constants.m
    
    fprintf('Project paths initialized.\n');
    fprintf('Run examples: see ./examples/ directory\n');
end