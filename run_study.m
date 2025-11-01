function run_study(config_file)
% RUN_STUDY Execute a complete quadrotor simulation study
%
% This is the main application entry point. Currently delegates to example
% scripts, but will eventually support full config-driven execution.
%
% SYNTAX:
%   run_study()                    % Uses default configuration
%   run_study(config_file)         % Uses specified config
%
% INPUTS:
%   config_file - Path to study configuration file (future)
%                 For now, runs default robustness study
%
% EXAMPLE:
%   run_study()  % Runs examples/robustness_study.m
%
% FUTURE:
%   run_study('configs/studies/sensitivity_analysis.yaml')
%
% See also: examples/robustness_study

    init_project();

    if nargin < 1
        fprintf('Running default robustness study...\n');
        fprintf('(In future, this will support config files)\n\n');
    end
    
    % For now, delegate to example script
    robustness_study;
    
    % Future: Parse config and execute study
    % config = load_study_config(config_file);
    % results = execute_study(config);
end
