classdef Constants
    properties (Constant = true)
        G = 9.81;
    
        TOL = 1e-10;              % Standard numerical tolerance
        TOL_LOOSE = 1e-6;         % Loose tolerance for numerical integration
        TOL_STRICT = 1e-12;       % Strict tolerance for analytical results

        % Trajectory generation markers
        AUTO_YAW = NaN;           % Marker for auto-calculate yaw from velocity direction
                                  % Usage: Set with Constants.AUTO_YAW, check with isnan()
                                  % Note: Cannot use == for checking due to NaN != NaN
    
        %% Output Filenames
        % Standard filenames used throughout the project
        ANALYSIS_REPORT = 'analysis_report.txt';    % Comprehensive analysis
        RUN_LOG = 'mc_run_log.txt';                 % MC configuration log
        LATEX_SNIPPETS = 'latex_snippets.txt';      % LaTeX copy-paste snippets
        
        % Data files
        NOMINAL_DATA = 'nominal.mat';
        MC_DATA = 'monte_carlo.mat';
        SINGLE_RUN_DATA = 'results.mat';
        
        % Directory names
        FIGURES_DIR = 'figures';
        RESULTS_DIR = 'results';
        
    end
end
