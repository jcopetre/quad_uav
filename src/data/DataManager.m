classdef DataManager
    % DATAMANAGER - Centralized data loading/saving with validation
    %
    % This class provides a single interface for all data I/O operations in the
    % simulation framework. It automatically validates data against schemas and
    % provides consistent error messages.
    %
    % BENEFITS:
    %   - Single point of failure → easier debugging
    %   - Automatic validation → catch errors at boundaries
    %   - Consistent interface → less code duplication
    %
    % USAGE:
    %   % Save simulation results
    %   DataManager.save_results(results, 'nominal', results_dir);
    %
    %   % Load with automatic validation
    %   results = DataManager.load_results('results/nominal.mat');
    %
    %   % Load trajectory
    %   traj = DataManager.load_trajectory('my_trajectory.mat');
    
    methods (Static)
        
        function filepath = save_results(results, label, output_dir, options)
            % SAVE_RESULTS - Save simulation results with validation
            %
            % INPUTS:
            %   results    - SimulationResult structure
            %   label      - Descriptive label for file (e.g., 'nominal')
            %   output_dir - Output directory
            %   options    - (optional) Struct with:
            %                .validate - Validate before saving (default: true)
            %                .timestamp - Add timestamp to filename (default: true)
            %                .verbose - Print status messages (default: false)
            %
            % OUTPUTS:
            %   filepath   - Full path to saved file
            %
            % EXAMPLE:
            %   filepath = DataManager.save_results(results, 'nominal', './results');
            
            if nargin < 4
                options = struct();
            end
            if ~isfield(options, 'validate'), options.validate = true; end
            if ~isfield(options, 'timestamp'), options.timestamp = true; end
            if ~isfield(options, 'verbose'), options.verbose = false; end
            
            % Validate before saving
            if options.validate
                schema = DataSchemas.SimulationResult();
                DataSchemas.validate(results, schema, 'results');
            end
            
            % Create output directory if needed
            if ~exist(output_dir, 'dir')
                mkdir(output_dir);
                if options.verbose
                    fprintf('Created directory: %s\n', output_dir);
                end
            end
            
            % Generate filename
            if options.timestamp
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                filename = sprintf('%s_%s.mat', label, timestamp);
            else
                filename = sprintf('%s.mat', label);
            end
            filepath = fullfile(output_dir, filename);
            
            % Save with -struct flag to unpack fields as top-level variables
            save(filepath, '-struct', 'results', '-v7.3');
            
            if options.verbose
                info = dir(filepath);
                fprintf('Saved: %s (%.2f MB)\n', filepath, info.bytes / 1e6);
            end
        end
        
        function results = load_results(filepath, options)
            % LOAD_RESULTS - Load simulation results with validation
            %
            % INPUTS:
            %   filepath - Path to .mat file
            %   options  - (optional) Struct with:
            %              .validate - Validate after loading (default: true)
            %              .verbose - Print status messages (default: false)
            %
            % OUTPUTS:
            %   results  - SimulationResult structure
            %
            % EXAMPLE:
            %   results = DataManager.load_results('results/nominal.mat');
            
            if nargin < 2
                options = struct();
            end
            if ~isfield(options, 'validate'), options.validate = true; end
            if ~isfield(options, 'verbose'), options.verbose = false; end
            
            % Check file exists
            if ~exist(filepath, 'file')
                error('DataManager:FileNotFound', ...
                      'Results file not found: %s', filepath);
            end
            
            % Load data
            results = load(filepath);
            
            if options.verbose
                fprintf('Loaded: %s\n', filepath);
            end
            
            % Validate if requested
            if options.validate
                schema = DataSchemas.SimulationResult();
                DataSchemas.validate(results, schema, 'results');
            end
        end
        
        function trajectory = load_trajectory(filepath_or_struct, options)
            % LOAD_TRAJECTORY - Load trajectory with validation
            %
            % INPUTS:
            %   filepath_or_struct - Path to .mat file OR struct from memory
            %   options - (optional) Struct with:
            %             .validate - Validate after loading (default: true)
            %             .verbose - Print status messages (default: false)
            %
            % OUTPUTS:
            %   trajectory - TrajectoryData structure
            %
            % EXAMPLE:
            %   traj = DataManager.load_trajectory('my_trajectory.mat');
            
            if nargin < 2
                options = struct();
            end
            if ~isfield(options, 'validate'), options.validate = true; end
            if ~isfield(options, 'verbose'), options.verbose = false; end
            
            % Handle both file paths and in-memory structs
            if ischar(filepath_or_struct)
                % Load from file
                if ~exist(filepath_or_struct, 'file')
                    error('DataManager:FileNotFound', ...
                          'Trajectory file not found: %s', filepath_or_struct);
                end
                trajectory = load(filepath_or_struct);
                if options.verbose
                    fprintf('Loaded trajectory: %s\n', filepath_or_struct);
                end
            else
                % Already a struct
                trajectory = filepath_or_struct;
            end
            
            % Validate if requested
            if options.validate
                schema = DataSchemas.TrajectoryData();
                DataSchemas.validate(trajectory, schema, 'trajectory');
            end
        end
        
        function filepath = save_monte_carlo(mc_results, output_dir, options)
            % SAVE_MONTE_CARLO - Save Monte Carlo results with validation
            %
            % INPUTS:
            %   mc_results - Monte Carlo results structure
            %   output_dir - Output directory
            %   options    - (optional) Struct with:
            %                .validate - Validate before saving (default: true)
            %                .verbose - Print status messages (default: false)
            %
            % OUTPUTS:
            %   filepath   - Full path to saved file
            
            if nargin < 3
                options = struct();
            end
            if ~isfield(options, 'validate'), options.validate = true; end
            if ~isfield(options, 'verbose'), options.verbose = false; end
            
            % Validate if requested
            if options.validate
                schema = DataSchemas.MonteCarloResult();
                DataSchemas.validate(mc_results, schema, 'mc_results');
            end
            
            % Create directory if needed
            if ~exist(output_dir, 'dir')
                mkdir(output_dir);
            end
            
            % Save
            filepath = fullfile(output_dir, 'monte_carlo.mat');
            save(filepath, 'mc_results', '-v7.3');
            
            if options.verbose
                info = dir(filepath);
                fprintf('Saved: %s (%.2f MB)\n', filepath, info.bytes / 1e6);
            end
        end
        
        function mc_results = load_monte_carlo(filepath, options)
            % LOAD_MONTE_CARLO - Load Monte Carlo results with validation
            %
            % INPUTS:
            %   filepath - Path to .mat file
            %   options  - (optional) Struct with:
            %              .validate - Validate after loading (default: true)
            %              .verbose - Print status messages (default: false)
            %
            % OUTPUTS:
            %   mc_results - MonteCarloResult structure
            
            if nargin < 2
                options = struct();
            end
            if ~isfield(options, 'validate'), options.validate = true; end
            if ~isfield(options, 'verbose'), options.verbose = false; end
            
            if ~exist(filepath, 'file')
                error('DataManager:FileNotFound', ...
                      'Monte Carlo file not found: %s', filepath);
            end
            
            % Load file (contains variable named 'mc_results')
            data = load(filepath);
            
            % Extract the mc_results variable
            if ~isfield(data, 'mc_results')
                error('DataManager:InvalidFormat', ...
                      'File does not contain mc_results variable. Found: %s', ...
                      strjoin(fieldnames(data), ', '));
            end
            mc_results = data.mc_results;
            
            % Validate
            if options.validate
                schema = DataSchemas.MonteCarloResult();
                DataSchemas.validate(mc_results, schema, 'mc_results');
            end
            
            % Verbose output
            if options.verbose
                fprintf('Loaded Monte Carlo results: %s\n', filepath);
                if isfield(mc_results, 'config') && isfield(mc_results.config, 'N_trials')
                    fprintf('  Trials: %d\n', mc_results.config.N_trials);
                end
            end
        end
        
        function check_field(data, fieldname, context)
            % CHECK_FIELD - Helper to verify field exists with helpful error
            %
            % INPUTS:
            %   data      - Structure to check
            %   fieldname - Field name to verify
            %   context   - Context string for error message
            %
            % THROWS:
            %   Error with helpful message if field missing
            %
            % EXAMPLE:
            %   DataManager.check_field(trajectory, 'position', 'trajectory');
            
            if ~isfield(data, fieldname)
                available = strjoin(fieldnames(data), ', ');
                error('DataManager:MissingField', ...
                      ['%s is missing required field: %s\n' ...
                       'Available fields: %s'], ...
                      context, fieldname, available);
            end
        end
        
        function summary = summarize_dataset(filepath)
            % SUMMARIZE_DATASET - Quick inspection of .mat file contents
            %
            % INPUTS:
            %   filepath - Path to .mat file
            %
            % OUTPUTS:
            %   summary  - Structure with metadata about contents
            %
            % EXAMPLE:
            %   DataManager.summarize_dataset('results/nominal.mat');
            
            if ~exist(filepath, 'file')
                error('DataManager:FileNotFound', 'File not found: %s', filepath);
            end
            
            % Get file info
            info = dir(filepath);
            summary.filepath = filepath;
            summary.size_mb = info.bytes / 1e6;
            summary.modified = info.date;
            
            % Load and inspect
            data = load(filepath);
            summary.fields = fieldnames(data);
            summary.n_fields = length(summary.fields);
            
            % Try to identify type
            if all(ismember({'t', 'x', 'u_log', 'trajectory'}, summary.fields))
                summary.type = 'SimulationResult';
            elseif all(ismember({'time', 'position', 'velocity'}, summary.fields))
                summary.type = 'TrajectoryData';
            elseif isfield(data, 'mc_results') && all(ismember({'config', 'trials', 'statistics'}, fieldnames(data.mc_results)))
                summary.type = 'MonteCarloResult';
            else
                summary.type = 'Unknown';
            end
            
            % Print summary
            fprintf('\n=== Dataset Summary ===\n');
            fprintf('File: %s\n', filepath);
            fprintf('Size: %.2f MB\n', summary.size_mb);
            fprintf('Type: %s\n', summary.type);
            fprintf('Fields: %s\n', strjoin(summary.fields, ', '));
            fprintf('======================\n\n');
        end
        
    end
end