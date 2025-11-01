classdef DataSchemas
    % DATASCHEMAS - Validation schemas for simulation data structures
    %
    % This class defines the expected structure of all data types in the
    % simulation framework and provides validation methods.
    %
    % SCHEMAS:
    %   SimulationResult  - Single simulation run output
    %   MonteCarloResult  - Monte Carlo study results
    %   TrajectoryData    - Generated trajectory data
    %
    % METHODS:
    %   validate(data, schema, name) - Validate data against schema
    %
    % EXAMPLE:
    %   schema = DataSchemas.SimulationResult();
    %   DataSchemas.validate(results, schema, 'results');
    
    methods (Static)
        
        function schema = SimulationResult()
            % SIMULATIONRESULT - Schema for single simulation results
            %
            % This is what simulate_trajectory() produces
            
            schema.required = {
                't',         % Time vector [s]
                'x',         % State history [n_states x n_points]
                'u_log',     % Control history [n_controls x n_points]
                'trajectory', % Reference trajectory structure
                'params',    % Vehicle parameters used
                'metrics'    % Performance metrics
            };
            
            schema.optional = {
                'config',    % Simulation configuration
                'timestamp', % When simulation was run
                'sim_time'   % Computational time
            };
            
            schema.types = struct(...
                't', 'double', ...
                'x', 'double', ...
                'u_log', 'double', ...
                'trajectory', 'struct', ...
                'params', 'struct', ...
                'metrics', 'struct', ...
                'config', 'struct', ...
                'timestamp', 'char', ...
                'sim_time', 'double' ...
            );
        end
        
        function schema = MonteCarloResult()
            % MONTECARLORESULT - Schema for Monte Carlo simulation results
            %
            % This is what simulate_monte_carlo() produces
            
            schema.required = {
                'config',       % MC configuration (N_trials, seed, parallel, etc.)
                'nominal',      % Nominal (unperturbed) simulation results
                'trials',       % Array of trial results with perturbations
                'statistics',   % Statistical analysis (mean, std, correlations)
                'timestamp',    % When MC study was run
                'elapsed_time'  % Total computation time [s]
            };
            
            schema.optional = {
                'failed_trials' % Indices of any failed trials
            };
            
            schema.types = struct(...
                'config', 'struct', ...
                'nominal', 'struct', ...
                'trials', 'struct', ...
                'statistics', 'struct', ...
                'timestamp', 'char', ...
                'elapsed_time', 'double', ...
                'failed_trials', 'double' ...
            );
        end
        
        function schema = TrajectoryData()
            % TRAJECTORYDATA - Schema for trajectory data
            %
            % This is what generate_trajectory_*() functions produce
            
            schema.required = {
                'time',         % Time vector [s]
                'position',     % Position trajectory [3 x n] [m]
                'velocity',     % Velocity trajectory [3 x n] [m/s]
                'acceleration', % Acceleration trajectory [3 x n] [m/s²]
                'yaw'           % Yaw angle trajectory [1 x n] [rad]
            };
            
            schema.optional = {
                'snap',              % Snap trajectory [3 x n] [m/s⁴]
                'yaw_rate',          % Yaw rate [1 x n] [rad/s]
                'roll',              % Reference roll [1 x n] [rad]
                'pitch',             % Reference pitch [1 x n] [rad]
                'jerk',              % Jerk trajectory [3 x n] [m/s³]
                'attitude',          % Attitude angles [3 x n] [rad]
                'omega',             % Angular velocity [3 x n] [rad/s] 
                'method',            % Generation method ('makima' or 'minsnap')
                'method_reason',     % Why this method was chosen (string)
                'selection_criteria' % Auto-selection metadata (struct)
            };
            
            schema.types = struct(...
                'time', 'double', ...
                'position', 'double', ...
                'velocity', 'double', ...
                'acceleration', 'double', ...
                'jerk', 'double', ...
                'yaw', 'double', ...
                'snap', 'double', ...
                'yaw_rate', 'double', ...
                'roll', 'double', ...
                'pitch', 'double', ...
                'attitude', 'double', ...
                'omega', 'double', ...
                'method', 'char', ...
                'method_reason', 'char', ...
                'selection_criteria', 'struct' ...
            );
        end
        
        function validate(data, schema, data_name)
            % VALIDATE - Validate data structure against schema
            %
            % INPUTS:
            %   data      - Data structure to validate
            %   schema    - Schema with required/optional/types fields
            %   data_name - Name for error messages
            %
            % THROWS:
            %   Error if validation fails
            %
            % EXAMPLE:
            %   schema = DataSchemas.SimulationResult();
            %   DataSchemas.validate(results, schema, 'results');
            
            if nargin < 3
                data_name = 'data';
            end
            
            % Check all required fields present
            for i = 1:length(schema.required)
                field = schema.required{i};
                if ~isfield(data, field)
                    available = strjoin(fieldnames(data), ', ');
                    error('DataSchemas:MissingRequiredField', ...
                          ['%s is missing required field: %s\n' ...
                           'Available fields: %s'], ...
                          data_name, field, available);
                end
            end
            
            % Check types of all present fields
            all_fields = [schema.required; schema.optional];
            for i = 1:length(all_fields)
                field = all_fields{i};
                
                if isfield(data, field)
                    % Get expected type
                    if isfield(schema.types, field)
                        expected_type = schema.types.(field);
                        actual_type = class(data.(field));
                        
                        % Check type matches (allow numeric types to be compatible)
                        if ~DataSchemas.types_compatible(actual_type, expected_type)
                            warning('DataSchemas:TypeMismatch', ...
                                   '%s.%s has type %s, expected %s', ...
                                   data_name, field, actual_type, expected_type);
                        end
                    end
                    
                    % Recursively validate nested structs
                    if isstruct(data.(field)) && ~isempty(data.(field))
                        % Check if there's a nested schema
                        nested_schema_name = [data_name '_' field];
                        if DataSchemas.has_schema(field)
                            nested_schema = DataSchemas.get_schema(field);
                            DataSchemas.validate(data.(field), nested_schema, ...
                                               sprintf('%s.%s', data_name, field));
                        end
                    end
                end
            end
            
            % Warn about undocumented fields
            data_fields = fieldnames(data);
            for i = 1:length(data_fields)
                field = data_fields{i};
                if ~ismember(field, all_fields)
                    warning('DataSchemas:UndocumentedField', ...
                           '%s has undocumented field: %s (skipping validation)', ...
                           data_name, field);
                end
            end
        end
        
        function compatible = types_compatible(actual, expected)
            % TYPES_COMPATIBLE - Check if types are compatible
            
            % Exact match
            if strcmp(actual, expected)
                compatible = true;
                return;
            end
            
            % datetime and char are compatible for timestamps  ← ADD THIS
            if (strcmp(actual, 'datetime') && strcmp(expected, 'char')) || ...
               (strcmp(actual, 'char') && strcmp(expected, 'datetime'))
                compatible = true;
                return;
            end
            
            % Numeric types are compatible
            numeric_types = {'double', 'single', 'int8', 'int16', 'int32', ...
                           'int64', 'uint8', 'uint16', 'uint32', 'uint64'};
            if ismember(actual, numeric_types) && ismember(expected, numeric_types)
                compatible = true;
                return;
            end
            
            % Otherwise not compatible
            compatible = false;
        end
        
        function has = has_schema(field_name)
            % HAS_SCHEMA - Check if nested schema exists for field
            %
            % Currently only trajectory has its own schema
            
            has = ismember(field_name, {'trajectory'});
        end
        
        function schema = get_schema(field_name)
            % GET_SCHEMA - Get schema for nested field
            
            switch field_name
                case 'trajectory'
                    schema = DataSchemas.TrajectoryData();
                otherwise
                    error('DataSchemas:NoSchema', ...
                          'No schema defined for field: %s', field_name);
            end
        end
        
    end
end