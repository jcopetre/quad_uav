classdef DataSchemas
    % DATASCHEMAS - Canonical data structure definitions for quadrotor simulation
    %
    % This class defines the standard schema for all data structures used in
    % the simulation framework. Use these schemas to validate data at load/save
    % boundaries to catch errors early.
    %
    % USAGE:
    %   schema = DataSchemas.SimulationResult();
    %   DataSchemas.validate(data, schema);
    %
    % DESIGN PRINCIPLES:
    %   1. Single source of truth for data structure definitions
    %   2. Explicit required vs optional fields
    %   3. Type specifications for validation
    %   4. Documentation embedded in schema
    
    methods (Static)
        
        function schema = SimulationResult()
            % SIMULATIONRESULT - Standard format for simulation outputs
            %
            % REQUIRED FIELDS:
            %   t           [Nx1 double]     - Time vector (seconds)
            %   x           [Nx12 double]    - State trajectory
            %   u_log       [Nx4 double]     - Control input history
            %   trajectory  [struct]         - Reference trajectory (see TrajectoryData)
            %   params      [struct]         - Vehicle parameters (see VehicleParams)
            %   metrics     [struct]         - Performance metrics (see PerformanceMetrics)
            %   config      [struct]         - Simulation configuration
            %   timestamp   [char]           - ISO 8601 timestamp
            %
            % OPTIONAL FIELDS:
            %   sim_time    [double]         - Wall-clock simulation duration (seconds)
            %   metadata    [struct]         - User-defined metadata
            
            schema = struct();
            schema.required = {'t', 'x', 'u_log', 'trajectory', 'params', 'metrics', 'config', 'timestamp'};
            schema.optional = {'sim_time', 'metadata'};
            schema.types = struct(...
                't', 'double', ...
                'x', 'double', ...
                'u_log', 'double', ...
                'trajectory', 'struct', ...
                'params', 'struct', ...
                'metrics', 'struct', ...
                'config', 'struct', ...
                'timestamp', 'char', ...
                'sim_time', 'double', ...
                'metadata', 'struct');
            schema.sizes = struct(...
                't', [NaN, 1], ...
                'x', [NaN, 12], ...
                'u_log', [NaN, 4]);
            % Nested schemas - fields that should be validated recursively
            schema.nested_schemas = struct(...
                'trajectory', 'TrajectoryData');
        end
        
        function schema = TrajectoryData()
            % TRAJECTORYDATA - Standard format for trajectory information
            %
            % REQUIRED FIELDS:
            %   time         [Nx1 double]    - Time samples (seconds)
            %   position     [Nx3 double]    - Position [x, y, z] (meters)
            %   velocity     [Nx3 double]    - Velocity [vx, vy, vz] (m/s)
            %   acceleration [Nx3 double]    - Acceleration [ax, ay, az] (m/s²)
            %   attitude     [Nx3 double]    - Euler angles [phi, theta, psi] (radians)
            %   omega        [Nx3 double]    - Angular velocity [p, q, r] (rad/s)
            %
            % OPTIONAL FIELDS:
            %   jerk         [Nx3 double]    - Jerk [jx, jy, jz] (m/s³)
            %   snap         [Nx3 double]    - Snap [sx, sy, sz] (m/s⁴)
            %   alpha        [Nx3 double]    - Angular acceleration (rad/s²)
            %   metadata     [struct]        - Generation metadata
            
            schema = struct();
            schema.required = {'time', 'position', 'velocity', 'acceleration', 'attitude', 'omega'};
            schema.optional = {'jerk', 'snap', 'alpha', 'metadata'};
            schema.types = struct(...
                'time', 'double', ...
                'position', 'double', ...
                'velocity', 'double', ...
                'acceleration', 'double', ...
                'attitude', 'double', ...
                'omega', 'double', ...
                'jerk', 'double', ...
                'snap', 'double', ...
                'alpha', 'double', ...
                'metadata', 'struct');
            schema.sizes = struct(...
                'time', [NaN, 1], ...
                'position', [NaN, 3], ...
                'velocity', [NaN, 3], ...
                'acceleration', [NaN, 3], ...
                'attitude', [NaN, 3], ...
                'omega', [NaN, 3], ...
                'jerk', [NaN, 3], ...
                'snap', [NaN, 3], ...
                'alpha', [NaN, 3]);
        end
        
        function schema = PerformanceMetrics()
            % PERFORMANCEMETRICS - Standard format for performance evaluation
            %
            % REQUIRED FIELDS:
            %   rmse_position   [double]    - Position RMSE (meters)
            %   rmse_velocity   [double]    - Velocity RMSE (m/s)
            %   rmse_attitude   [double]    - Attitude RMSE (degrees)
            %   max_position_error [double] - Maximum position error (meters)
            %   max_attitude_error [double] - Maximum attitude error (degrees)
            %   mean_thrust     [double]    - Mean thrust command (N)
            %   mean_torque     [double]    - Mean torque magnitude (N·m)
            %   max_thrust      [double]    - Peak thrust command (N)
            %   max_torque      [double]    - Peak torque magnitude (N·m)
            %
            % OPTIONAL FIELDS:
            %   settling_time   [double]    - Time to settle within tolerance (s)
            %   overshoot       [double]    - Maximum overshoot percentage
            %   control_effort  [double]    - Integrated control effort
            
            schema = struct();
            schema.required = {'rmse_position', 'rmse_velocity', 'rmse_attitude', ...
                              'max_position_error', 'max_attitude_error', ...
                              'mean_thrust', 'mean_torque', 'max_thrust', 'max_torque'};
            schema.optional = {'settling_time', 'overshoot', 'control_effort'};
            schema.types = struct(...
                'rmse_position', 'double', ...
                'rmse_velocity', 'double', ...
                'rmse_attitude', 'double', ...
                'max_position_error', 'double', ...
                'max_attitude_error', 'double', ...
                'mean_thrust', 'double', ...
                'mean_torque', 'double', ...
                'max_thrust', 'double', ...
                'max_torque', 'double', ...
                'settling_time', 'double', ...
                'overshoot', 'double', ...
                'control_effort', 'double');
        end
        
        function schema = MonteCarloResult()
            % MONTECARLORESULT - Standard format for Monte Carlo results
            %
            % REQUIRED FIELDS:
            %   n_trials        [double]        - Number of trials executed
            %   parameters      [struct]        - Perturbed parameters
            %   metrics         [struct array]  - Per-trial metrics
            %   statistics      [struct]        - Aggregate statistics
            %
            % OPTIONAL FIELDS:
            %   failed_trials   [logical array] - Trial success/failure flags
            %   correlations    [struct]        - Parameter-metric correlations
            %   timestamp       [char]          - Execution timestamp
            
            schema = struct();
            schema.required = {'n_trials', 'parameters', 'metrics', 'statistics'};
            schema.optional = {'failed_trials', 'correlations', 'timestamp'};
            schema.types = struct(...
                'n_trials', 'double', ...
                'parameters', 'struct', ...
                'metrics', 'struct', ...
                'statistics', 'struct', ...
                'failed_trials', 'logical', ...
                'correlations', 'struct', ...
                'timestamp', 'char');
        end
        
        function validate(data, schema, varname)
            % VALIDATE - Check data structure against schema
            %
            % INPUTS:
            %   data     - Structure to validate
            %   schema   - Schema from DataSchemas (e.g., DataSchemas.TrajectoryData())
            %   varname  - (optional) Variable name for error messages
            %
            % THROWS:
            %   Error if validation fails with descriptive message
            %
            % EXAMPLE:
            %   schema = DataSchemas.TrajectoryData();
            %   DataSchemas.validate(trajectory, schema, 'trajectory');
            
            if nargin < 3
                varname = 'data';
            end
            
            % Check it's a struct
            if ~isstruct(data)
                error('DataSchemas:InvalidType', ...
                      '%s must be a structure, got %s', varname, class(data));
            end
            
            % Check required fields exist
            for i = 1:length(schema.required)
                field = schema.required{i};
                if ~isfield(data, field)
                    error('DataSchemas:MissingField', ...
                          '%s is missing required field: %s', varname, field);
                end
            end
            
            % Validate types and sizes for all present fields
            all_fields = fieldnames(data);
            for i = 1:length(all_fields)
                field = all_fields{i};
                
                % Skip if not in schema (allows extra fields)
                if ~isfield(schema.types, field)
                    warning('DataSchemas:UnknownField', ...
                           '%s has undocumented field: %s (skipping validation)', ...
                           varname, field);
                    continue;
                end
                
                % Check type
                expected_type = schema.types.(field);
                if ~isa(data.(field), expected_type)
                    error('DataSchemas:InvalidType', ...
                          '%s.%s must be %s, got %s', ...
                          varname, field, expected_type, class(data.(field)));
                end
                
                % Check size if specified
                if isfield(schema, 'sizes') && isfield(schema.sizes, field)
                    expected_size = schema.sizes.(field);
                    actual_size = size(data.(field));
                    
                    % NaN means "any size" for that dimension
                    for dim = 1:length(expected_size)
                        if ~isnan(expected_size(dim)) && actual_size(dim) ~= expected_size(dim)
                            error('DataSchemas:InvalidSize', ...
                                  '%s.%s dimension %d must be %d, got %d', ...
                                  varname, field, dim, expected_size(dim), actual_size(dim));
                        end
                    end
                end
            end
            
            % Validate nested schemas if specified
            if isfield(schema, 'nested_schemas')
                nested_fields = fieldnames(schema.nested_schemas);
                for i = 1:length(nested_fields)
                    field = nested_fields{i};
                    if isfield(data, field)
                        nested_schema_name = schema.nested_schemas.(field);
                        nested_schema = DataSchemas.(nested_schema_name)();
                        nested_varname = sprintf('%s.%s', varname, field);
                        % Recursive validation
                        DataSchemas.validate(data.(field), nested_schema, nested_varname);
                    end
                end
            end
        end
        
        function data_out = migrate(data_in, target_schema)
            % MIGRATE - Convert old data format to current schema
            %
            % INPUTS:
            %   data_in       - Structure in old format
            %   target_schema - Target schema name ('SimulationResult', 'TrajectoryData', etc.)
            %
            % OUTPUTS:
            %   data_out      - Structure conforming to target schema
            %
            % EXAMPLE:
            %   % Load old .mat file
            %   old_data = load('old_results.mat');
            %   % Migrate to current format
            %   new_data = DataSchemas.migrate(old_data, 'SimulationResult');
            
            data_out = data_in;  % Start with copy
            
            switch target_schema
                case 'SimulationResult'
                    % Migrate nested trajectory if present
                    if isfield(data_out, 'trajectory')
                        data_out.trajectory = DataSchemas.migrate(data_out.trajectory, 'TrajectoryData');
                    end
                    
                case 'TrajectoryData'
                    % Handle old field names
                    if isfield(data_out, 'r_ref') && ~isfield(data_out, 'position')
                        data_out.position = data_out.r_ref;
                        data_out = rmfield(data_out, 'r_ref');
                    end
                    if isfield(data_out, 'att_ref') && ~isfield(data_out, 'attitude')
                        data_out.attitude = data_out.att_ref;
                        data_out = rmfield(data_out, 'att_ref');
                    end
                    
                otherwise
                    % Silent for unknown schemas - may not need migration
            end
        end
        
    end
end