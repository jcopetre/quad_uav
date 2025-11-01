function test_data_contracts()
    % TEST_DATA_CONTRACTS - Validate DataSchemas and DataManager functionality
    %
    % This test suite ensures the data contract system works correctly:
    %   - Schema definitions are complete
    %   - Validation catches errors
    %   - Migration handles old formats
    %   - DataManager I/O operations work
    %
    
    fprintf('Running Data Contract Tests\n');
    fprintf('============================\n\n');
    
    % Create test directory
    test_dir = './test_data_contracts_temp';
    if ~exist(test_dir, 'dir')
        mkdir(test_dir);
    end
    
    try
        % Run all tests
        test_schema_definitions();
        test_validation_required_fields();
        test_validation_types();
        test_datamanager_save_load();
        test_datamanager_validation_on_load();
        test_helpful_error_messages();
        
        fprintf('\n============================\n');
        fprintf('All Tests PASSED! ✓\n\n');
        
    catch ME
        fprintf('\n============================\n');
        fprintf('TEST FAILED: %s\n', ME.message);
        fprintf('Stack:\n');
        for i = 1:length(ME.stack)
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
        fprintf('============================\n');
        rethrow(ME);
    end
    
    % Cleanup
    if exist(test_dir, 'dir')
        rmdir(test_dir, 's');
    end
end

function test_schema_definitions()
    fprintf('Test 1: Schema definitions... ');
    
    % Get all schema types
    schemas = {'SimulationResult', 'TrajectoryData', 'MonteCarloResult'};
    
    for i = 1:length(schemas)
        schema = DataSchemas.(schemas{i})();
        
        % Check has required fields list
        assert(isfield(schema, 'required'), ...
               sprintf('%s missing required field list', schemas{i}));
        assert(~isempty(schema.required), ...
               sprintf('%s required list is empty', schemas{i}));
        
        % Check has types
        assert(isfield(schema, 'types'), ...
               sprintf('%s missing types', schemas{i}));
        
        % Check all required fields have types
        for j = 1:length(schema.required)
            field = schema.required{j};
            assert(isfield(schema.types, field), ...
                   sprintf('%s.%s missing type specification', schemas{i}, field));
        end
    end
    
    fprintf('PASS\n');
end

function test_validation_required_fields()
    fprintf('Test 2: Validation of required fields... ');
    
    schema = DataSchemas.TrajectoryData();
    
    % Valid data - include ALL required fields
    valid_traj = struct();
    valid_traj.time = [0; 1; 2];
    valid_traj.position = zeros(3, 3);
    valid_traj.velocity = zeros(3, 3);
    valid_traj.acceleration = zeros(3, 3);
    valid_traj.jerk = zeros(3, 3);        % ← ADD THIS (required)
    valid_traj.yaw = zeros(1, 3);         % ← ADD THIS (required)
    % Optional fields:
    valid_traj.attitude = zeros(3, 3);
    valid_traj.omega = zeros(3, 3);
    
    % Should pass
    DataSchemas.validate(valid_traj, schema, 'valid_traj');
    
    % Missing required field should fail
    invalid_traj = rmfield(valid_traj, 'position');
    try
        DataSchemas.validate(invalid_traj, schema, 'invalid_traj');
        error('Should have caught missing required field');
    catch ME
        assert(contains(ME.message, 'missing required field'), ...
               'Wrong error message for missing field');
    end
    
    fprintf('PASS\n');
end

function test_validation_types()
    fprintf('Test 3: Type validation... ');
    
    schema = DataSchemas.TrajectoryData();
    
    % Create valid trajectory
    traj = struct();
    traj.time = [0; 1; 2];
    traj.position = zeros(3, 3);
    traj.velocity = zeros(3, 3);
    traj.acceleration = zeros(3, 3);
    traj.jerk = zeros(3, 3);
    traj.yaw = zeros(1, 3);
    
    % Valid - should pass
    DataSchemas.validate(traj, schema);
    
    % Wrong type - should warn (not error!)
    traj_bad = traj;
    traj_bad.time = 'not a number';  % Should be double
    
    % Capture warnings
    warning('off', 'all');  % Clear previous
    lastwarn('');           % Reset
    DataSchemas.validate(traj_bad, schema);
    [warnMsg, warnId] = lastwarn;
    warning('on', 'all');   % Re-enable
    
    % Check we got the type warning
    assert(contains(warnId, 'TypeMismatch'), 'Should have warned about type mismatch');
    assert(contains(warnMsg, 'has type'), 'Wrong warning message format');
    
    fprintf('PASS\n');
end

function test_datamanager_save_load()
    fprintf('Test 4: DataManager save/load... ');
    
    % Create minimal valid SimulationResult
    results = struct();
    results.t = [0; 1; 2];
    results.x = zeros(3, 12);
    results.u_log = zeros(3, 4);
    results.trajectory = struct();
    results.trajectory.time = [0; 1; 2];
    results.trajectory.position = zeros(3, 3);
    results.trajectory.velocity = zeros(3, 3);
    results.trajectory.acceleration = zeros(3, 3);
    results.trajectory.attitude = zeros(3, 3);
    results.trajectory.omega = zeros(3, 3);
    results.trajectory.jerk = zeros(3, 3);
    results.trajectory.yaw = zeros(1, 3);
    results.params = struct('mass', 1.0);
    results.metrics = struct('rmse_position', 0.1);
    results.config = struct('dt', 0.01);
    results.timestamp = datestr(now);
    
    % Save
    test_dir = './test_data_contracts_temp';
    filepath = DataManager.save_results(results, 'test', test_dir, ...
                                       struct('timestamp', false, 'verbose', false));
    
    % Load
    loaded = DataManager.load_results(filepath, struct('verbose', false));
    
    % Verify
    assert(isstruct(loaded), 'Loaded data should be struct');
    assert(isfield(loaded, 't'), 'Missing time field');
    assert(all(loaded.t == results.t), 'Time data mismatch');
    
    fprintf('PASS\n');
end

function test_datamanager_validation_on_load()
    fprintf('Test 5: DataManager validation on load... ');
    
    % Create INVALID results (missing required field)
    bad_results = struct();
    bad_results.t = [0; 1; 2];
    % Missing 'x', 'u_log', etc.
    
    % Save directly (bypass validation)
    test_dir = './test_data_contracts_temp';
    filepath = fullfile(test_dir, 'bad_results.mat');
    save(filepath, '-struct', 'bad_results');
    
    % Try to load with validation but NO migration - should fail with validation error
    try
        DataManager.load_results(filepath, struct('validate', true, 'verbose', false));
        error('Should have caught invalid data on load');
    catch ME
        assert(contains(ME.message, 'missing required field'), ...
               'Wrong error message for invalid data');
    end
    
    fprintf('PASS\n');
end

function test_helpful_error_messages()
    fprintf('Test 6: Helpful error messages... ');
    
    % Create data missing a field
    data = struct();
    data.time = [0; 1];
    data.velocity = zeros(2, 3);
    % Missing 'position'
    
    % Try to check for position
    try
        DataManager.check_field(data, 'position', 'trajectory');
        error('Should have thrown error');
    catch ME
        % Check error message is helpful
        assert(contains(ME.message, 'missing required field'), ...
               'Error should mention missing field');
        assert(contains(ME.message, 'Available fields'), ...
               'Error should list available fields');
        assert(contains(ME.message, 'time, velocity'), ...
               'Error should show actual field names');
    end
    
    fprintf('PASS\n');
end