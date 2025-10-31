function test_data_contracts()
    % TEST_DATA_CONTRACTS - Validate DataSchemas and DataManager functionality
    %
    % This test suite ensures the data contract system works correctly:
    %   - Schema definitions are complete
    %   - Validation catches errors
    %   - Migration handles old formats
    %   - DataManager I/O operations work
    %
    % NOTE: Requires +data package in MATLAB path
    %       Files should be in: <project_root>/+data/
    
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
        test_validation_sizes();
        test_migration_old_trajectory();
        test_datamanager_save_load();
        test_datamanager_validation_on_load();
        test_backward_compatibility();
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

%% Test 1: Schema Definitions Complete
function test_schema_definitions()
    fprintf('Test 1: Schema definitions... ');
    
    % Get all schema types
    schemas = {'SimulationResult', 'TrajectoryData', 'PerformanceMetrics', 'MonteCarloResult'};
    
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

%% Test 2: Validation Catches Missing Required Fields
function test_validation_required_fields()
    fprintf('Test 2: Validation of required fields... ');
    
    schema = DataSchemas.TrajectoryData();
    
    % Valid data
    valid_traj = struct();
    valid_traj.time = [0; 1; 2];
    valid_traj.position = zeros(3, 3);
    valid_traj.velocity = zeros(3, 3);
    valid_traj.acceleration = zeros(3, 3);
    valid_traj.attitude = zeros(3, 3);
    valid_traj.omega = zeros(3, 3);
    
    % Should pass
    DataSchemas.validate(valid_traj, schema);
    
    % Missing required field should fail
    invalid_traj = rmfield(valid_traj, 'position');
    try
        DataSchemas.validate(invalid_traj, schema);
        error('Should have caught missing required field');
    catch ME
        assert(contains(ME.message, 'missing required field'), ...
               'Wrong error message for missing field');
    end
    
    fprintf('PASS\n');
end

%% Test 3: Validation Checks Types
function test_validation_types()
    fprintf('Test 3: Type validation... ');
    
    schema = DataSchemas.TrajectoryData();
    
    % Create valid trajectory
    traj = struct();
    traj.time = [0; 1; 2];
    traj.position = zeros(3, 3);
    traj.velocity = zeros(3, 3);
    traj.acceleration = zeros(3, 3);
    traj.attitude = zeros(3, 3);
    traj.omega = zeros(3, 3);
    
    % Valid - should pass
    DataSchemas.validate(traj, schema);
    
    % Wrong type - should fail
    traj_bad = traj;
    traj_bad.time = 'not a number';  % Should be double
    try
        DataSchemas.validate(traj_bad, schema);
        error('Should have caught type mismatch');
    catch ME
        assert(contains(ME.message, 'must be double'), ...
               'Wrong error for type mismatch');
    end
    
    fprintf('PASS\n');
end

%% Test 4: Validation Checks Sizes
function test_validation_sizes()
    fprintf('Test 4: Size validation... ');
    
    schema = DataSchemas.TrajectoryData();
    
    % Create valid trajectory
    traj = struct();
    traj.time = [0; 1; 2];
    traj.position = zeros(3, 3);
    traj.velocity = zeros(3, 3);
    traj.acceleration = zeros(3, 3);
    traj.attitude = zeros(3, 3);
    traj.omega = zeros(3, 3);
    
    % Valid - should pass
    DataSchemas.validate(traj, schema);
    
    % Wrong size - should fail
    traj_bad = traj;
    traj_bad.position = zeros(3, 5);  % Should be Nx3, not Nx5
    try
        DataSchemas.validate(traj_bad, schema);
        error('Should have caught size mismatch');
    catch ME
        assert(contains(ME.message, 'dimension'), ...
               'Wrong error for size mismatch');
    end
    
    fprintf('PASS\n');
end

%% Test 5: Migration Handles Old Field Names
function test_migration_old_trajectory()
    fprintf('Test 5: Migration of old formats... ');
    
    % Create trajectory with OLD field names
    old_traj = struct();
    old_traj.time = [0; 1; 2];
    old_traj.r_ref = zeros(3, 3);      % OLD name
    old_traj.att_ref = zeros(3, 3);    % OLD name
    old_traj.velocity = zeros(3, 3);
    old_traj.acceleration = zeros(3, 3);
    old_traj.omega = zeros(3, 3);
    
    % Migrate
    new_traj = DataSchemas.migrate(old_traj, 'TrajectoryData');
    
    % Check new field names exist
    assert(isfield(new_traj, 'position'), 'Migration did not create position field');
    assert(isfield(new_traj, 'attitude'), 'Migration did not create attitude field');
    
    % Check old field names removed
    assert(~isfield(new_traj, 'r_ref'), 'Migration did not remove r_ref');
    assert(~isfield(new_traj, 'att_ref'), 'Migration did not remove att_ref');
    
    % Validate migrated data
    schema = DataSchemas.TrajectoryData();
    DataSchemas.validate(new_traj, schema);
    
    fprintf('PASS\n');
end

%% Test 6: DataManager Save/Load
function test_datamanager_save_load()
    fprintf('Test 6: DataManager save/load... ');
    
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

%% Test 7: DataManager Validates On Load
function test_datamanager_validation_on_load()
    fprintf('Test 7: DataManager validation on load... ');
    
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
        DataManager.load_results(filepath, struct('validate', true, 'migrate', false, 'verbose', false));
        error('Should have caught invalid data on load');
    catch ME
        assert(contains(ME.message, 'missing required field'), ...
               'Wrong error message for invalid data');
    end
    
    fprintf('PASS\n');
end

%% Test 8: Backward Compatibility
function test_backward_compatibility()
    fprintf('Test 8: Backward compatibility... ');
    
    % Create old-format results
    old_results = struct();
    old_results.t = [0; 1; 2];
    old_results.x = zeros(3, 12);
    old_results.u_log = zeros(3, 4);
    old_results.trajectory = struct();
    old_results.trajectory.time = [0; 1; 2];
    old_results.trajectory.r_ref = zeros(3, 3);      % OLD field name
    old_results.trajectory.att_ref = zeros(3, 3);    % OLD field name
    old_results.trajectory.velocity = zeros(3, 3);
    old_results.trajectory.acceleration = zeros(3, 3);
    old_results.trajectory.omega = zeros(3, 3);
    old_results.params = struct('mass', 1.0);
    old_results.metrics = struct('rmse_position', 0.1);
    old_results.config = struct('dt', 0.01);
    old_results.timestamp = datestr(now);
    
    % Save old format
    test_dir = './test_data_contracts_temp';
    filepath = fullfile(test_dir, 'old_format.mat');
    save(filepath, '-struct', 'old_results');
    
    % Load with auto-migration
    loaded = DataManager.load_results(filepath, struct('migrate', true, 'verbose', false));
    
    % Check migration worked
    assert(isfield(loaded.trajectory, 'position'), 'Migration failed: missing position');
    assert(isfield(loaded.trajectory, 'attitude'), 'Migration failed: missing attitude');
    
    fprintf('PASS\n');
end

%% Test 9: Helpful Error Messages
function test_helpful_error_messages()
    fprintf('Test 9: Helpful error messages... ');
    
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