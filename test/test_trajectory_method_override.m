function test_trajectory_method_override()
% TEST_TRAJECTORY_METHOD_OVERRIDE - Test trajectory generation method override
%
% Tests that the method option in simulate_trajectory correctly overrides
% automatic method selection for trajectory generation.
%
% Run this file to validate method override functionality

    fprintf('Running Unit Tests: Trajectory Method Override\n');
    fprintf('==========================================\n\n');
    
    % Setup test environment
    init_project();
    
    % Run all test cases
    test_default_auto_selection();
    test_force_makima();
    test_force_minsnap();
    test_auto_selection_logic();
    test_case_insensitive();
    
    fprintf('\n==========================================\n');
    fprintf('All Tests Passed!\n\n');
end

%% Test 1: Default Auto-Selection
function test_default_auto_selection()
    fprintf('Test 1: Default auto-selection... ');
    
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    % Don't set method - should default to 'auto'
    
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    % Verify method field exists
    assert(isfield(result, 'trajectory'), 'Result missing trajectory field');
    assert(isfield(result.trajectory, 'method'), 'Trajectory missing method field');
    
    % Should be either 'minimum_snap' or 'interpolation' based on auto-selection
    valid_methods = {'minimum_snap', 'interpolation'};
    assert(ismember(result.trajectory.method, valid_methods), ...
           sprintf('Method "%s" not in valid auto-selected methods', result.trajectory.method));
    
    fprintf('PASS (selected: %s)\n', result.trajectory.method);
end

%% Test 2: Force MAKIMA
function test_force_makima()
    fprintf('Test 2: Force MAKIMA method... ');
    
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    opts.method = 'makima';
    
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    % Verify method is interpolation
    assert(strcmp(result.trajectory.method, 'interpolation'), ...
           sprintf('Expected "interpolation", got "%s"', result.trajectory.method));
    
    fprintf('PASS\n');
end

%% Test 3: Force MinSnap
function test_force_minsnap()
    fprintf('Test 3: Force MinSnap method... ');
    
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    opts.method = 'minsnap';
    
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    % Verify method is minimum_snap
    assert(strcmp(result.trajectory.method, 'minimum_snap'), ...
           sprintf('Expected "minimum_snap", got "%s"', result.trajectory.method));
    
    fprintf('PASS\n');
end

%% Test 4: Auto-Selection Logic
function test_auto_selection_logic()
    fprintf('Test 4: Auto-selection logic (5s segments)... ');
    
    % For simple_square with 5s segments, auto should choose minimum_snap
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    opts.method = 'auto';
    
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    
    % Verify auto chose minimum_snap for long segments (5s segments in simple_square)
    assert(strcmp(result.trajectory.method, 'minimum_snap'), ...
           'Auto-selection should choose minimum_snap for 5s segments');
    
    fprintf('PASS\n');
end

%% Test 5: Case-Insensitive Method Names
function test_case_insensitive()
    fprintf('Test 5: Case-insensitive method names... ');
    
    opts.verbose = false;
    opts.plot = false;
    opts.save_results = false;
    
    % Test uppercase MAKIMA
    opts.method = 'MAKIMA';
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    assert(strcmp(result.trajectory.method, 'interpolation'), ...
           'Uppercase MAKIMA should work');
    
    % Test mixed case MinSnap
    opts.method = 'MinSnap';
    result = simulate_trajectory('simple_square.wpt', [], [], [], opts);
    assert(strcmp(result.trajectory.method, 'minimum_snap'), ...
           'Mixed case MinSnap should work');
    
    fprintf('PASS\n');
end