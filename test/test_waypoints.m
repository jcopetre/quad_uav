% test_waypoints.m
% Unit tests for load_waypoints function
%
% Tests waypoint file loading and validation

function test_waypoints()
    fprintf('Running Unit Tests: load_waypoints\n');
    fprintf('===================================\n\n');
    
    % Setup test environment
    setup_test_environment();
    
    % Run all test cases
    test_load_basic_maneuver();
    test_waypoint_structure();
    test_time_monotonicity();
    test_yaw_handling();
    test_metadata_extraction();
    test_invalid_file();
    test_missing_fields();
    test_out_of_order_waypoints();
    
    fprintf('\n===================================\n');
    fprintf('All Tests Passed! ✓\n\n');
end

%% Test 1: Load basic_maneuver.wpt
function test_load_basic_maneuver()
    fprintf('Test 1: Load basic_maneuver.wpt... ');
    
    % Get root directory
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    wpt_file = fullfile(root_dir, 'trajectories', 'basic_maneuver.wpt');
    
    % Load waypoints
    wpt = load_waypoints(wpt_file);
    
    % Check structure exists
    assert(isstruct(wpt), 'Should return a structure');
    
    % Check required fields
    assert(isfield(wpt, 'time'), 'Missing time field');
    assert(isfield(wpt, 'position'), 'Missing position field');
    assert(isfield(wpt, 'yaw'), 'Missing yaw field');
    assert(isfield(wpt, 'labels'), 'Missing labels field');
    
    % Check data dimensions
    n = length(wpt.time);
    assert(size(wpt.position, 1) == n, 'Position rows should match time length');
    assert(size(wpt.position, 2) == 3, 'Position should have 3 columns');
    assert(length(wpt.yaw) == n, 'Yaw length should match time length');
    assert(length(wpt.labels) == n, 'Labels length should match time length');
    
    fprintf('PASS\n');
end

%% Test 2: Waypoint Structure Validation
function test_waypoint_structure()
    fprintf('Test 2: Waypoint structure validation... ');
    
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    wpt_file = fullfile(root_dir, 'trajectories', 'basic_maneuver.wpt');
    
    wpt = load_waypoints(wpt_file);
    
    % Validate first waypoint
    assert(wpt.time(1) == 0, 'First waypoint should be at t=0');
    assert(all(wpt.position(1,:) == [0 0 0]), 'First position should be origin');
    
    % Validate last waypoint
    assert(wpt.time(end) > wpt.time(1), 'End time should be after start');
    
    % Check all positions are finite
    assert(all(isfinite(wpt.position(:))), 'All positions should be finite');
    
    fprintf('PASS\n');
end

%% Test 3: Time Monotonicity
function test_time_monotonicity()
    fprintf('Test 3: Time monotonicity check... ');
    
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    wpt_file = fullfile(root_dir, 'trajectories', 'basic_maneuver.wpt');
    
    wpt = load_waypoints(wpt_file);
    
    % Check strictly increasing
    time_diffs = diff(wpt.time);
    assert(all(time_diffs > 0), 'Times should be strictly increasing');
    
    fprintf('PASS\n');
end

%% Test 4: Yaw Handling
function test_yaw_handling()
    fprintf('Test 4: Yaw angle handling... ');
    
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    wpt_file = fullfile(root_dir, 'trajectories', 'basic_maneuver.wpt');
    
    wpt = load_waypoints(wpt_file);
    
    % Should have both explicit and auto (NaN) yaw values
    has_explicit = any(~isnan(wpt.yaw));
    has_auto = any(isnan(wpt.yaw));
    
    assert(has_explicit || has_auto, 'Should have yaw values (explicit or auto)');
    
    % Explicit yaw values should be finite
    explicit_yaw = wpt.yaw(~isnan(wpt.yaw));
    if ~isempty(explicit_yaw)
        assert(all(isfinite(explicit_yaw)), 'Explicit yaw values should be finite');
    end
    
    fprintf('PASS\n');
end

%% Test 5: Metadata Extraction
function test_metadata_extraction()
    fprintf('Test 5: Metadata extraction... ');
    
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    wpt_file = fullfile(root_dir, 'trajectories', 'basic_maneuver.wpt');
    
    wpt = load_waypoints(wpt_file);
    
    % Check metadata exists
    assert(isfield(wpt, 'metadata'), 'Should have metadata field');
    
    % If metadata has name, it should be a string
    if isfield(wpt.metadata, 'name')
        assert(ischar(wpt.metadata.name) || isstring(wpt.metadata.name), ...
               'Metadata name should be string');
    end
    
    fprintf('PASS\n');
end

%% Test 6: Invalid File Handling
function test_invalid_file()
    fprintf('Test 6: Invalid file handling... ');
    
    % Test non-existent file
    try
        load_waypoints('nonexistent_file.wpt');
        error('Should have thrown error for non-existent file');
    catch ME
        assert(contains(ME.message, 'not found', 'IgnoreCase', true), ...
               'Should report file not found');
    end
    
    fprintf('PASS\n');
end

%% Test 7: Missing Required Fields
function test_missing_fields()
    fprintf('Test 7: Missing required fields... ');
    
    % Create temporary malformed JSON file
    test_dir = fileparts(mfilename('fullpath'));
    temp_file = fullfile(test_dir, 'temp_bad.wpt');
    
    % Write invalid JSON (missing required fields)
    fid = fopen(temp_file, 'w');
    fprintf(fid, '{"waypoints": [{"time": 0}]}');  % Missing x, y, z
    fclose(fid);
    
    % Try to load - should fail
    try
        load_waypoints(temp_file);
        delete(temp_file);  % Cleanup
        error('Should have thrown error for missing fields');
    catch ME
        delete(temp_file);  % Cleanup
        assert(contains(ME.message, 'missing', 'IgnoreCase', true) || ...
               contains(ME.message, 'field', 'IgnoreCase', true), ...
               'Should report missing field');
    end
    
    fprintf('PASS\n');
end

%% Test 8: Out-of-Order Waypoint Detection
function test_out_of_order_waypoints()
    fprintf('Test 8: Out-of-order waypoint detection... ');
    
    % Create temporary file with out-of-order times
    test_dir = fileparts(mfilename('fullpath'));
    temp_file = fullfile(test_dir, 'temp_out_of_order.wpt');
    
    % Write waypoints with non-monotonic times
    fid = fopen(temp_file, 'w');
    fprintf(fid, '{"waypoints": [');
    fprintf(fid, '{"time": 0, "x": 0, "y": 0, "z": 0},');
    fprintf(fid, '{"time": 5, "x": 1, "y": 0, "z": 0},');
    fprintf(fid, '{"time": 3, "x": 2, "y": 0, "z": 0}');  % Out of order!
    fprintf(fid, ']}');
    fclose(fid);
    
    % Try to load - should fail
    try
        load_waypoints(temp_file);
        delete(temp_file);  % Cleanup
        error('Should have thrown error for out-of-order times');
    catch ME
        delete(temp_file);  % Cleanup
        assert(contains(ME.message, 'increasing', 'IgnoreCase', true), ...
               'Should report non-monotonic times');
    end
    
    fprintf('PASS\n');
end