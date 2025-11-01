% run_all_tests.m
% Comprehensive test runner for quadrotor control system
%
% Runs all unit tests and reports results without stopping on errors.
% Provides summary statistics and identifies failing tests.

clear; clc;

fprintf('========================================\n');
fprintf('Quadrotor Control System - Test Suite\n');
fprintf('========================================\n\n');

%% Setup test environment
try
    test_dir = fileparts(mfilename('fullpath'));
    root_dir = fileparts(test_dir);
    cd(root_dir);
    
    % Add all necessary paths
    init_project();
    
    fprintf('✓ Test environment setup complete\n\n');
catch ME
    fprintf('✗ Failed to setup test environment:\n');
    fprintf('  %s\n\n', ME.message);
    return;
end

%% Discover test files
test_files = dir(fullfile(root_dir, 'test', 'test_*.m'));

if isempty(test_files)
    fprintf('✗ No test files found in test/ directory\n');
    fprintf('  Looking for files matching: test_*.m\n\n');
    return;
end

% Build test suite from discovered files
n_tests = length(test_files);
test_suite = cell(n_tests, 2);  % {function_name, filename}

for i = 1:n_tests
    [~, func_name, ~] = fileparts(test_files(i).name);
    test_suite{i, 1} = func_name;
    test_suite{i, 2} = test_files(i).name;
end

fprintf('Discovered %d test file(s):\n', n_tests);
for i = 1:n_tests
    fprintf('  - %s\n', test_suite{i, 2});
end
fprintf('\n');

results = cell(n_tests, 3);  % {name, status, message}

%% Run each test suite
for i = 1:n_tests
    test_name = test_suite{i, 1};
    test_file = test_suite{i, 2};
    
    fprintf('Running: %s\n', test_file);
    fprintf('----------------------------------------\n');
    
    try
        % Capture output
        evalc(test_name);
        
        % If we get here, test passed
        results{i, 1} = test_name;
        results{i, 2} = 'PASS';
        results{i, 3} = '';
        fprintf('✓ PASSED\n\n');
        
    catch ME
        % Test failed
        results{i, 1} = test_name;
        results{i, 2} = 'FAIL';
        results{i, 3} = ME.message;
        
        fprintf('✗ FAILED\n');
        fprintf('  Error: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('  Location: %s (line %d)\n', ...
                    ME.stack(1).name, ME.stack(1).line);
        end
        fprintf('\n');
    end
end

%% Generate summary report
fprintf('\n========================================\n');
fprintf('TEST SUMMARY\n');
fprintf('========================================\n\n');

n_passed = sum(strcmp(results(:, 2), 'PASS'));
n_failed = sum(strcmp(results(:, 2), 'FAIL'));

% Display results table
fprintf('%-40s %s\n', 'Test File', 'Status');
fprintf('%-40s %s\n', '---------', '------');
for i = 1:n_tests
    status_str = results{i, 2};
    if strcmp(status_str, 'PASS')
        status_display = '✓ PASS';
    else
        status_display = '✗ FAIL';
    end
    fprintf('%-40s %s\n', test_suite{i, 2}, status_display);
end

fprintf('\n');
fprintf('Total Tests: %d\n', n_tests);
fprintf('Passed:      %d (%.1f%%)\n', n_passed, 100*n_passed/n_tests);
fprintf('Failed:      %d (%.1f%%)\n', n_failed, 100*n_failed/n_tests);
fprintf('\n');

%% Show detailed failure information
if n_failed > 0
    fprintf('========================================\n');
    fprintf('FAILURE DETAILS\n');
    fprintf('========================================\n\n');
    
    for i = 1:n_tests
        if strcmp(results{i, 2}, 'FAIL')
            fprintf('Test: %s\n', results{i, 1});
            fprintf('Error: %s\n', results{i, 3});
            fprintf('\n');
        end
    end
end

%% Final verdict
fprintf('========================================\n');
if n_failed == 0
    fprintf('ALL TESTS PASSED! ✓\n');
else
    fprintf('SOME TESTS FAILED ✗\n');
    fprintf('Review failure details above.\n');
end
fprintf('========================================\n\n');

%% Return status
if n_failed > 0
    fprintf('Run individual test files for more detailed output:\n');
    for i = 1:n_tests
        if strcmp(results{i, 2}, 'FAIL')
            fprintf('  >> %s\n', results{i, 1});
        end
    end
    fprintf('\n');
end
