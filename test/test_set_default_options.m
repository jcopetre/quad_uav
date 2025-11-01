function test_set_default_options()
% TEST_SET_DEFAULT_OPTIONS - Unit tests for set_default_options helper
%
% Tests:
%   1. Empty/missing options input
%   2. Partial options (some fields specified)
%   3. Complete options (all fields specified)
%   4. Extra fields in options (not in defaults)
%   5. Various data types (logical, numeric, string, cell)
%   6. Edge cases (empty strings, zeros, false values)
%
% See also: set_default_options

    fprintf('Running set_default_options tests...\n');
    
    test_empty_options();
    test_partial_options();
    test_complete_options();
    test_extra_fields();
    test_various_types();
    test_edge_cases();
    test_non_struct_input();
    
    fprintf('All set_default_options tests passed!\n\n');
end

function test_empty_options()
    % Test with empty/missing options
    
    defaults.verbose = true;
    defaults.plot = false;
    defaults.dt = 0.01;
    
    % Empty struct
    opts = set_default_options(struct(), defaults);
    assert(opts.verbose == true);
    assert(opts.plot == false);
    assert(opts.dt == 0.01);
    
    % Empty matrix
    opts = set_default_options([], defaults);
    assert(opts.verbose == true);
    assert(opts.plot == false);
    assert(opts.dt == 0.01);
end

function test_partial_options()
    % Test with some fields specified
    
    defaults.verbose = true;
    defaults.plot = false;
    defaults.save_results = true;
    defaults.dt = 0.01;
    
    user_opts.verbose = false;  % User override
    user_opts.dt = 0.02;        % User override
    % Missing: plot, save_results
    
    opts = set_default_options(user_opts, defaults);
    
    % User values preserved
    assert(opts.verbose == false, 'User verbose should be preserved');
    assert(opts.dt == 0.02, 'User dt should be preserved');
    
    % Defaults applied
    assert(opts.plot == false, 'Default plot should be applied');
    assert(opts.save_results == true, 'Default save_results should be applied');
end

function test_complete_options()
    % Test when all fields are specified by user
    
    defaults.verbose = true;
    defaults.plot = false;
    defaults.dt = 0.01;
    
    user_opts.verbose = false;
    user_opts.plot = true;
    user_opts.dt = 0.05;
    
    opts = set_default_options(user_opts, defaults);
    
    % All user values preserved
    assert(opts.verbose == false);
    assert(opts.plot == true);
    assert(opts.dt == 0.05);
end

function test_extra_fields()
    % Test that extra user fields are preserved
    
    defaults.verbose = true;
    defaults.plot = false;
    
    user_opts.verbose = false;
    user_opts.custom_field = 'my_data';  % Extra field
    user_opts.another_field = 42;         % Another extra
    
    opts = set_default_options(user_opts, defaults);
    
    % Defaults applied
    assert(opts.verbose == false);
    assert(opts.plot == false);
    
    % Extra fields preserved
    assert(isfield(opts, 'custom_field'));
    assert(strcmp(opts.custom_field, 'my_data'));
    assert(isfield(opts, 'another_field'));
    assert(opts.another_field == 42);
end

function test_various_types()
    % Test with different data types
    
    defaults.logical_val = true;
    defaults.numeric_val = 3.14;
    defaults.string_val = 'default_string';
    defaults.cell_val = {'a', 'b', 'c'};
    defaults.struct_val = struct('x', 1, 'y', 2);
    
    user_opts.logical_val = false;
    user_opts.numeric_val = 2.71;
    % Others use defaults
    
    opts = set_default_options(user_opts, defaults);
    
    % User values
    assert(opts.logical_val == false);
    assert(abs(opts.numeric_val - 2.71) < 1e-10);
    
    % Default values
    assert(strcmp(opts.string_val, 'default_string'));
    assert(iscell(opts.cell_val) && length(opts.cell_val) == 3);
    assert(isstruct(opts.struct_val) && opts.struct_val.x == 1);
end

function test_edge_cases()
    % Test edge cases: false, 0, empty string
    
    defaults.flag = true;
    defaults.count = 10;
    defaults.name = 'default';
    defaults.list = [1, 2, 3];
    
    % User explicitly sets to "falsy" values
    user_opts.flag = false;     % false (not missing!)
    user_opts.count = 0;        % zero (not missing!)
    user_opts.name = '';        % empty string (not missing!)
    user_opts.list = [];        % empty array (not missing!)
    
    opts = set_default_options(user_opts, defaults);
    
    % All user values should be preserved, even though "falsy"
    assert(opts.flag == false, 'User false should be preserved');
    assert(opts.count == 0, 'User zero should be preserved');
    assert(isempty(opts.name) && ischar(opts.name), 'User empty string preserved');
    assert(isempty(opts.list), 'User empty array should be preserved');
end

function test_non_struct_input()
    % Test that non-struct options are handled gracefully
    
    defaults.verbose = true;
    defaults.plot = false;
    
    % Non-struct input should return defaults
    opts = set_default_options(42, defaults);
    assert(opts.verbose == true);
    assert(opts.plot == false);
    
    opts = set_default_options('string', defaults);
    assert(opts.verbose == true);
    assert(opts.plot == false);
    
    opts = set_default_options({1, 2, 3}, defaults);
    assert(opts.verbose == true);
    assert(opts.plot == false);
end
