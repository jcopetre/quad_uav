function root_dir = setup_test_environment()
% SETUP_TEST_ENVIRONMENT - Add project directories to MATLAB path for testing
%
% Automatically detects project root and adds necessary directories to path.
% Assumes this file is located in <root>/test/
%
% SYNTAX:
%   root_dir = setup_test_environment()
%
% OUTPUTS:
%   root_dir - String path to project root directory
%
% DIRECTORIES ADDED:
%   - vehicle/
%   - trajectories/
%   - control/
%   - dynamics/
%
% EXAMPLE:
%   % At start of any test file:
%   setup_test_environment();
%
% See also: addpath, rmpath

% Author: Trey Copeland, jcopetre@gmail.com
% Date: 2025-10-09

%% Determine project structure
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);

%% Define directories to add
dirs_to_add = {
    'vehicle';
    'trajectories';
    'control';
    'dynamics';
};

%% Add directories to path
fprintf('Setting up test environment...\n');
fprintf('Root directory: %s\n', root_dir);

for i = 1:length(dirs_to_add)
    dir_path = fullfile(root_dir, dirs_to_add{i});
    
    if exist(dir_path, 'dir')
        addpath(dir_path);
        fprintf('  ✓ Added: %s\n', dirs_to_add{i});
    else
        warning('Directory not found (skipping): %s', dirs_to_add{i});
    end
end

fprintf('\n');

end
