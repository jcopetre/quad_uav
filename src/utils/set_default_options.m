function options = set_default_options(options, defaults)
% SET_DEFAULT_OPTIONS - Apply default values to an options struct
%
% SYNTAX:
%   options = set_default_options(options, defaults)
%
% DESCRIPTION:
%   Fills in missing fields in an options struct with default values.
%   This eliminates repetitive isfield checks throughout the codebase.
%
% INPUTS:
%   options  - Struct with user-specified options (may be empty or incomplete)
%   defaults - Struct containing default field values
%
% OUTPUTS:
%   options  - Complete options struct with all defaults applied
%
% BEHAVIOR:
%   - If options is empty or not a struct, returns defaults
%   - For each field in defaults:
%       * If field exists in options: keep user value
%       * If field missing in options: use default value
%   - Does NOT remove extra fields from options (only adds missing ones)
%
% EXAMPLE:
%   % Define defaults
%   defaults.verbose = true;
%   defaults.plot = false;
%   defaults.save_results = true;
%   defaults.dt = 0.01;
%   
%   % User provides partial options
%   user_opts.verbose = false;
%   user_opts.dt = 0.02;
%   
%   % Merge with defaults
%   opts = set_default_options(user_opts, defaults);
%   % Result: opts.verbose = false (user), opts.plot = false (default),
%   %         opts.save_results = true (default), opts.dt = 0.02 (user)
%
% NOTES:
%   - Designed for flat structs (no nested struct handling)
%   - Preserves user-specified values, including false/0/empty
%   - Useful for standardizing options handling across functions
%
% See also: isfield, fieldnames

% Author: Trey Copeland
% Date: 2025-10-31

    % Input validation
    assert(isstruct(defaults), 'defaults must be a struct');
    
    % If options is empty/missing/not a struct, use all defaults
    if isempty(options) || ~isstruct(options)
        options = defaults;
        return;
    end
    
    % Apply defaults for any missing fields
    default_fields = fieldnames(defaults);
    for i = 1:length(default_fields)
        field = default_fields{i};
        if ~isfield(options, field)
            options.(field) = defaults.(field);
        end
    end
    
end
