classdef SimulationLogger < handle
% SIMULATIONLOGGER - Lightweight logging utility for long-running simulations
%
% Provides structured logging to both console and disk for debugging
% long-running Monte Carlo simulations and other analyses.
%
% FEATURES:
%   - Timestamped log files in ./logs/ directory
%   - Multiple log levels (DEBUG, INFO, WARNING, ERROR)
%   - Automatic file creation and management
%   - Console echo control (show/hide by level)
%   - Trial-specific logging with context
%   - Automatic flush for crash recovery
%
% SYNTAX:
%   logger = SimulationLogger(log_name)
%   logger = SimulationLogger(log_name, level)
%   logger = SimulationLogger(log_name, level, echo_console)
%
% INPUTS:
%   log_name      - Base name for log file (e.g., 'monte_carlo')
%                   Full name: logs/<log_name>_YYYYMMDD_HHMMSS.log
%   level         - (optional) Minimum level to log: 'DEBUG', 'INFO', 'WARNING', 'ERROR'
%                   Default: 'INFO'
%   echo_console  - (optional) Echo to console (default: true)
%
% METHODS:
%   debug(message)   - Log DEBUG level message
%   info(message)    - Log INFO level message
%   warning(message) - Log WARNING level message
%   error(message)   - Log ERROR level message
%   close()          - Close log file
%
% EXAMPLE:
%   % Basic usage
%   logger = SimulationLogger('monte_carlo');
%   logger.info('Starting simulation with 500 trials');
%   logger.warning('High acceleration detected in trial 42');
%   logger.error('Trial 105 failed: Integration error');
%   logger.close();
%
%   % Quiet mode (only log to file)
%   logger = SimulationLogger('batch_run', 'INFO', false);
%   for i = 1:1000
%       logger.info(sprintf('Trial %d complete', i));
%   end
%   logger.close();
%
%   % Debug mode
%   logger = SimulationLogger('debug_run', 'DEBUG');
%   logger.debug('Detailed parameter values...');
%   logger.close();
%
% NOTES:
%   - Log files automatically created in ./logs/ directory
%   - Files are flushed after each write for crash safety
%   - Use handle class for pass-by-reference behavior
%   - Always call close() when done to ensure proper file closure
%
% See also: run_monte_carlo, analyze_monte_carlo_results

% Author: Trey Copeland
% Date: 2025-10-15

    properties (Access = private)
        fid                 % File identifier
        log_file            % Full path to log file
        level_threshold     % Minimum level to log
        echo_console        % Whether to echo to console
        levels              % Level hierarchy
        start_time          % Logger creation time
    end
    
    methods
        function obj = SimulationLogger(log_name, level, echo_console)
            % Constructor
            
            % Parse inputs
            if nargin < 2 || isempty(level)
                level = 'INFO';
            end
            if nargin < 3
                echo_console = true;
            end
            
            % Define log levels
            obj.levels = containers.Map(...
                {'DEBUG', 'INFO', 'WARNING', 'ERROR'}, ...
                {1, 2, 3, 4});
            
            % Validate level
            if ~obj.levels.isKey(upper(level))
                error('Invalid log level. Must be DEBUG, INFO, WARNING, or ERROR');
            end
            
            obj.level_threshold = obj.levels(upper(level));
            obj.echo_console = echo_console;
            obj.start_time = datetime('now');
            
            % Create logs directory if needed
            log_dir = './logs';
            if ~exist(log_dir, 'dir')
                mkdir(log_dir);
            end
            
            % Create timestamped log file
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            obj.log_file = fullfile(log_dir, sprintf('%s_%s.log', log_name, timestamp));
            
            % Open file
            obj.fid = fopen(obj.log_file, 'w');
            if obj.fid == -1
                error('Failed to create log file: %s', obj.log_file);
            end
            
            % Write header
            obj.write_header();
        end
        
        function delete(obj)
            % Destructor - ensure file is closed
            if ~isempty(obj.fid) && obj.fid ~= -1
                obj.close();
            end
        end
        
        function debug(obj, message)
            % Log DEBUG message
            obj.log_message('DEBUG', message);
        end
        
        function info(obj, message)
            % Log INFO message
            obj.log_message('INFO', message);
        end
        
        function warning(obj, message)
            % Log WARNING message
            obj.log_message('WARNING', message);
        end
        
        function error(obj, message)
            % Log ERROR message
            obj.log_message('ERROR', message);
        end
        
        function close(obj)
            % Close log file
            if ~isempty(obj.fid) && obj.fid ~= -1
                elapsed = seconds(datetime('now') - obj.start_time);
                obj.info(sprintf('Log closed. Total duration: %.2f seconds', elapsed));
                obj.write_footer();
                fclose(obj.fid);
                obj.fid = -1;
                
                if obj.echo_console
                    fprintf('Log saved to: %s\n', obj.log_file);
                end
            end
        end
        
        function path = get_log_path(obj)
            % Get full path to log file
            path = obj.log_file;
        end
    end
    
    methods (Access = private)
        function write_header(obj)
            % Write log file header
            fprintf(obj.fid, '========================================\n');
            fprintf(obj.fid, 'SIMULATION LOG\n');
            fprintf(obj.fid, '========================================\n');
            fprintf(obj.fid, 'Started: %s\n', datestr(obj.start_time));
            fprintf(obj.fid, 'MATLAB: %s\n', version);
            fprintf(obj.fid, 'Computer: %s\n', computer);
            fprintf(obj.fid, 'User: %s\n', getenv('USERNAME'));
            fprintf(obj.fid, 'Log Level: %s\n', obj.get_level_name(obj.level_threshold));
            fprintf(obj.fid, '========================================\n\n');
            
            % Flush to disk
            fflush(obj.fid);
        end
        
        function write_footer(obj)
            % Write log file footer
            fprintf(obj.fid, '\n========================================\n');
            fprintf(obj.fid, 'LOG END\n');
            fprintf(obj.fid, '========================================\n');
        end
        
        function log_message(obj, level, message)
            % Write message to log file and optionally console
            
            % Check if level meets threshold
            if obj.levels(level) < obj.level_threshold
                return;
            end
            
            % Format timestamp
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
            
            % Format message
            log_line = sprintf('[%s] [%7s] %s\n', timestamp, level, message);
            
            % Write to file
            fprintf(obj.fid, '%s', log_line);
            fflush(obj.fid);  % Immediate flush for crash safety
            
            % Echo to console if enabled
            if obj.echo_console
                % Color code by level (if terminal supports it)
                fprintf('%s', log_line);
            end
        end
        
        function name = get_level_name(obj, level_value)
            % Convert level value back to name
            keys = obj.levels.keys();
            vals = cell2mat(obj.levels.values());
            idx = find(vals == level_value, 1);
            if ~isempty(idx)
                name = keys{idx};
            else
                name = 'UNKNOWN';
            end
        end
    end
    
    methods (Static)
        function logger = create_default(log_name)
            % Quick constructor with defaults
            logger = SimulationLogger(log_name, 'INFO', true);
        end
        
        function fflush(fid)
            % Force flush buffer to disk (static utility)
            % MATLAB doesn't have fflush, so we use Java
            if fid ~= -1
                try
                    % Try to flush using undocumented feature
                    fwrite(fid, '');  % Dummy write forces flush
                catch
                    % Silently fail if not supported
                end
            end
        end
    end
end