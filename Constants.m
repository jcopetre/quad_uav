classdef Constants
    properties (Constant = true)
        G = 9.81;
    
        TOL = 1e-10;              % Standard numerical tolerance
        TOL_LOOSE = 1e-6;         % Loose tolerance for numerical integration
        TOL_STRICT = 1e-12;       % Strict tolerance for analytical results

        % Trajectory generation markers
        AUTO_YAW = NaN;           % Marker for auto-calculate yaw from velocity direction
                                  % Usage: Set with Constants.AUTO_YAW, check with isnan()
                                  % Note: Cannot use == for checking due to NaN != NaN
    end
end
