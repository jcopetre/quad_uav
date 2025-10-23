clear, clc;
init_project();

TRAJECTORY = 'simple_square.wpt';

%% Get nominal parameters to compute absolute uncertainties
params_nominal = quadrotor_linear_6dof();

%% Operational Uncertainties
perturb_config = struct();
perturb_config.params = {
    % Parameter, Type, Param1, Param2

    % Mass: ±10% (payload changes, battery depletion)
    'm', 'normal', params_nominal.m, params_nominal.m * 0.05;
    
    % Inertia: ±15% (payload distribution, manufacturing)
    'Ixx', 'normal', params_nominal.Ixx, params_nominal.Ixx * 0.075;
    'Iyy', 'normal', params_nominal.Iyy, params_nominal.Iyy * 0.075;
    'Izz', 'normal', params_nominal.Izz, params_nominal.Izz * 0.075;
    
    % TODO implement these in quadrotor_linear_6dof etc
    % % Thrust coefficient: ±12% (propeller wear, voltage sag, temperature)
    % 'k_thrust', 'normal', params_nominal.k_thrust, params_nominal.k_thrust * 0.06;
    % 
    % % Moment coefficient: ±12% (similar to thrust)
    % 'k_torque', 'normal', params_nominal.k_torque, params_nominal.k_torque * 0.06;
    
    % Arm length: ±1% (manufacturing tolerance)
    'L', 'normal', params_nominal.L, params_nominal.L * 0.005;
};


%% Monte Carlo Options
mc_options = struct();
mc_options.n_trials = 500;  % Good statistical power for pilot study
mc_options.seed = 42;       % Reproducibility
mc_options.parallel = true; % Set true if you have parallel computing available
mc_options.verbose = true;  % Monitor progress

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
mc_options.save_file = sprintf('./results/mc_pilot_study_%s.mat', timestamp);

%% Run the study
results = run_monte_carlo(TRAJECTORY, perturb_config, mc_options);

%% Analyze results
analysis = analyze_monte_carlo_results(results);