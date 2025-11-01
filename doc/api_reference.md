# API Reference

Complete function documentation for the Quadrotor LQR Control Framework.

## Table of Contents

- [Simulation Functions](#simulation-functions)
  - [simulate_trajectory](#simulate_trajectory)
- [Monte Carlo Functions](#monte-carlo-functions)
  - [simulate_monte_carlo](#simulate_monte_carlo)
  - [run_monte_carlo](#run_monte_carlo)
  - [analyze_monte_carlo](#analyze_monte_carlo)
- [Trajectory Functions](#trajectory-functions)
  - [load_waypoints](#load_waypoints)
  - [generate_trajectory_auto](#generate_trajectory_auto)
  - [generate_trajectory_interp](#generate_trajectory_interp)
  - [generate_trajectory_minsnap](#generate_trajectory_minsnap)
- [Control Functions](#control-functions)
  - [compute_lqr_control](#compute_lqr_control)
  - [get_reference_state](#get_reference_state)
- [Data Management](#data-management)
  - [DataManager.save_results](#datamanagersave_results)
  - [DataManager.load_results](#datamanagerload_results)
  - [DataManager.save_monte_carlo](#datamanagersave_monte_carlo)
- [Utility Functions](#utility-functions)
  - [set_default_options](#set_default_options)
- [Vehicle Models](#vehicle-models)
  - [quadrotor_linear_6dof](#quadrotor_linear_6dof)
- [Output Functions](#output-functions)
  - [generate_paper_outputs](#generate_paper_outputs)
- [Performance Metrics](#performance-metrics)
  - [compute_performance_metrics](#compute_performance_metrics)
- [Constants](#constants)
- [Common Patterns](#common-patterns)
---

## Simulation Functions

### simulate_trajectory

Main simulation orchestrator for single trajectory tracking.

**Location**: `src/simulation/simulate_trajectory.m`

**Syntax**:
```matlab
simulate_trajectory(trajectory_file)
simulate_trajectory(trajectory_file, Q, R)
simulate_trajectory(trajectory_file, Q, R, x0)
simulate_trajectory(trajectory_file, Q, R, x0, options)
results = simulate_trajectory(...)
```

**Inputs**:
- `trajectory_file` (string): Filename in `./trajectories/` (e.g., `'basic_maneuver.wpt'`)
- `Q` (12×12 matrix, optional): LQR state weights
- `R` (4×4 matrix, optional): LQR control weights
- `x0` (12×1 vector, optional): Initial state (position overridden by trajectory start)
- `options` (struct, optional):
  - `.verbose` (bool): Print progress (default: `true`)
  - `.plot` (bool): Show plots (default: `true` if no output)
  - `.save_results` (bool): Save to `.mat` file (default: `true`)
  - `.dt` (double): Integration timestep (default: `0.01` s)
  - `.params` (struct): Pre-designed parameters (for Monte Carlo)

**Outputs**:
- `results` (struct): Simulation results
  - `.t` [N×1]: Time vector
  - `.x` [N×12]: State history
  - `.u_log` [N×4]: Control history
  - `.trajectory` (struct): Reference trajectory
  - `.params` (struct): Vehicle parameters
  - `.metrics` (struct): Performance metrics
  - `.config` (struct): Configuration used
  - `.timestamp` (string): When run

**Examples**:
```matlab
% Basic usage
simulate_trajectory('basic_maneuver.wpt');

% Custom tuning
Q = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R = diag([0.01 0.5 0.5 0.5]);
simulate_trajectory('basic_maneuver.wpt', Q, R);

% Capture results
opts.plot = false;
results = simulate_trajectory('basic_maneuver.wpt', [], [], [], opts);
```

---

## Monte Carlo Functions

### simulate_monte_carlo

High-level orchestrator for Monte Carlo robustness studies.

**Location**: `src/monte_carlo/simulate_monte_carlo.m`

**Syntax**:
```matlab
results_dir = simulate_monte_carlo(trajectory_file, run_label)
results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config)
results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options)
```

**Inputs**:
- `trajectory_file` (string): Waypoint file
- `run_label` (string): Identifier for this study (e.g., `'paper_final'`)
- `perturb_config` (struct, optional): Parameter perturbation configuration
  - `.params` (cell array): `{param_name, dist_type, mean/min, std/max}`
  - If empty, uses default operational uncertainties
- `mc_options` (struct, optional):
  - `.N_trials` (int): Number of trials (default: `500`)
  - `.seed` (int): RNG seed (default: `42`)
  - `.parallel` (bool): Use `parfor` (default: `true`)
  - `.verbose` (bool): Print progress (default: `true`)

**Outputs**:
- `results_dir` (string): Path to results directory

**Directory Structure Created**:
```
./results/<run_label>_<timestamp>/
├── nominal.mat              % Nominal simulation
├── monte_carlo.mat          % MC results
└── metrics_<timestamp>.txt  % Summary report
```

**Examples**:
```matlab
% Use defaults
results_dir = simulate_monte_carlo('simple_square.wpt', 'test1');

% Custom parameters
params = quadrotor_linear_6dof();
perturb_config.params = {
    'm',   'normal', params.m, params.m * 0.05;
    'Ixx', 'normal', params.Ixx, params.Ixx * 0.075;
};

mc_options.N_trials = 1000;
mc_options.parallel = true;

results_dir = simulate_monte_carlo('square.wpt', 'robust_test', ...
                                   perturb_config, mc_options);
```

### run_monte_carlo

Core Monte Carlo engine (parallel trial execution).

**Location**: `src/monte_carlo/run_monte_carlo.m`

**Syntax**:
```matlab
mc_results = run_monte_carlo(trajectory_file, perturb_config, mc_options)
```

**Inputs**:
- `trajectory_file` (string): Waypoint file
- `perturb_config` (struct): Parameter configuration
- `mc_options` (struct): MC options (see `simulate_monte_carlo`)

**Outputs**:
- `mc_results` (struct): Complete MC results
  - `.config`: Configuration used
  - `.trials` [1×N]: Array of trial results
  - `.statistics`: Summary statistics
  - `.nominal`: Nominal simulation results
  - `.param_data`: Parameter samples
  - `.elapsed_time`: Total computation time

**Note**: Usually called via `simulate_monte_carlo()`, not directly.

### analyze_monte_carlo

Statistical analysis and visualization of MC results.

**Location**: `src/monte_carlo/analyze_monte_carlo.m`

**Syntax**:
```matlab
analyze_monte_carlo(mc_results)
analyze_monte_carlo(mc_results, analysis_options)
analysis = analyze_monte_carlo(...)
```

**Inputs**:
- `mc_results` (struct): Results from `run_monte_carlo()`
- `analysis_options` (struct, optional):
  - `.plot` (bool): Generate plots (default: `true`)
  - `.save_plots` (bool): Save figures (default: `false`)
  - `.correlation` (bool): Compute correlations (default: `true`)
  - `.verbose` (bool): Print report (default: `true`)

**Outputs**:
- `analysis` (struct): Analysis results
  - `.statistics`: Summary stats
  - `.correlations`: Parameter-metric correlations
  - `.sensitivity`: Sensitivity indices
  - `.failure_analysis`: Failed trial details

---

## Trajectory Functions

### load_waypoints

Parse JSON waypoint files.

**Location**: `src/trajectory/load_waypoints.m`

**Syntax**:
```matlab
wpt = load_waypoints(filename)
```

**Inputs**:
- `filename` (string): Path to `.wpt` file

**Outputs**:
- `wpt` (struct): Waypoint structure
  - `.time` [N×1]: Waypoint times (s)
  - `.position` [N×3]: Positions (m)
  - `.yaw` [N×1]: Yaw angles (rad), `NaN` for auto
  - `.metadata` (struct): File metadata

**Yaw specification**:
- Values in **radians** (0 = North in NED frame)
- Explicit value (e.g., `0`, `1.5708`, `3.1416`): Fixed heading
- `null` (JSON) / `NaN` (MATLAB): Auto-calculate from velocity direction

**Example**:
```matlab
wpt = load_waypoints('basic_maneuver.wpt');
fprintf('Trajectory: %s\n', wpt.metadata.name);
fprintf('Duration: %.1f s\n', wpt.time(end));
```

### generate_trajectory_auto

Intelligent automatic method selection (RECOMMENDED).

**Location**: `src/trajectory/generate_trajectory_auto.m`

**Syntax**:
```matlab
trajectory = generate_trajectory_auto(wpt, params)
trajectory = generate_trajectory_auto(wpt, params, dt)
```

**Inputs**:
- `wpt` (struct): Waypoint structure from `load_waypoints()`
- `params` (struct): Vehicle parameters from `quadrotor_linear_6dof()`
- `dt` (double, optional): Timestep (default: `0.01` s)

**Outputs**:
- `trajectory` (struct): Trajectory structure
  - `.time` [N×1]: Time vector
  - `.position` [N×3]: Position trajectory
  - `.velocity` [N×3]: Velocity trajectory
  - `.acceleration` [N×3]: Acceleration trajectory
  - `.attitude` [N×3]: Feedforward attitude (φ, θ, ψ)
  - `.omega` [N×3]: Angular rates
  - `.method` (string): `'interpolation'` or `'minsnap'`
  - `.method_reason` (string): Why this method was chosen
  - `.selection_criteria` (struct): Decision metadata

**Selection Logic**:
- Analyzes all segment durations
- If ANY segment < 3s → MAKIMA interpolation
- If ALL segments ≥ 3s → Minimum snap optimization

**Example**:
```matlab
wpt = load_waypoints('basic_maneuver.wpt');
params = quadrotor_linear_6dof();
trajectory = generate_trajectory_auto(wpt, params);

fprintf('Method: %s\n', trajectory.method);
fprintf('Reason: %s\n', trajectory.method_reason);
```

### generate_trajectory_interp

MAKIMA interpolation-based trajectory generation.

**Location**: `src/trajectory/generate_trajectory_interp.m`

**Syntax**:
```matlab
trajectory = generate_trajectory_interp(wpt, params)
trajectory = generate_trajectory_interp(wpt, params, dt)
```

**Inputs**: Same as `generate_trajectory_auto`

**Outputs**: Same as `generate_trajectory_auto`, but `.method = 'interpolation'`

**Best for**:
- Short segments (< 3s)
- Quick maneuvers
- When waypoints define precise path

### generate_trajectory_minsnap

Minimum snap optimization-based trajectory.

**Location**: `src/trajectory/generate_trajectory_minsnap.m`

**Syntax**:
```matlab
trajectory = generate_trajectory_minsnap(wpt, params)
trajectory = generate_trajectory_minsnap(wpt, params, dt)
```

**Inputs**: Same as `generate_trajectory_auto`

**Outputs**: Same as `generate_trajectory_auto`, but `.method = 'minsnap'`

**Best for**:
- Long, smooth segments (≥ 3s)
- Photogrammetry, inspection
- Optimal smoothness required

**Requires**: Optimization Toolbox (`quadprog`)

**Reference**: Mellinger & Kumar (2011)

---

## Control Functions

### compute_lqr_control

Compute LQR control law with saturation.

**Location**: `control/compute_lqr_control.m`

**Syntax**:
```matlab
u = compute_lqr_control(x, x_ref, params)
```

**Inputs**:
- `x` [12×1]: Current state
- `x_ref` [12×1]: Reference state
- `params` (struct): Vehicle parameters (includes `.K`, `.u_hover`)

**Outputs**:
- `u` [4×1]: Control inputs [F, τ_φ, τ_θ, τ_ψ]'

**Control Law**:
```matlab
e = x - x_ref;
u = u_hover - K * e;
u = saturate(u, u_min, u_max);
```

### get_reference_state

Look up reference state from trajectory at time t.

**Location**: `control/get_reference_state.m`

**Syntax**:
```matlab
x_ref = get_reference_state(t, trajectory)
```

**Inputs**:
- `t` (double): Current time (s)
- `trajectory` (struct): Trajectory structure

**Outputs**:
- `x_ref` [12×1]: Reference state at time `t`

**Interpolation**: Linear interpolation between trajectory points

---

## Data Management

### DataManager.save_results

Save simulation results with validation.

**Location**: `src/data/DataManager.m`

**Syntax**:
```matlab
filepath = DataManager.save_results(results, label, output_dir)
filepath = DataManager.save_results(results, label, output_dir, options)
```

**Inputs**:
- `results` (struct): Simulation results
- `label` (string): Filename (without extension)
- `output_dir` (string): Directory path
- `options` (struct, optional):
  - `.timestamp` (bool): Add timestamp to filename (default: `true`)
  - `.validate` (bool): Validate structure (default: `true`)
  - `.verbose` (bool): Print messages (default: `false`)

**Outputs**:
- `filepath` (string): Full path to saved file

**Example**:
```matlab
results = simulate_trajectory('test.wpt', [], [], [], opts);
filepath = DataManager.save_results(results, 'my_sim', './results');
```

### DataManager.load_results

Load simulation results with validation.

**Syntax**:
```matlab
results = DataManager.load_results(filepath)
results = DataManager.load_results(filepath, options)
```

**Inputs**:
- `filepath` (string): Path to `.mat` file
- `options` (struct, optional):
  - `.validate` (bool): Validate structure (default: `true`)
  - `.verbose` (bool): Print messages (default: `false`)

**Outputs**:
- `results` (struct): Loaded simulation results

### DataManager.save_monte_carlo

Save Monte Carlo results.

**Syntax**:
```matlab
filepath = DataManager.save_monte_carlo(mc_results, output_dir)
filepath = DataManager.save_monte_carlo(mc_results, output_dir, options)
```

**Inputs**: Similar to `save_results`

**Example**:
```matlab
DataManager.save_monte_carlo(mc_results, './results/study1');
```

---

## Utility Functions

### set_default_options

Apply default values to options struct.

**Location**: `src/utils/set_default_options.m`

**Syntax**:
```matlab
options = set_default_options(options, defaults)
```

**Inputs**:
- `options` (struct): User-provided options (may be partial/empty)
- `defaults` (struct): Default values

**Outputs**:
- `options` (struct): Complete options with defaults applied

**Behavior**:
- If field exists in `options`: Keep user value
- If field missing: Use default value
- Extra fields in `options`: Preserved

**Example**:
```matlab
defaults.verbose = true;
defaults.plot = false;
defaults.dt = 0.01;

user_opts.verbose = false;  % User override
% plot and dt will use defaults

opts = set_default_options(user_opts, defaults);
% opts.verbose = false (user)
% opts.plot = false (default)
% opts.dt = 0.01 (default)
```

---

## Vehicle Models

### quadrotor_linear_6dof

Vehicle model, parameters, and LQR controller design.

**Location**: `vehicle/quadrotor_linear_6dof.m`

**Syntax**:
```matlab
params = quadrotor_linear_6dof()
params = quadrotor_linear_6dof(Q, R)
params = quadrotor_linear_6dof(Q, R, verbose)
```

**Inputs**:
- `Q` (12×12 matrix, optional): State weights for LQR
- `R` (4×4 matrix, optional): Control weights for LQR
- `verbose` (bool, optional): Print design info (default: `true`)

**Outputs**:
- `params` (struct): Complete vehicle parameters
  - **Physical**:
    - `.m` (double): Mass (kg)
    - `.g` (double): Gravity (m/s²)
    - `.L` (double): Arm length (m)
    - `.Ixx`, `.Iyy`, `.Izz` (double): Inertias (kg·m²)
  - **Actuation**:
    - `.u_min`, `.u_max` [4×1]: Control limits
  - **Controller**:
    - `.K` [4×12]: LQR gain matrix
    - `.u_hover` [4×1]: Hover control [m*g; 0; 0; 0]
    - `.poles` [12×1]: Closed-loop eigenvalues
  - **Weights**:
    - `.Q` [12×12]: State weights used
    - `.R` [4×4]: Control weights used

**Default Parameters**:
- Mass: 0.5 kg
- Arm length: 0.25 m
- Inertias: Ixx = Iyy = 0.01 kg·m², Izz = 0.02 kg·m²

**Default Weights**:
```matlab
Q = diag([100 100 100 10 10 1 20 20 20 1 1 0.1]);
R = diag([1 2 2 2]);
```

**Example**:
```matlab
% Default parameters
params = quadrotor_linear_6dof();

% Custom tuning (aggressive)
Q_tight = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R_smooth = diag([2 4 4 4]);
params = quadrotor_linear_6dof(Q_tight, R_smooth, false);
```

---

## Output Functions

### generate_paper_outputs

Generate publication-quality figures from saved MC data.

**Location**: `src/output/generate_paper_outputs.m`

**Syntax**:
```matlab
generate_paper_outputs(results_dir)
generate_paper_outputs(results_dir, options)
```

**Inputs**:
- `results_dir` (string): Path to results directory
- `options` (struct, optional):
  - `.close_figures` (bool): Close after saving (default: `true`)
  - `.verbose` (bool): Print progress (default: `true`)

**Output Files** (in `<results_dir>/figures/`):
1. `tracking_3d.png`
2. `tracking_timeseries.png`
3. `control_inputs.png`
4. `attitude_dynamics.png`
5. `distributions.png`
6. `boxplots.png`
7. `correlation.png`
8. `paper_metrics.txt` (LaTeX snippets)

**Example**:
```matlab
results_dir = simulate_monte_carlo('square.wpt', 'study1');
generate_paper_outputs(results_dir);

% Keep figures open for inspection
opts.close_figures = false;
generate_paper_outputs(results_dir, opts);
```

---

## Performance Metrics

### compute_performance_metrics

Compute comprehensive performance metrics.

**Location**: `src/utils/compute_performance_metrics.m` (or similar)

**Syntax**:
```matlab
metrics = compute_performance_metrics(t, x, trajectory, params, u_log)
```

**Inputs**:
- `t` [N×1]: Time vector
- `x` [N×12]: State history
- `trajectory` (struct): Reference trajectory
- `params` (struct): Vehicle parameters
- `u_log` [N×4]: Control history

**Outputs**:
- `metrics` (struct):
  - `.tracking`:
    - `.rmse_position` (double): Position RMSE (m)
    - `.rmse_attitude` (double): Attitude RMSE (rad)
    - `.max_position_error` (double): Max error (m)
  - `.control`:
    - `.mean_thrust` (double): Average thrust (N)
    - `.total_effort` (double): Integrated effort
    - `.thrust_saturation_pct` (double): % time saturated
  - `.linearity`:
    - `.max_actual_attitude` (double): Max attitude (rad)
    - `.violated` (bool): Exceeds ~15°?
  - `.success`:
    - `.completed` (bool): Simulation finished
    - `.tracking_ok` (bool): RMSE < threshold
    - `.overall` (bool): All criteria met

---

## Constants

### Constants.m

Shared constants across framework.

**Location**: `Constants.m` (project root)

**Contents**:
```matlab
classdef Constants
    properties (Constant)
        TOL = 1e-6;                    % Numerical tolerance for tests
        AUTO_YAW = NaN;                % Magic value for auto yaw
        GRAVITY = 9.81;                % m/s² (Earth)
        % Add others as needed
    end
end
```

**Usage**:
```matlab
if abs(value) < Constants.TOL
    % Approximately zero
end

if isnan(yaw) || yaw == Constants.AUTO_YAW
    % Auto-calculate yaw
end
```

---

## Common Patterns

### Pattern 1: Basic Simulation

```matlab
init_project();
simulate_trajectory('basic_maneuver.wpt');
```

### Pattern 2: Batch Processing

```matlab
trajectories = {'hover.wpt', 'square.wpt', 'slalom.wpt'};
opts.plot = false;

for i = 1:length(trajectories)
    results(i) = simulate_trajectory(trajectories{i}, [], [], [], opts);
end
```

### Pattern 3: Parameter Sweep

```matlab
Q_values = logspace(1, 3, 10);  % 10 to 1000
opts.plot = false;

for i = 1:length(Q_values)
    Q = Q_values(i) * eye(12);
    results(i) = simulate_trajectory('test.wpt', Q, [], [], opts);
    rmse(i) = results(i).metrics.tracking.rmse_position;
end

plot(Q_values, rmse);
```

### Pattern 4: Monte Carlo Workflow

```matlab
% Run study
results_dir = simulate_monte_carlo('trajectory.wpt', 'study1', [], mc_opts);

% Generate figures
generate_paper_outputs(results_dir);

% Load and analyze
mc_data = load(fullfile(results_dir, 'monte_carlo.mat'));
analyze_monte_carlo(mc_data.mc_results);
```

---

## See Also

- [Quick Start Guide](./quick_start.md) - Getting started
- [Architecture Overview](./architecture.md) - System design
- [Monte Carlo Guide](./monte_carlo_guide.md) - Robustness analysis
- [Installation Guide](./installation.md) - Setup and requirements
