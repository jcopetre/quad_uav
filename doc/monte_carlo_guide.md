# Monte Carlo Robustness Analysis Guide

Complete guide to running parameter uncertainty studies and analyzing robustness.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Workflow](#workflow)
- [Parameter Configuration](#parameter-configuration)
- [Running Studies](#running-studies)
- [Analyzing Results](#analyzing-results)
- [Figure Generation](#figure-generation)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## Overview

Monte Carlo analysis tests controller robustness by running many simulations with randomly perturbed vehicle parameters. This reveals:

- **Performance distribution**: How tracking varies across parameter space
- **Sensitivity**: Which parameters most affect performance
- **Failure modes**: What combinations cause instability
- **Statistical confidence**: Mean, std dev, percentiles

**Typical use case**: Validate that your controller works across manufacturing tolerances, wear, or payload variations.

## Quick Start

### 1-Minute Demo (10 trials)

```matlab
cd examples
robustness_study  % Uses default configuration
```

This runs a minimal study (10 trials) for testing. For real research, use 500+ trials.

### 5-Minute Study (Production)

Edit `examples/robustness_study.m`:

```matlab
N_TRIALS = 500;         % Increase for real study
PARALLEL = true;        % Enable parallel processing
```

Then run:

```matlab
robustness_study
```

**Time estimates**:
- 500 trials, parallel (8 cores): ~20 minutes
- 500 trials, serial: ~2 hours
- 2000 trials, parallel: ~1.5 hours

## Workflow

The Monte Carlo framework uses a two-phase approach:

### Phase 1: Data Generation (Slow)

Run **once** - generates all simulation data:

```matlab
results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options);
```

Output: `./results/<run_label>_<timestamp>/`
- `nominal.mat` - Baseline (unperturbed) simulation
- `monte_carlo.mat` - All trial results
- `mc_run_log.txt` - Run configuration and execution summary

**Important**: This is expensive (minutes to hours). Only run once per study.

### Phase 2: Figure Generation (Fast)

Run **many times** - creates publication figures:

```matlab
generate_paper_outputs(results_dir);
```

Output: `./results/<run_label>_<timestamp>/figures/`
- 7 publication-quality PNG figures
- `analysis_report.txt` - Comprehensive analysis with correlations
- `latex_snippets.txt` - LaTeX snippets for paper

**Benefit**: Iterate on figure appearance without regenerating data.

### Phase 3: Regeneration (Instant)

**Key capability**: Regenerate all outputs from .mat files:
```matlab
% Delete outputs to test
delete(fullfile(results_dir, 'analysis_report.txt'));
delete(fullfile(results_dir, 'latex_snippets.txt'));
rmdir(fullfile(results_dir, 'figures'), 's');

% Regenerate everything instantly
generate_paper_outputs(results_dir);
```

**Benefits**:
- Iterate on report formatting without re-simulating
- Update figures after tweaking visualization code
- Recover from accidental deletions
- True separation: expensive simulation vs. cheap post-processing

## Parameter Configuration

### Default Configuration

If you don't specify parameters, sensible defaults are used:

```matlab
% Use defaults (±5% mass, ±7.5% inertia, etc.)
perturb_config = [];
results_dir = simulate_monte_carlo('my_trajectory.wpt', 'study1', perturb_config);
```

Default uncertainties represent typical manufacturing tolerances:
- Mass: ±5%
- Inertias: ±7.5%
- Arm length: ±0.5%

### Custom Configuration

Define your own parameter distributions:

```matlab
% Get nominal parameters
params_nominal = quadrotor_linear_6dof();

% Define perturbations
perturb_config.params = {
    % Name,  Distribution, Mean,                  Spread
    'm',     'normal',     params_nominal.m,      params_nominal.m * 0.10;    % ±10% mass
    'Ixx',   'normal',     params_nominal.Ixx,    params_nominal.Ixx * 0.15;  % ±15% roll inertia
    'Iyy',   'normal',     params_nominal.Iyy,    params_nominal.Iyy * 0.15;  % ±15% pitch inertia
    'Izz',   'normal',     params_nominal.Izz,    params_nominal.Izz * 0.15;  % ±15% yaw inertia
    'L',     'uniform',    0.22,                  0.27;                        % Arm length [0.22, 0.27]m
};
```

**Distribution types**:
- `'normal'`: Gaussian (mean, std dev)
- `'uniform'`: Uniform over [min, max]

**Parameters you can perturb**:
- `m`: Mass (kg)
- `Ixx`, `Iyy`, `Izz`: Moments of inertia (kg·m²)
- `L`: Arm length (m)
- `g`: Gravity (m/s²) - for Mars/Moon simulations!

### Physical Constraints

The framework automatically enforces:
- Mass > 0
- Inertias > 0
- Arm length > 0
- Other physical validity checks

Invalid samples are rejected and resampled.

## Running Studies

### Basic Study

```matlab
% Configuration
trajectory_file = 'simple_square.wpt';
run_label = 'square_robustness';

perturb_config = [];  % Use defaults
mc_options.N_trials = 500;
mc_options.parallel = true;
mc_options.verbose = true;

% Run
results_dir = simulate_monte_carlo(trajectory_file, run_label, perturb_config, mc_options);
```

### Monte Carlo Options

```matlab
mc_options = struct();
mc_options.N_trials = 500;          % Number of trials
mc_options.seed = 42;               % RNG seed (for reproducibility)
mc_options.parallel = true;         % Use parfor (requires Parallel Toolbox)
mc_options.verbose = true;          % Print progress
```

**Key options**:

- **N_trials**: How many simulations to run
  - Development: 50-100
  - Pilot study: 500
  - Final paper: 1000-2000

- **seed**: Random seed for reproducibility
  - Same seed → identical results
  - Different seed → different samples (but same statistics)

- **parallel**: Use parallel processing
  - `true`: Uses all CPU cores (4-8x speedup)
  - `false`: Serial execution (easier debugging)

### Parallel Processing Setup

If you have Parallel Computing Toolbox:

```matlab
% Start parallel pool (do once)
parpool('local', 8);  % 8 workers

% Run study
mc_options.parallel = true;
simulate_monte_carlo(..., mc_options);

% Clean up (optional)
delete(gcp('nocreate'));
```

**Tips**:
- Pool startup takes ~10 seconds
- Reuse pool for multiple studies
- Watch CPU/memory usage (Task Manager / Activity Monitor)

## Analyzing Results

### Metrics Summary File

After running `simulate_monte_carlo`, check:
`````
./results/<run_label>_<timestamp>/mc_run_log.txt
`````

Contains run configuration:
- Trajectory settings
- Vehicle parameters
- Monte Carlo configuration (trials, seed, parallel)
- Parameter perturbations
- Execution summary (success rate, timing)

For comprehensive analysis with correlations, run:
`````matlab
generate_paper_outputs(results_dir);
`````

This creates:
`````
./results/<run_label>_<timestamp>/analysis_report.txt
`````

Contains:
- Configuration recap
- Nominal and Monte Carlo performance
- **Parameter correlations** (which params affect performance most)
- **Sensitivity analysis** (normalized impact)
- **Worst-case performance** (with parameter configurations)
- Failure analysis (if any trials failed)

### Example Metrics

```
================================================================
OVERALL STATISTICS
================================================================
Total trials:     500
Successful:       498 (99.6%)
Failed:           2 (0.4%)
Elapsed time:     1247.3 seconds
Time per trial:   2.495 seconds

================================================================
NOMINAL PERFORMANCE
================================================================
Position RMSE:    0.0234 m
Attitude RMSE:    0.0215 rad (1.23 deg)

================================================================
MONTE CARLO PERFORMANCE
================================================================
Position RMSE (mean ± std):     0.0567 ± 0.0123 m
Position RMSE percentiles [5 25 50 75 95]:
    [0.0389 0.0478 0.0545 0.0638 0.0812] m

Attitude RMSE (mean ± std):     0.0578 ± 0.0089 rad (3.31 ± 0.51 deg)
Attitude RMSE percentiles:
    [0.0445 0.0516 0.0567 0.0631 0.0751] rad

================================================================
CORRELATIONS WITH PERFORMANCE
================================================================
Parameter     | Position RMSE | Attitude RMSE
-------------------------------------------------
m             |    +0.087     |    -0.042
Ixx           |    -0.123     |    +0.231
Iyy           |    -0.152     |    +0.512      ← Strongest
Izz           |    +0.045     |    -0.167
L             |    +0.201     |    +0.089
```

**Interpretation**:
- **Correlation values**: Range from -1 to +1
  - Positive: Parameter increase → metric increase
  - Negative: Parameter increase → metric decrease
  - Near zero: Little relationship
- **Strong correlations** (|r| > 0.3): These parameters matter most
  - Example: Iyy correlation with attitude RMSE = +0.512 (strong!)
  - Means: Higher pitch inertia → worse attitude tracking

### Programmatic Access

Load results in MATLAB:

```matlab
% Load data
results_dir = './results/square_robustness_20251101_140322';
nominal = load(fullfile(results_dir, 'nominal.mat'));
mc_data = load(fullfile(results_dir, 'monte_carlo.mat'));
mc_results = mc_data.mc_results;

% Access statistics
n_success = mc_results.statistics.n_success;
success_rate = mc_results.statistics.success_rate;

pos_rmse_mean = mc_results.statistics.metrics.rmse_position_mean;
pos_rmse_std = mc_results.statistics.metrics.rmse_position_std;

% Access individual trials
for i = 1:length(mc_results.trials)
    if mc_results.trials(i).success
        trial_rmse = mc_results.trials(i).metrics.tracking.rmse_position;
        % Process...
    end
end

% Parameter samples
param_names = mc_results.param_data.param_names;
param_samples = mc_results.param_data.param_samples;  % [N_trials × N_params]
```

## Figure Generation

### Generate All Figures

```matlab
results_dir = './results/square_robustness_20251101_140322';
generate_paper_outputs(results_dir);
```

Creates 7 figures in `./results/<run_label>_<timestamp>/figures/`:

1. **tracking_3d.png**: 3D trajectory with waypoints
2. **tracking_timeseries.png**: Position and attitude time series
3. **control_inputs.png**: Thrust and torques
4. **attitude_dynamics.png**: Attitude evolution with rate of change
5. **distributions.png**: Performance histograms across trials
6. **boxplots.png**: Statistical distributions
7. **correlation.png**: Parameter-performance correlations

Plus:
- **analysis_report.txt** - Comprehensive metrics with correlations
- **latex_snippets.txt** - Copy-paste LaTeX for paper

### Options

```matlab
opts.close_figures = true;   % Close after saving (default: true)
opts.verbose = true;         % Print progress (default: true)

generate_paper_outputs(results_dir, opts);
```

**Tip**: Set `close_figures = false` to inspect figures interactively

### Customizing Figures

To modify figure appearance:

1. **Option 1**: Edit `src/output/generate_paper_outputs.m`
   - Change colors, line styles, fonts
   - Regenerate figures instantly

2. **Option 2**: Load data and plot manually
   ```matlab
   mc_data = load(fullfile(results_dir, 'monte_carlo.mat'));
   % Create custom plots using mc_data
   ```

## Best Practices

### Study Design

**1. Start Small, Scale Up**
```matlab
% Development (fast)
mc_options.N_trials = 50;
results_dir = simulate_monte_carlo(...);

% Pilot (validate approach)
mc_options.N_trials = 500;
results_dir = simulate_monte_carlo(...);

% Final (publication)
mc_options.N_trials = 2000;
results_dir = simulate_monte_carlo(...);
```

**2. Use Consistent Seeds for Comparison**
```matlab
mc_options.seed = 42;  % Same seed → reproducible results
```

**3. Name Runs Descriptively**
```matlab
run_label = sprintf('%s_N%d_mass%.0fpct', ...
    trajectory_name, n_trials, mass_uncertainty*100);
% Example: "square_N500_mass10pct"
```

### Trajectory Selection

**Good trajectories for robustness testing**:
- Diverse maneuvers (climb, turn, descend)
- Multiple speeds
- Long enough to reach steady-state
- Representative of real operations

**Example**:
```json
{
  "waypoints": [
    {"time": 0, "x": 0, "y": 0, "z": 0},
    {"time": 3, "x": 0, "y": 0, "z": 1},  // Climb
    {"time": 7, "x": 2, "y": 0, "z": 1},  // Forward
    {"time": 11, "x": 2, "y": 2, "z": 1}, // Turn
    {"time": 15, "x": 0, "y": 2, "z": 1}, // Return
    {"time": 19, "x": 0, "y": 0, "z": 1}, // Position
    {"time": 22, "x": 0, "y": 0, "z": 0}  // Land
  ]
}
```
```json
{
  "waypoints": [
    {"time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},        // North (radians)
    {"time": 3, "x": 0, "y": 0, "z": 1, "yaw": null},     // Auto from velocity
    ...
  ]
}
```

### Parameter Selection

**What to perturb**:
- ✅ Uncertain parameters (manufacturing tolerances, wear)
- ✅ Mission variations (payload mass, battery depletion)
- ✅ Environmental factors (gravity for different planets)

**What NOT to perturb**:
- ❌ Well-known constants (g on Earth)
- ❌ Design choices (if you control them exactly)
- ❌ Parameters that violate physics (negative mass)

### Data Management

**Organize results**:
```
results/
├── square_N500_20251101_140322/     # Dated runs
├── square_N1000_20251101_153045/
└── hover_N500_20251101_160133/
```

**Archive important runs**:
```matlab
% Compress for archival
zip('square_N500_20251101_140322.zip', 'results/square_N500_20251101_140322');
```

**Delete old runs**:
```matlab
% Clean up test runs
rmdir('results/test_*', 's');
```

## Troubleshooting

### Problem: Study Takes Too Long

**Solutions**:
1. Enable parallel processing:
   ```matlab
   mc_options.parallel = true;
   ```

2. Reduce trials for development:
   ```matlab
   mc_options.N_trials = 100;  % Instead of 500
   ```

3. Use simpler trajectory (shorter duration)

4. Increase integration timestep:
   ```matlab
   % In simulate_trajectory options
   options.dt = 0.02;  % Instead of 0.01 (2x speedup)
   ```

### Problem: High Failure Rate

**Diagnosis**: Check metrics file for failure analysis

**Common causes**:
1. **Parameter range too wide**
   - Solution: Reduce uncertainty (e.g., ±5% instead of ±20%)

2. **Aggressive trajectory**
   - Solution: Use gentler maneuvers, longer segments

3. **Controller not robust**
   - Solution: Retune LQR weights (increase Q for position)

**Example fix**:
```matlab
% Was too aggressive
perturb_config.params = {
    'm', 'uniform', 0.2, 0.8  % 60% variation - TOO WIDE
};

% More realistic
perturb_config.params = {
    'm', 'normal', 0.5, 0.025  % ±5% (3σ = 15%) - BETTER
};
```

### Problem: Figures Look Wrong

**Check**:
1. Did `simulate_monte_carlo` complete successfully?
2. Do `nominal.mat` and `monte_carlo.mat` exist?
3. Are you using correct `results_dir` path?

**Debug**:
```matlab
% Verify data files
ls(results_dir)

% Check structure
mc_data = load(fullfile(results_dir, 'monte_carlo.mat'));
fieldnames(mc_data.mc_results)
```

### Problem: Out of Memory

**For large studies** (>2000 trials):

```matlab
% Option 1: Don't save all trial data
mc_options.save_all_trials = false;  % Only saves statistics

% Option 2: Use more memory-efficient datatypes
% (already optimized in framework)

% Option 3: Split into multiple studies
for batch = 1:4
    mc_options.seed = 42 + batch;
    mc_options.N_trials = 500;
    simulate_monte_carlo(..., sprintf('batch_%d', batch), ...);
end
```

## Example Studies

### Study 1: Manufacturing Tolerance

**Question**: Does controller work with ±10% manufacturing variation?

```matlab
perturb_config.params = {
    'm',   'normal', 0.5,  0.05;   % ±10% mass
    'Ixx', 'normal', 0.01, 0.001;  % ±10% inertia
    'Iyy', 'normal', 0.01, 0.001;
    'Izz', 'normal', 0.02, 0.002;
};

mc_options.N_trials = 500;
simulate_monte_carlo('basic_maneuver.wpt', 'manufacturing_tol', perturb_config, mc_options);
```

### Study 2: Payload Variation

**Question**: Performance with 0-200g payload?

```matlab
params = quadrotor_linear_6dof();
m_base = params.m;  % Base mass (no payload)

perturb_config.params = {
    'm', 'uniform', m_base, m_base + 0.2;  % 0-200g payload
};

mc_options.N_trials = 300;
simulate_monte_carlo('delivery_path.wpt', 'payload_study', perturb_config, mc_options);
```

### Study 3: Sensitivity Analysis

**Question**: Which parameter matters most?

```matlab
% Test each parameter individually
params_to_test = {'m', 'Ixx', 'Iyy', 'Izz', 'L'};

for i = 1:length(params_to_test)
    perturb_config.params = {
        params_to_test{i}, 'normal', nominal_value, uncertainty
    };
    
    run_label = sprintf('sensitivity_%s', params_to_test{i});
    simulate_monte_carlo('test_trajectory.wpt', run_label, perturb_config, mc_options);
end

% Compare correlation values across studies
```

## Publishing Results

### For Conference Papers

Include:
1. **Description**: "500-trial Monte Carlo study with ±10% parameter uncertainties"
2. **Metrics**: Mean ± std dev for key metrics
3. **Success rate**: "99.2% trials completed successfully"
4. **Figures**: Distribution histograms, boxplots, correlation matrix

### For Journal Papers

Additionally include:
1. **Sensitivity analysis**: Correlation coefficients
2. **Failure analysis**: What caused unsuccessful trials
3. **Statistical tests**: Confidence intervals, hypothesis tests
4. **Comparison**: Against other controllers

### LaTeX Snippets

The `latex_snippets.txt` file contains ready-to-use LaTeX:

```latex
% From latex_snippets.txt
Monte Carlo validation: $N = 500$ trials with $99.6\%$ success rate

Performance under parameter uncertainty:
$0.057 \pm 0.012$ m (mean $\pm$ std)

Attitude tracking most sensitive to $I_{yy}$ ($r = +0.512$)
```

## Next Steps

- **Understand architecture**: [Architecture Overview](./architecture.md)
- **Learn API**: [API Reference](./api_reference.md)
- **Custom controllers**: Modify `compute_lqr_control.m`
- **Advanced analysis**: Write custom analysis scripts using saved data
