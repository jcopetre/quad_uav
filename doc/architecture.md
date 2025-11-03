# Architecture Overview

Complete system design documentation for the Quadrotor LQR Control Framework.

## Table of Contents

- [System Overview](#system-overview)
- [Directory Structure](#directory-structure)
- [Data Flow](#data-flow)
- [Coordinate Frames](#coordinate-frames)
- [State and Control Definitions](#state-and-control-definitions)
- [Trajectory Generation](#trajectory-generation)
- [Control System](#control-system)
- [Data Management](#data-management)
- [Testing Framework](#testing-framework)

## System Overview

The framework implements a complete closed-loop simulation pipeline:

```
Waypoints (.wpt) → Trajectory Generation → LQR Control → 6DOF Dynamics → Analysis
```

**Key Design Principles**:
1. **Separation of Concerns**: Trajectory, control, and dynamics are decoupled
2. **Data Validation**: Strict schemas prevent runtime errors
3. **Pure MATLAB**: No Simulink, minimal external dependencies
4. **Research Transparency**: Explicit parameters, no hidden helper functions
5. **Reproducibility**: Deterministic with seed control for Monte Carlo

## Directory Structure

### Core Framework (`src/`)

```
src/
├── simulation/          # Single trajectory simulation
│   ├── simulate_trajectory.m         # Main orchestrator
│   └── ode_simulate.m                # ODE45 integration wrapper
│
├── monte_carlo/         # Robustness analysis
│   ├── simulate_monte_carlo.m        # MC orchestrator (high-level)
│   ├── run_monte_carlo.m             # MC engine (parallel trials)
│   └── analyze_monte_carlo.m         # Statistical analysis
│
├── trajectory/          # Trajectory generation
│   ├── load_waypoints.m              # JSON waypoint parser
│   ├── generate_trajectory_auto.m    # Intelligent method selection
│   ├── generate_trajectory_interp.m  # MAKIMA interpolation
│   └── generate_trajectory_minsnap.m # Minimum snap optimization
│
├── output/              # Results generation
│   ├── generate_paper_outputs.m      # Publication figures
│   ├── write_unified_metrics_report.m    # Unified metrics writer
│   ├── write_run_log.m                   # MC configuration log
│   └── write_latex_snippets.m            # LaTeX snippet generator
│
├── data/                # Data management
│   ├── DataManager.m                 # I/O with validation
│   └── DataSchemas.m                 # Struct definitions
│
└── utils/               # Utility functions
    └── set_default_options.m         # Options handling
```

### Support Modules

```
vehicle/                 # Vehicle parameters
└── quadrotor_linear_6dof.m          # Model + LQR design

control/                 # Control laws
├── compute_lqr_control.m            # LQR control law
├── get_reference_state.m            # Trajectory lookup
└── quadrotor_closed_loop_dynamics.m # Closed-loop ODE

dynamics/                # Plant models
└── quadrotor_dynamics_pure.m        # Nonlinear 6DOF

trajectories/            # Waypoint files
└── *.wpt                            # JSON trajectory definitions

examples/                # Usage demonstrations
├── basic_simulation.m               # Simple examples
└── robustness_study.m               # Monte Carlo workflow

test/                    # Test suite
├── run_tests.m                      # Automated test runner
└── test_*.m                         # 61+ unit tests
```

## Data Flow

### Single Simulation Flow

```
simulate_trajectory()                    [src/simulation/]
    │
    ├──> quadrotor_linear_6dof()         [vehicle/]
    │    └──> Designs LQR controller (K matrix, u_hover)
    │
    ├──> load_waypoints()                [src/trajectory/]
    │    └──> Parses .wpt JSON file
    │
    ├──> generate_trajectory_auto()      [src/trajectory/]
    │    ├──> Analyzes segment durations
    │    ├──> Selects method (interp vs minsnap)
    │    └──> Generates smooth reference trajectory
    │
    └──> ode_simulate()                  [src/simulation/]
         │
         └──> ODE45 integration loop:
              ├──> get_reference_state()       [control/]
              ├──> compute_lqr_control()       [control/]
              ├──> quadrotor_dynamics_pure()   [dynamics/]
              └──> Log states and controls
```

### Monte Carlo Flow

```
simulate_monte_carlo()                   [src/monte_carlo/]
    │
    ├──> Creates organized output directory
    │    ./results/<run_label>_<timestamp>/
    │
    ├──> run_monte_carlo()               [src/monte_carlo/]
    │    │
    │    ├──> Design nominal controller (once)
    │    │
    │    ├──> PARFOR loop over N trials:
    │    │    ├──> Perturb parameters (m, I, L)
    │    │    ├──> simulate_trajectory()
    │    │    └──> Collect metrics
    │    │
    │    └──> analyze_monte_carlo()
    │         └──> Statistics, correlations, sensitivity
    │
    ├──> DataManager.save_results()      [src/data/]
    │    └──> Saves nominal.mat
    │
    ├──> DataManager.save_monte_carlo()  [src/data/]
    │    └──> Saves monte_carlo.mat
    │
    └──> Write mc_run_log.txt (configuration only)
```

Later (separate call):
```
generate_paper_outputs(results_dir)     [src/output/]
    │
    ├──> Loads nominal.mat, monte_carlo.mat
    ├──> Generates 7 publication figures
    ├──> Analyze results (correlations, sensitivity)
    ├──> Write analysis_report.txt (comprehensive metrics)
    └──> Write latex_snippets.txt (LaTeX snippets)
```

## Coordinate Frames

### NED (North-East-Down) Frame

The framework uses standard aerospace NED coordinates:

```
     North (+X)
        ↑
        │
        │
        └────→ East (+Y)
       ╱
      ╱
     ↓
   Down (+Z)
```

**Convention Details**:
- **+X**: North (forward for most maneuvers)
- **+Y**: East (right)
- **+Z**: Down (toward ground)
- **Gravity**: g = [0; 0; 9.81] m/s² (acts in +Z direction)
- **Thrust**: Points in -Z direction (upward, opposes gravity)

**Why NED?**
- Standard in aerospace (ISO 8855, SAE J670)
- Matches GPS/INS conventions
- Right-handed coordinate system
- Positive altitude = negative Z

### Attitude Representation

Euler angles (3-2-1 sequence):
```
R = Rz(ψ) * Ry(θ) * Rx(φ)
```

- **Roll (φ)**: Rotation about X-axis (body pitch axis)
- **Pitch (θ)**: Rotation about Y-axis (body roll axis)  
- **Yaw (ψ)**: Rotation about Z-axis (heading)

**Sign Conventions**:
- Positive roll → right wing down
- Positive pitch → nose up
- Positive yaw → turn right (clockwise when viewed from above)

## State and Control Definitions

### State Vector (12 states)

```matlab
x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]ᵀ
```

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 1-3 | x, y, z | Position (NED) | m |
| 4-6 | φ, θ, ψ | Euler angles (roll, pitch, yaw) | rad |
| 7-9 | ẋ, ẏ, ż | Linear velocity (NED) | m/s |
| 10-12 | p, q, r | Angular rates (body frame) | rad/s |

**Body angular rates** (p, q, r):
- p: Roll rate (about body X)
- q: Pitch rate (about body Y)
- r: Yaw rate (about body Z)

### Control Vector (4 inputs)

```matlab
u = [F, τ_φ, τ_θ, τ_ψ]ᵀ
```

| Index | Symbol | Description | Units | Typical Range |
|-------|--------|-------------|-------|---------------|
| 1 | F | Total thrust | N | 0 - 10 N |
| 2 | τ_φ | Roll torque | N·m | ±0.05 N·m |
| 3 | τ_θ | Pitch torque | N·m | ±0.05 N·m |
| 4 | τ_ψ | Yaw torque | N·m | ±0.01 N·m |

**Actuation Limits** (enforced in dynamics):
- Thrust: [0, 10] N (can't pull downward)
- Torques: ±0.05 N·m (roll/pitch), ±0.01 N·m (yaw)

Limits defined in `quadrotor_linear_6dof.m` (single source of truth).

### Nominal Operating Point

The LQR controller is designed around hover equilibrium:

```matlab
x_hover = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]  % Stationary at origin
u_hover = [m*g; 0; 0; 0]                         % Thrust balances weight
```

## Trajectory Generation

### Automatic Method Selection

`generate_trajectory_auto()` intelligently selects between two methods:

```
Analyze waypoint segment durations
    │
    ├─ ANY segment < 3s? → Use MAKIMA interpolation
    │                       (Prevents excessive accelerations)
    │
    └─ ALL segments ≥ 3s? → Use minimum snap optimization
                            (Optimal smoothness for long segments)
```

**Why 3 seconds?**  
Empirically determined threshold where minimum snap starts generating feasible trajectories without exceeding small-angle assumptions.

### Method 1: MAKIMA Interpolation

**File**: `generate_trajectory_interp.m`

**Approach**:
- MAKIMA (Modified Akima) spline interpolation
- Preserves waypoint positions exactly
- Smooth first derivatives
- Fast computation
- Conservative (safer for short segments)

**Best for**:
- Quick maneuvers (< 3s segments)
- When waypoints define precise path
- Hover-to-hover transitions

**Limitations**:
- Not optimal in any mathematical sense
- May have acceleration discontinuities

### Method 2: Minimum Snap Optimization

**File**: `generate_trajectory_minsnap.m`

**Approach**:
- Minimizes 4th derivative (snap) → smooth accelerations
- Quadratic programming problem (`quadprog`)
- Globally optimal trajectory
- Continuous derivatives up to jerk

**Best for**:
- Long, smooth maneuvers (≥ 3s segments)
- When optimality matters
- Photogrammetry, inspection tasks

**Limitations**:
- Can generate excessive accelerations for short segments
- Computationally expensive (but still fast)
- Requires Optimization Toolbox

**Reference**: Mellinger & Kumar (2011), "Minimum Snap Trajectory Generation"

### Waypoint File Format

JSON format with metadata:

```json
{
  "metadata": {
    "name": "Flight Name",
    "description": "What this trajectory does",
    "created": "2025-11-01"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "hover", "time": 3, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 6, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

**Yaw specification**:
- Explicit value (0.0): Fixed heading
- `null`: Auto-calculate from velocity direction
  - ⚠️ **Warning**: Auto-calculated yaw can produce excessive yaw rates on curved trajectories (e.g., circles, figure-eights), potentially exceeding actuator limits or linearization validity. Always check trajectory feasibility (see [Trajectory Feasibility Checking](#trajectory-feasibility-checking)) when using auto-yaw.
  - **Best practice**: Use constant yaw (e.g., 0.0) unless heading must track velocity direction
  - **Alternative**: Manually specify yaw at waypoints for smoother transitions

## Trajectory Feasibility Checking

**File**: `check_trajectory_feasibility.m`

Validates trajectory demands against vehicle physical constraints before simulation.

### Purpose

Catches infeasible trajectories early to prevent:
- Excessive yaw rates exceeding actuator capabilities
- Aggressive maneuvers violating linearization assumptions
- Poor tracking performance from unrealistic reference trajectories

### Checks Performed
```matlab
feasibility = check_trajectory_feasibility(trajectory, params)

% Returns structure:
feasibility.feasible           % Boolean: overall feasibility
feasibility.warnings           % Cell array of warning messages
feasibility.violations
    .max_attitude              % Maximum reference attitude [deg]
    .max_velocity              % Maximum velocity [m/s]
    .max_horiz_accel          % Maximum horizontal acceleration [m/s²]
    .max_yaw_rate             % Maximum yaw rate [deg/s]
    .rms_yaw_rate             % RMS yaw rate [deg/s]
    .max_yaw_accel            % Maximum yaw acceleration [deg/s²]
    .limits
        .yaw_rate_warning     % Warning threshold [deg/s]
        .yaw_accel_physical   % Physical limit [deg/s²]
```

### Validation Criteria

| Metric | Warning Threshold | Critical Limit | Rationale |
|--------|------------------|----------------|-----------|
| Yaw rate | 50 deg/s | 100 deg/s | Actuator saturation |
| Yaw acceleration | τ_max / I_zz | - | Physical constraint |
| Attitude angle | 15 deg | - | Linearization validity |

### Integration

Feasibility checking is automatically integrated into `simulate_trajectory()`:
- Runs after trajectory generation
- Results stored in `results.trajectory.feasibility`
- Warnings printed to console if issues detected
- Included in `analysis_report.txt`

### Example
```matlab
% Check a waypoint file
wpt = load_waypoints('aggressive_maneuver.wpt');
trajectory = generate_trajectory_auto(wpt, params);
feasibility = check_trajectory_feasibility(trajectory, params);

if ~feasibility.feasible
    fprintf('Trajectory infeasible:\n');
    for i = 1:length(feasibility.warnings)
        fprintf('  - %s\n', feasibility.warnings{i});
    end
end

% Common fix: Use constant yaw instead of auto-calculated
wpt.yaw = zeros(size(wpt.yaw));
trajectory = generate_trajectory_auto(wpt, params);
```
### Common Feasibility Issues

**Auto-Calculated Yaw on Curved Paths**

The most common source of infeasible trajectories is auto-calculated yaw (`yaw: null` in waypoint files) on curved paths:
```matlab
% ❌ PROBLEMATIC: Auto-yaw on figure-eight
wpt = load_waypoints('figure_eight_long.wpt');  % Has yaw: null
trajectory = generate_trajectory_auto(wpt, params);
feas = check_trajectory_feasibility(trajectory, params);
% WARNING: Max yaw rate 127 deg/s exceeds physical limit 87 deg/s

% ✅ SOLUTION 1: Use constant yaw
wpt.yaw = zeros(size(wpt.yaw));  % Force yaw = 0
trajectory = generate_trajectory_auto(wpt, params);
% PASS: Max yaw rate 3.2 deg/s

% ✅ SOLUTION 2: Manually specify waypoint yaw angles
% Edit .wpt file to include explicit yaw values at each waypoint
```

**Why This Happens:**

Auto-calculated yaw attempts to align heading with instantaneous velocity direction. On curved trajectories:
1. Velocity direction changes rapidly
2. Interpolation between waypoints creates high yaw rates
3. Yaw acceleration can exceed τ_max / I_zz

**When Auto-Yaw Is Safe:**
- Straight-line segments
- Gentle curves with long segment durations (>5 seconds)
- Low-speed maneuvers

**When to Avoid Auto-Yaw:**
- Tight turns or circular paths
- Figure-eight or S-curve patterns
- High-speed maneuvers
- Short segment durations (<3 seconds)

### Validation Workflow

Recommended workflow for new trajectories:
```matlab
% 1. Load and generate
wpt = load_waypoints('new_trajectory.wpt');
trajectory = generate_trajectory_auto(wpt, params);

% 2. Check feasibility
feas = check_trajectory_feasibility(trajectory, params);

% 3. If infeasible, try constant yaw
if ~feas.feasible
    fprintf('Infeasible! Trying constant yaw...\n');
    wpt.yaw = zeros(size(wpt.yaw));
    trajectory = generate_trajectory_auto(wpt, params);
    feas = check_trajectory_feasibility(trajectory, params);
end

% 4. Simulate if feasible
if feas.feasible
    results = simulate_trajectory(wpt, params.Q, params.R);
else
    fprintf('Trajectory remains infeasible. Adjust waypoints.\n');
end
```

## Control System

### LQR Design

**File**: `quadrotor_linear_6dof.m`

The LQR controller is designed around the linearized hover dynamics:

```
Linearize:  ẋ = A x + B u
Design:     K = lqr(A, B, Q, R)
Control:    u = u_hover - K(x - x_ref)
```

**Weight Matrices**:

```matlab
% Default Q (state penalty)
Q = diag([100 100 100 10 10 1 20 20 20 1 1 0.1]);
%        [x   y   z   φ  θ  ψ  ẋ  ẏ  ż  p  q  r  ]

% Default R (control penalty)
R = diag([1 2 2 2]);
%        [F τφ τθ τψ]
```

**Tuning Guidelines**:
- Higher Q values → Tighter tracking, more aggressive
- Higher R values → Gentler control, more conservative
- Balance: Q/R ratio determines response speed

### Control Law

**File**: `compute_lqr_control.m`

```matlab
% Error feedback
e = x - x_ref;

% LQR control law
u = u_hover - K * e;

% Saturation
u = clip(u, u_min, u_max);
```

**Feedforward** from trajectory:
- Reference attitudes (φ_ref, θ_ref) computed from desired acceleration
- Included in x_ref for improved tracking

### Linearization Validity

The LQR uses small-angle approximations:
- sin(θ) ≈ θ
- cos(θ) ≈ 1

**Valid region**: |φ|, |θ| < ~15° (0.26 rad)

**Checking**:
```matlab
metrics = compute_performance_metrics(t, x, trajectory, params, u_log);

if metrics.linearity.violated
    warning('Linearization assumptions violated!');
    fprintf('Max attitude: %.1f deg\n', rad2deg(metrics.linearity.max_actual_attitude));
end
```

## Data Management

### Data Schemas

**File**: `DataSchemas.m`

Defines strict contracts for all data structures:

```matlab
% Simulation result
DataSchemas.SimulationResult()
    .t              % Time vector [N×1]
    .x              % State history [N×12]
    .u_log          % Control history [N×4]
    .trajectory     % TrajectoryData struct
    .params         % Vehicle parameters
    .params_plant   % Plant parameters (MC studies, optional)
    .metrics        % PerformanceMetrics struct
    .config         % Configuration used
    .output_dir     % Output directory path (optional)

% Trajectory data
DataSchemas.TrajectoryData()
    .time           % Time vector
    .position       % [N×3]
    .velocity       % [N×3]
    .acceleration   % [N×3]
    .attitude       % [N×3] (Euler angles)
    .omega          % [N×3] (angular rates)
    .method         % 'makima' or 'minsnap'
    .waypoints      % Original waypoint structure (optional)
    .feasibility    % Feasibility check results (optional)

% Monte Carlo result
DataSchemas.MonteCarloResult()
    .config         % Configuration
    .trials         % Array of trial results
    .statistics     % Summary statistics
    .nominal        % Nominal simulation
    .param_data     % Parameter samples
```

### DataManager

**File**: `DataManager.m`

Centralized I/O with automatic validation:

```matlab
% Save with validation
DataManager.save_results(results, 'nominal', './results');

% Load with validation
nominal = DataManager.load_results('./results/nominal.mat');

% Load without validation (faster)
nominal = DataManager.load_results(filepath, struct('validate', false));
```

**Benefits**:
- Prevents field name typos
- Catches structure errors early
- Automatic migration for legacy data
- Consistent error messages

## Testing Framework

### Test Organization

```
test/
├── run_tests.m                      # Automated test runner
├── setup_test_environment.m         # Path configuration
├── test_*.m                         # Unit tests 
├── test_trajectory_feasibility.m    # Feasibility checking 
├── inttest_*.m                      # Integration tests, run manually 
└── quick_*.m                        # Fast validation 
```

### Running Tests

```matlab
cd test

% Run all tests
run_tests

% Run specific test
test_linear_6dof
test_dynamics_pure
test_trajectory_auto

% Quick validation
quick_test_sim
```

### Test Coverage

- **Vehicle Model**: Parameter validation, LQR design
- **Dynamics**: Nonlinear 6DOF, equilibrium, frame conversions
- **Control**: LQR law, saturation, reference tracking
- **Trajectories**: Waypoint loading, generation methods, auto-selection
- **Monte Carlo**: Parameter sampling, reproducibility, statistics
- **Integration**: Closed-loop, trajectory tracking, end-to-end

## Performance Considerations

### Computational Complexity

**Single Simulation**:
- Trajectory generation: O(N) for interpolation, O(N³) for minsnap
- ODE integration: ~0.5-2 seconds (MATLAB ode45)
- Total: ~1-3 seconds per simulation

**Monte Carlo** (500 trials):
- Serial: ~10-15 minutes
- Parallel (8 cores): ~2-3 minutes
- Dominated by ODE integration time

### Memory Usage

- Single simulation: ~5 MB (100s at 0.01s dt)
- Monte Carlo results: ~50-100 MB (500 trials)
- Figures (PNG): ~500 KB each

### Optimization Tips

1. **Reduce time steps**: Use `dt = 0.02` instead of `0.01` (2x speedup)
2. **Batch mode**: Disable plots with `options.plot = false`
3. **Parallel processing**: Enable with `mc_options.parallel = true`
4. **Fewer trials**: Use 100-200 for development, 500+ for final runs

## Design Decisions

### Why Pure MATLAB?

- **Accessibility**: No Simulink license required
- **Transparency**: All code visible and modifiable
- **Portability**: Runs on any MATLAB installation
- **Version Control**: Text files only (no binary .slx files)

### Why Separate Trajectory and Controller?

- **Reusability**: Same trajectory with different controllers
- **Testing**: Validate trajectory generation independently
- **Monte Carlo**: Test controller robustness on fixed trajectory

### Why LQR?

- **Optimal**: Provably optimal for linearized system
- **Systematic**: Explicit tuning via Q, R matrices
- **Fast**: Instant gain computation (no iteration)
- **Baseline**: Good reference for comparing advanced methods

### Why NED Coordinates?

- **Standard**: Aerospace convention
- **GPS Compatible**: Matches navigation sensors
- **Literature**: Most papers use NED or similar
- **Right-Handed**: Consistent with physics conventions

## Next Steps

- **API Details**: See [API Reference](./api_reference.md)
- **Usage Examples**: See [Quick Start Guide](./quick_start.md)
- **Monte Carlo**: See [Monte Carlo Guide](./monte_carlo_guide.md)
- **Extend System**: Add new controllers, dynamics models, or trajectory methods
