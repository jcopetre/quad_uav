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
│   └── generate_paper_outputs.m      # Publication figures
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
    └──> Write metrics_*.txt report
```

Later (separate call):
```
generate_paper_outputs(results_dir)     [src/output/]
    │
    ├──> Loads nominal.mat, monte_carlo.mat
    ├──> Generates 7 publication figures
    └──> Writes paper_metrics.txt (LaTeX snippets)
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
    .metrics        % PerformanceMetrics struct
    .config         % Configuration used

% Trajectory data
DataSchemas.TrajectoryData()
    .time           % Time vector
    .position       % [N×3]
    .velocity       % [N×3]
    .acceleration   % [N×3]
    .attitude       % [N×3] (Euler angles)
    .omega          % [N×3] (angular rates)
    .method         % 'interpolation' or 'minsnap'

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
├── run_tests.m                  # Automated test runner
├── setup_test_environment.m     # Path configuration
├── test_*.m                     # Unit tests (38 tests)
├── inttest_*.m                  # Integration tests (8 tests)
└── quick_*.m                    # Fast validation (3 tests)
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

**Total**: 61+ tests across all core functionality

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
