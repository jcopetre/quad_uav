# Quadrotor 6DOF LQR Control - Pure MATLAB Implementation

A complete simulation framework for trajectory tracking control of a 6 degree-of-freedom quadrotor UAV using Linear Quadratic Regulator (LQR) control with multiple trajectory generation methods.

## 📁 Project Structure

```
<root>/
│
├── README.md                            [This file]
├── Constants.m                          [Shared constants]
├── init_project.m                       [Path initialization script]
├── simulate_quadrotor_pure.m            [Main simulation script - RUN THIS]
├── simulate_quadrotor_pure_main.m       [Example usage script]
├── simulate_monte_carlo.m               [Monte Carlo simulation orchestrator]
│
├── vehicle/
│   └── quadrotor_linear_6dof.m          [Vehicle model, parameters, LQR design]
│
├── trajectories/
│   ├── load_waypoints.m                 [Waypoint file loader (JSON)]
│   ├── generate_trajectory_interp.m     [MAKIMA interpolation-based generation]
│   ├── generate_trajectory_minsnap.m    [Minimum snap optimization]
│   └── *.wpt                            [Waypoint definition files (JSON)]
│
├── control/
│   ├── compute_lqr_control.m            [LQR control law implementation]
│   ├── get_reference_state.m            [Reference state lookup]
│   └── quadrotor_closed_loop_dynamics.m [Closed-loop system for ODE solver]
│
├── dynamics/
│   └── quadrotor_dynamics_pure.m        [Nonlinear 6DOF dynamics]
│
├── utilities/
│   ├── simulate_quadrotor.m             [ODE simulation with control logging]
│   ├── compute_performance_metrics.m    [Standardized performance evaluation]
│   ├── traj_feasibility_check.m         [Trajectory validation utilities]
│   ├── run_monte_carlo.m                [Parallel Monte Carlo trial execution]
│   └── analyze_monte_carlo_results.m    [Monte Carlo results analysis and visualization]
│
├── test/
│   ├── setup_test_environment.m         [Test environment configuration]
│   ├── run_tests.m                      [Automated test runner]
│   ├── test_*.m                         [Unit test suites]
│   ├── inttest_*.m                      [Integration tests]
│   └── quick_*.m                        [Quick validation scripts]
│
└── results/
    └── simulation_*.mat                 [Saved simulation outputs]
```

## 🔄 Data Flow Architecture

### High-Level Flow

```
simulate_quadrotor_pure.m (Main Orchestrator)
    │
    ├──> quadrotor_linear_6dof() → LQR controller design
    │
    ├──> load_waypoints() → Parse .wpt file
    │    └──> generate_trajectory_*() → Smooth trajectory
    │
    └──> simulate_quadrotor() → Run ODE45
         │
         └──> (Each time step)
              ├──> get_reference_state() → x_ref(t)
              ├──> compute_lqr_control() → u = u₀ - K(x - x_ref)
              └──> quadrotor_dynamics_pure() → ẋ = f(x,u)
```

Monte Carlo simulations are handled by:
```
simulate_monte_carlo.m → Orchestrates trials
    └──> run_monte_carlo.m → Executes trials in parallel with parfor
    └──> analyze_monte_carlo_results.m → Computes statistics and generates plots
```

For detailed component flow diagrams, see the comprehensive architecture documentation in the extended README sections.

## 🎯 State Vector Definition

```
x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]ᵀ  (12 states)

Where:
  Position:        x, y, z           [m]
  Attitude:        φ, θ, ψ           [rad] (roll, pitch, yaw)
  Linear Velocity: ẋ, ẏ, ż           [m/s]
  Angular Rate:    p, q, r           [rad/s]
```

## 🎮 Control Input Definition

```
u = [F, τ_φ, τ_θ, τ_ψ]ᵀ  (4 inputs)

Where:
  Thrust:  F                [N]
  Torques: τ_φ, τ_θ, τ_ψ    [N·m]
```

## 🧭 Coordinate Frame Convention

NED (North-East-Down):
* +X : North (forward)
* +Y : East (right)
* +Z : Down (toward ground)
* Gravity : Acts in +Z direction (downward)
* Thrust : Acts in -Z direction (upward, opposes +Z)

This is the standard aerospace convention used throughout the simulation.

## 🚀 Quick Start

### 1. Initialize Project Environment

Before running any simulations, initialize the MATLAB path:

```matlab
init_project
```

This adds all necessary directories to your MATLAB path. Run once per session or use `savepath` for permanence.

### 2. Run Basic Simulation

```matlab
simulate_quadrotor_pure('basic_maneuver.wpt');
```

### 3. Run with Custom LQR Tuning

```matlab
% Aggressive position tracking
Q = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R = diag([0.01 0.5 0.5 0.5]);
simulate_quadrotor_pure('basic_maneuver.wpt', Q, R);
```

### 4. Run Monte Carlo Simulation

```matlab
% Run 1000 trials with perturbations
perturb_params.sigma_m = 0.05; % Mass std dev (kg)
perturb_params.sigma_I = 0.001; % Inertia std dev (kg·m²)
perturb_params.sigma_p = 0.1; % Initial position std dev (m)
results = simulate_monte_carlo('basic_maneuver.wpt', [], [], 1000, perturb_params);
```

### Expected Output

```
================================================================
       Quadrotor 6DOF LQR Control System
       Pure MATLAB Implementation
================================================================

Step 1/6: Loading quadrotor model and designing controller...
Step 2/6: Loading trajectory...
Step 3/6: Preparing trajectory from waypoints...
Step 4/6: Setting up initial conditions...
Step 5/6: Running ODE simulation...
Step 6/6: Computing performance metrics...

================================================================
                    PERFORMANCE SUMMARY
================================================================
Position Tracking:
  RMSE:           0.0234 m
  Max error:      0.0456 m
  Time in bounds: 95.2% (within 10cm)

Attitude:
  RMSE:           1.23 deg
  Max roll:       5.67 deg
  Max pitch:      6.12 deg

Control Effort:
  Total effort:   145.67
  Mean thrust:    4.91 N (hover: 4.91 N)
  Thrust sat.:    0.0% of time
  Torque sat.:    0.0% of time

Success Criteria:
  Completed:      YES
  Tracking OK:    YES (RMSE < 0.5m)
  Attitude safe:  YES (angles < 60deg)
  Overall:        YES

Summary Score:    0.386 (lower is better)
================================================================
```

## 🧪 Testing

The project includes comprehensive unit and integration tests, automatically discovered and executable individually or as a suite.

### Run All Tests

```matlab
cd test
run_tests
```

### Run Individual Test Suites

```matlab
test_linear_6dof          % Vehicle model tests (7 tests)
test_dynamics_pure        % Nonlinear dynamics tests (7 tests)
test_control_loop         % Control loop tests (8 tests)
test_waypoints            % Waypoint loader tests (8 tests)
test_trajectory_interp    % MAKIMA interpolation trajectory tests (11 tests)
test_trajectory_minsnap   % Minimum snap trajectory tests (20 tests)
inttest_trajectory_closedloop    % Compare trajectory methods with LQR
inttest_minsnap_visualization    % Visual validation of minimum snap
quick_test_sim                   % Fast end-to-end test
find_angle_limit                 % Empirical controller limit analysis
```

**Test Coverage**: 61+ unit tests across all core functionality. See individual test files for detailed descriptions.

## 📊 Trajectory Generation Methods

### Method 1: MAKIMA Interpolation (`generate_trajectory_interp.m`)

**Best for**: Real-time applications, rapid prototyping, general-purpose use

**Characteristics**:
* Fast generation (< 0.1s for typical trajectories)
* C¹ continuity (smooth velocity)
* Non-zero velocities at waypoints (natural motion)
* Reduced oscillations vs. spline methods
* Robust to unequally-spaced waypoints

**When to use**:
* Real-time trajectory planning
* Rapid iteration during development
* When computation time is critical
* When acceptable smoothness is sufficient

### Method 2: Minimum Snap Optimization (`generate_trajectory_minsnap.m`)

**Best for**: Offline planning, aggressive maneuvers, optimal performance

**Characteristics**:
* Minimizes snap (4th derivative of position)
* C⁴ continuity (very smooth)
* Optimal for aggressive flight (reduces jerk in controls)
* Slower generation (0.5-2s depending on waypoint count)
* Follows Mellinger & Kumar (2011) formulation
* Requires Optimization Toolbox for `quadprog`

**When to use**:
* Offline trajectory optimization
* Aggressive racing or aerobatic maneuvers
* When control smoothness is critical
* When optimality matters more than speed

### Comparison

```matlab
% Generate with both methods
traj_interp = generate_trajectory_interp(wpt, params, 0.01);
traj_minsnap = generate_trajectory_minsnap(wpt, params, 0.01);

% Run integration test to compare
cd test
inttest_trajectory_closedloop
```

See `test/inttest_trajectory_closedloop.m` for detailed performance comparison metrics.

## 🛠️ Customization

### Define Trajectories

#### Method 1: Waypoint Files (Recommended)

Create a `.wpt` file (JSON format) in `./trajectories/`:

```json
{
  "metadata": {
    "name": "Basic Test Flight",
    "description": "Takeoff, maneuver, and land",
    "created": "2025-01-09",
    "vehicle": "quadrotor_500g"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "climb", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "forward", "time": 5, "x": 2, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 8, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

Load in simulation:

```matlab
simulate_quadrotor_pure('my_trajectory.wpt');
```

**Yaw Behavior**:
* Explicit value (e.g., `1.57`): UAV orients to specified heading
* `null` (in JSON): Auto-calculate from velocity direction
* Useful for camera pointing, inspection tasks, or wind compensation

**Benefits of .wpt Files**:
* Version control trajectory designs separately
* Self-documenting with metadata
* Easy batch processing for Monte Carlo analysis
* Non-programmers can define flight paths
* Reusable across experiments

#### Method 2: Direct Definition (Quick Testing)

```matlab
% Matrix format: [time, x, y, z, yaw]
waypoints = [
    0,   0,   0,   0,   0;
    3,   2,   0,   1,   NaN;    % NaN for auto yaw
    10,  0,   0,   0,   0;
];
trajectory = generate_trajectory_interp(waypoints, params);
```

### Tune LQR Controller

```matlab
% Define custom weights
Q = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);
R = diag([0.5 2 2 2]);
simulate_quadrotor_pure('my_trajectory.wpt', Q, R);
```

#### Understanding Q and R Matrices

**Q Matrix (State Penalty)** - 12×12 diagonal

```matlab
Q = diag([qx qy qz qφ qθ qψ qvx qvy qvz qp qq qr])
```

* Higher Q values → Tighter tracking, more aggressive corrections
* Lower Q values → Looser tracking, gentler response

**R Matrix (Control Penalty)** - 4×4 diagonal

```matlab
R = diag([rF rτφ rτθ rτψ])
```

* Higher R values → Smaller control inputs, smoother but slower
* Lower R values → Larger control inputs, faster but more aggressive

**Practical Tuning Guidelines**:

| Desired Behavior | Modification | Trade-off |
| -- | -- | -- |
| Tighter position tracking | Increase position Q | More control effort |
| Smoother flight | Increase R | Slower tracking |
| Faster response | Decrease R | Risk of oscillation |
| Reduce oscillations | Increase velocity Q | Less aggressive |

**Example Configurations**:

```matlab
% Aggressive tracking (racing)
Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R_aggressive = diag([0.01 0.5 0.5 0.5]);

% Smooth operation (aerial photography)
Q_smooth = diag([50 50 100 5 5 2 5 5 10 0.5 0.5 0.1]);
R_smooth = diag([1 5 5 5]);

% Energy efficient (long flight time)
Q_efficient = diag([80 80 100 8 8 1 8 8 10 1 1 0.2]);
R_efficient = diag([5 2 2 2]);
```

### Modify Vehicle Parameters

Edit `./vehicle/quadrotor_linear_6dof.m`:

```matlab
params.m = 1.0;      % Mass (kg)
params.L = 0.30;     % Arm length (m)
params.Ixx = 0.01;   % Inertia (kg·m²)
```

## 🎓 Theory

### LQR Controller

Linear Quadratic Regulator (LQR) finds optimal feedback gains by minimizing:

```matlab
J = ∫ (xᵀQx + uᵀRu) dt
```

**How It Works**:
1. **Algebraic Riccati Equation**: MATLAB’s `lqr()` solves:
   ```
   AᵀS + SA - SBR⁻¹BᵀS + Q = 0
   ```
2. **Optimal Gain**:
   ```
   K = R⁻¹BᵀS
   ```
3. **Control Law**:
   ```
   u = u_hover - K(x - x_ref)
   ```

**Key Properties**:
* Guaranteed stability (all closed-loop poles in left half-plane)
* Optimal for linear systems
* Intuitive tuning via Q and R matrices
* Robust margins (60°/6dB gain/phase)

**Why It Works for Quadrotors**:
* System linearized around hover
* Small deviations → linear approximation accurate
* LQR provides inherent robustness
* Quadrotor dynamics “mildly nonlinear” near hover

**Limitations**:
* Performance degrades far from linearization point
* No guarantees for large angles or aggressive flight
* Valid operating region: typically < 15° roll/pitch

See `test/find_angle_limit.m` for empirical controller limit analysis.

### Linearization Validity

The LQR controller uses small-angle assumptions (sin(θ) ≈ θ, cos(θ) ≈ 1). The framework checks linearization validity:

```matlab
metrics = compute_performance_metrics(t, x, trajectory, params, u_log);

if metrics.linearity.violated
    fprintf('⚠ Linearization assumptions violated!\n');
    fprintf('  Max attitude: %.1f deg (limit: 15 deg)\n', ...
            rad2deg(metrics.linearity.max_actual_attitude));
    fprintf('  Severity: %s\n', metrics.linearity.severity);
end
```

Validate trajectory demands with:

```matlab
[feasible, warnings] = check_trajectory_feasibility(trajectory, params);
if ~feasible
    for i = 1:length(warnings)
        fprintf('⚠ %s\n', warnings{i});
    end
end
```

## 📝 Key File Descriptions

| File | Purpose |
| -- | -- |
| `init_project.m` | Initialize MATLAB path (run once per session) |
| `simulate_quadrotor_pure.m` | Main simulation orchestrator |
| `simulate_monte_carlo.m` | Orchestrates Monte Carlo simulations |
| `quadrotor_linear_6dof.m` | Vehicle model and LQR design |
| `load_waypoints.m` | JSON waypoint file parser |
| `generate_trajectory_interp.m` | MAKIMA interpolation-based trajectory |
| `generate_trajectory_minsnap.m` | Minimum snap optimization using `quadprog` |
| `compute_lqr_control.m` | LQR control law with saturation |
| `get_reference_state.m` | Reference state interpolation |
| `quadrotor_closed_loop_dynamics.m` | ODE wrapper function |
| `quadrotor_dynamics_pure.m` | Nonlinear 6DOF dynamics (NED) |
| `simulate_quadrotor.m` | ODE45 integration with control logging |
| `compute_performance_metrics.m` | Standardized performance evaluation |
| `run_monte_carlo.m` | Parallel Monte Carlo trial execution using `parfor` |
| `analyze_monte_carlo_results.m` | Monte Carlo results analysis with statistics and plots (histograms, boxplots) |
| `Constants.m` | Shared constants |
| `setup_test_environment.m` | Test path configuration |
| `run_tests.m` | Automated test discovery and execution |

## 📚 Output and Results

### Generated Plots

* 3D trajectory (reference vs actual)
* Position tracking (X, Y, Z vs time)
* Attitude tracking (roll, pitch, yaw)
* Control inputs (thrust and torques)
* Velocity profiles
* Monte Carlo results (histograms, boxplots)

### Saved Data

`./results/simulation_[trajectory]_[timestamp].mat` contains:
* Time history
* State history
* Control history
* Performance metrics
* Parameters used
* Configuration

### Performance Metrics

Computed by `compute_performance_metrics.m` and `analyze_monte_carlo_results.m`:
* **Tracking Performance**:
  - Position RMSE (overall and per-axis)
  - Attitude RMSE
  - Time in bounds (percentage within tolerance)
  - Maximum errors
* **Control Effort**:
  - Total control effort (integral of u²)
  - Mean/max thrust and torques
  - Saturation analysis (% of time saturated)
* **Linearization Validity**:
  - Maximum attitude angles
  - Angular velocity bounds
  - Time spent violating linearization assumptions
  - Severity assessment (NONE/MILD/MODERATE/SEVERE)
* **Monte Carlo Statistics**:
  - Mean, median, std, min/max RMSE
  - Success rate (e.g., RMSE < 0.5m)
  - Percentiles (e.g., 95th)

## 🔬 Advanced Usage

### Monte Carlo Analysis

```matlab
% Fixed controller, varied mass
params_nominal = quadrotor_linear_6dof();
perturb_params.sigma_m = 0.05;
perturb_params.sigma_I = 0.001;
perturb_params.sigma_p = 0.1;
results = simulate_monte_carlo('basic_maneuver.wpt', [], [], 1000, perturb_params);
analyze_monte_carlo_results(results);

% Visualize results
figure;
histogram(cellfun(@(r) r.metrics.tracking.rmse_position, results));
xlabel('Position RMSE (m)'); title('Monte Carlo Robustness');
```

### Custom Initial Conditions

```matlab
% Start tilted with velocity
x0 = zeros(12, 1);
x0(4) = deg2rad(5);   % 5° roll
x0(7) = 0.5;          % 0.5 m/s forward velocity
simulate_quadrotor_pure('basic_maneuver.wpt', [], [], x0);
```

## 🛠️ Requirements

- MATLAB R2019b or later
- Control System Toolbox (for `lqr`)
- Optimization Toolbox (for `quadprog` in minimum snap)
- Parallel Computing Toolbox (optional, for `parfor` in Monte Carlo)
- Statistics and Machine Learning Toolbox (optional, for `boxplot` in Monte Carlo visualization)
- No Simulink required

## 🔜 Future Work

### Planned Features
* Extended Kalman Filter (EKF) state estimation
* Multiple controller comparison framework (PID, MPC, etc.)
* Hardware deployment utilities
* Automatic gain tuning utilities
* KKT solver for minimum snap to eliminate Optimization Toolbox dependency

### Research Extensions
* Gain scheduling for large-angle maneuvers
* Nonlinear control methods (backstepping, sliding mode)
* Trajectory optimization with obstacle avoidance
* Formation flight control

## 📚 References
* LQR Theory: Anderson, B. D., & Moore, J. B. (1990). *Optimal Control: Linear Quadratic Methods*
* Quadrotor Dynamics: Bouabdallah, S. (2007). *Design and Control of Quadrotors with Application to Autonomous Flying*
* Minimum Snap: Mellinger, D., & Kumar, V. (2011). *Minimum Snap Trajectory Generation and Control for Quadrotors*. ICRA 2011.
* NED Coordinates: Standard aerospace convention (ISO 8855, SAE J670)

## 📧 Contact

Project maintained as part of quadrotor control research.

**Last Updated**: October 23, 2025