# Quadrotor 6DOF LQR Control - Pure MATLAB Implementation

A complete simulation framework for trajectory tracking control of a 6 degree-of-freedom quadrotor UAV using Linear Quadratic Regulator (LQR) control.

---

## 📁 Project Structure

```
<root>/
│
├── README.md                            [This file]
├── Constants.m                          [Shared constants for tests]
├── simulate_quadrotor_pure.m            [Main simulation script - RUN THIS]
├── quick_test_simulation.m              [Quick test script]
│
├── vehicle/
│   └── quadrotor_linear_6dof.m          [Vehicle model, parameters, LQR design]
│
├── trajectories/
│   ├── load_waypoints.m                 [Waypoint file loader (JSON)]
│   ├── generate_trajectory.m            [Smooth trajectory generation]
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
├── test/
│   ├── setup_test_environment.m         [Test environment setup]
│   ├── test_linear_6dof.m               [Vehicle model tests]
│   ├── test_dynamics_pure.m             [Dynamics tests]
│   ├── test_control_loop.m              [Control loop tests]
│   ├── test_waypoints.m                 [Waypoint loader tests]
│   └── test_trajectory.m                [Trajectory generation tests]
│
└── results/
    └── simulation_results.mat           [Saved simulation outputs]
```

---

## 🔄 Data Flow Architecture

### High-Level Flow

```
┌─────────────────────────────────────────────────────────────┐
│                 simulate_quadrotor_pure.m                   │
│                     [Main Orchestrator]                     │
└────────┬──────────────────────────────────────────┬─────────┘
         │                                          │
         ↓                                          ↓
┌─────────────────────┐                   ┌──────────────────┐
│ quadrotor_linear_   │                   │ load_waypoints   │
│ 6dof.m              │                   │ generate_        │
│                     │                   │ trajectory       │
│ • Physical params   │                   │                  │
│ • Linearized model  │                   │ • JSON file in   │
│ • LQR gains (K)     │                   │ • Smooth path    │
└──────────┬──────────┘                   │   out            │
           │                              └────────┬─────────┘
           │                                       │
           └───────────────┬───────────────────────┘
                           │
                           ↓
                  ┌────────────────┐
                  │  params struct │
                  │  trajectory    │
                  │    struct      │
                  └────────┬───────┘
                           │
                           ↓
                  ┌─────────────────────┐
                  │   ode45() solver    │
                  │   (MATLAB built-in) │
                  └──────────┬──────────┘
                             │
                             ↓
         ┌───────────────────────────────────────┐
         │  quadrotor_closed_loop_dynamics.m     │
         │  [Called at each time step]           │
         └────┬──────────────────────────────┬───┘
              │                              │
              ↓                              ↓
   ┌──────────────────────┐      ┌──────────────────────┐
   │ get_reference_state  │      │ compute_lqr_control  │
   │                      │      │                      │
   │ • Lookup ref from    │      │ • e = x - x_ref      │
   │   trajectory         │      │ • u = u₀ - K*e       │
   │ • Return x_ref       │      │ • Apply saturation   │
   └──────────┬───────────┘      └──────────┬───────────┘
              │                             │
              └──────────────┬──────────────┘
                             │
                             ↓
                  ┌─────────────────────────┐
                  │ quadrotor_dynamics_pure │
                  │                         │
                  │ • Nonlinear 6DOF        │
                  │ • ẋ = f(x, u)           │
                  │ • Return state deriv    │
                  └──────────┬──────────────┘
                             │
                             ↓
                    [Back to ode45]
                             │
                             ↓
                  ┌──────────────────┐
                  │  State History   │
                  │  Control History │
                  └──────────┬───────┘
                             │
                             ↓
                  ┌──────────────────┐
                  │  Analysis &      │
                  │  Visualization   │
                  └──────────────────┘
```

### Detailed Component Flow

```
1. INITIALIZATION
   └─> quadrotor_linear_6dof()
       ├─> Define: m, g, L, Ixx, Iyy, Izz
       ├─> Build: A, B matrices (linearized, NED coordinates)
       ├─> Design: LQR gains K from (A,B,Q,R)
       └─> Return: params struct

2. TRAJECTORY GENERATION
   └─> load_waypoints(wpt file)
       ├─> Parse .wpt file (JSON format)
       ├─> Load metadata and waypoints
       ├─> Handle null for auto yaw calculation
       ├─> Validate waypoint data
       └─> Return structure with labels and waypoint data
   
   └─> generate_trajectory(wpt, params)
       ├─> Accept JSON structure OR matrix input
       ├─> Process yaw: explicit values or auto from velocity
       ├─> Shape-preserving cubic interpolation (pchip)
       ├─> Compute: pos, vel, acc, yaw
       ├─> Feedforward: phi_d, theta_d from acc
       └─> Return: trajectory struct

3. SIMULATION LOOP (ode45 calls repeatedly)
   └─> quadrotor_closed_loop_dynamics(t, x, params, trajectory)
       │
       ├─> get_reference_state(t, trajectory)
       │   └─> Lookup x_ref at time t
       │
       ├─> compute_lqr_control(x, x_ref, params)
       │   ├─> e = x - x_ref
       │   ├─> u = u_hover - K*e
       │   └─> Saturate u
       │
       └─> quadrotor_dynamics_pure(x, u, params)
           ├─> Parse: position, attitude, velocities
           ├─> Compute: rotation matrices, forces, torques
           ├─> Calculate: ẋ = f(x,u) [nonlinear]
           └─> Return: state derivative

4. POST-PROCESSING
   └─> Compute metrics, generate plots, save results
```

---

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

**NED (North-East-Down):**
- **+X**: North (forward)
- **+Y**: East (right)
- **+Z**: Down (toward ground)
- **Gravity**: Acts in +Z direction (downward)
- **Thrust**: Acts in -Z direction (upward, opposes +Z)

This is the standard aerospace convention used throughout the simulation.

---

## 🚀 Quick Start

### 1. Setup Directories
```matlab
mkdir('./trajectories');
mkdir('./control');
mkdir('./dynamics');
mkdir('./vehicle');
mkdir('./test');
mkdir('./results');
```

### 2. Place All Files
- Copy each `.m` file to its designated folder (see structure above)
- Ensure `quadrotor_linear_6dof.m` is in `./vehicle/`
- Place waypoint `.wpt` files in `./trajectories/`
- Place test files in `./test/`

### 3. Run Quick Test
```matlab
quick_test_simulation
```

### 4. Run Full Simulation
```matlab
simulate_quadrotor_pure
```

### Expected Output
```
===================================
Quadrotor 6DOF LQR Control System
Pure MATLAB Implementation
===================================

Step 1/5: Loading quadrotor model...
Step 2/5: Generating optimal trajectory...
Step 3/5: Setting up simulation...
Step 4/5: Running ODE simulation...
Step 5/5: Analyzing results...

=== PERFORMANCE METRICS ===
Position RMSE:     X=0.0234m  Y=0.0198m  Z=0.0156m
Attitude RMSE:     φ=1.23°  θ=1.45°  ψ=0.87°
Control Effort:    145.67

=== SIMULATION COMPLETE ===
```

---

## 🧪 Testing

### Run All Tests
```matlab
% Run individual test suites
test_linear_6dof        % Vehicle model (7 tests)
test_dynamics_pure      % Nonlinear dynamics (7 tests)
test_control_loop       % Control loop (8 tests)
test_waypoints          # Waypoint loader (7 tests)
test_trajectory         # Trajectory generation (8 tests)
```

### Test Coverage
- **37 unit tests** covering all core functionality
- Tests validate:
  - Physical parameter correctness
  - LQR stability and controllability
  - Nonlinear dynamics accuracy
  - Control law implementation
  - Trajectory generation smoothness
  - Waypoint file parsing

---

## 🔧 Customization

### Define Trajectories

**Method 1: Waypoint Files (Recommended)**

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
    {"label": "turn_right", "time": 8, "x": 2, "y": 2, "z": 1, "yaw": 1.57},
    {"label": "return", "time": 11, "x": 0, "y": 2, "z": 1.5, "yaw": 3.14},
    {"label": "hover", "time": 14, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 17, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

Load in simulation:
```matlab
% In simulate_quadrotor_pure.m
wpt = load_waypoints('./trajectories/my_trajectory.wpt');
trajectory = generate_trajectory(wpt, params);
```

**Waypoint File Format (.wpt using JSON):**
- **metadata** (optional): Trajectory information for documentation
  - name: Descriptive trajectory name
  - description: Purpose or details
  - created: Date created
  - vehicle: Target vehicle identifier
- **waypoints** (required): Array of waypoint objects
  - **label**: Waypoint identifier used in outputs and plot annotations
  - **time**: Time to reach waypoint in seconds
  - **x, y, z**: Position in meters (NED coordinates)
  - **yaw**: Heading angle in radians, or `null` for auto-calculation

**Yaw Behavior:**
- **Explicit value** (e.g., `1.57`): UAV will orient to specified heading
- **null** (in JSON): UAV will automatically face direction of travel (computed from velocity vector)
- Useful for camera pointing, inspection tasks, or wind compensation

**Method 2: Direct Definition (Quick Testing)**

For rapid prototyping, define inline as a matrix:
```matlab
% In simulate_quadrotor_pure.m
% Format: [time, x, y, z, yaw]
waypoints = [
    0,   0,   0,   0,   0;
    3,   2,   0,   1,   NaN;    % NaN for auto yaw in MATLAB arrays
    10,  0,   0,   0,   0;
];
trajectory = generate_trajectory(waypoints, params);
```

**Note:** Use `NaN` for auto-yaw in MATLAB arrays, `null` in JSON files.

**Benefits of .wpt JSON Files:**
- ✅ Reuse trajectories across experiments
- ✅ Version control trajectory designs separately
- ✅ Easy batch processing for Monte Carlo
- ✅ Self-documenting with metadata
- ✅ Native MATLAB support (jsondecode)
- ✅ Comments and optional fields
- ✅ Non-programmers can define flight paths
- ✅ Clear separation of data and code

### Tune LQR Controller
```matlab
% Before calling quadrotor_linear_6dof()
Q_custom = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);
R_custom = diag([0.5 2 2 2]);
params = quadrotor_linear_6dof(Q_custom, R_custom);

% Load trajectory
wpt = load_waypoints('./trajectories/my_trajectory.wpt');
trajectory = generate_trajectory(wpt, params);
```

#### Understanding Q and R Matrices

The Q and R matrices fundamentally shape controller behavior by defining the trade-off between tracking performance and control effort:

**Q Matrix (State Penalty) - 12×12 diagonal**
```
Q = diag([qx qy qz qφ qθ qψ qvx qvy qvz qp qq qr])
```
- **Higher Q values** → Tighter tracking, more aggressive corrections
- **Lower Q values** → Looser tracking, gentler response
- Position weights (qx, qy, qz): Affect spatial accuracy
- Attitude weights (qφ, qθ, qψ): Affect orientation precision
- Velocity weights: Damping and overshoot characteristics

**R Matrix (Control Penalty) - 4×4 diagonal**
```
R = diag([rF rτφ rτθ rτψ])
```
- **Higher R values** → Smaller control inputs, smoother but slower
- **Lower R values** → Larger control inputs, faster but more aggressive
- Thrust penalty (rF): Affects altitude response and energy usage
- Torque penalties: Affect rotational maneuvers

**Practical Tuning Guidelines:**

| Desired Behavior | Modification | Trade-off |
|------------------|--------------|-----------|
| Tighter position tracking | Increase position Q | More control effort |
| Smoother flight | Increase R | Slower tracking |
| Faster response | Decrease R | Risk of oscillation |
| Reduce oscillations | Increase velocity Q | Less aggressive |
| Prioritize yaw accuracy | Increase qψ | More yaw control usage |

**Example Configurations:**
```matlab
% Aggressive tracking (racing drone)
Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R_aggressive = diag([0.01 0.5 0.5 0.5]);
params = quadrotor_linear_6dof(Q_aggressive, R_aggressive);
wpt = load_waypoints('./trajectories/racing_track.wpt');
trajectory = generate_trajectory(wpt, params);

% Smooth operation (aerial photography)
Q_smooth = diag([50 50 100 5 5 2 5 5 10 0.5 0.5 0.1]);
R_smooth = diag([1 5 5 5]);
params = quadrotor_linear_6dof(Q_smooth, R_smooth);
wpt = load_waypoints('./trajectories/photo_survey.wpt');
trajectory = generate_trajectory(wpt, params);

% Energy efficient (long flight time)
Q_efficient = diag([80 80 100 8 8 1 8 8 10 1 1 0.2]);
R_efficient = diag([5 2 2 2]);
params = quadrotor_linear_6dof(Q_efficient, R_efficient);
wpt = load_waypoints('./trajectories/patrol_route.wpt');
trajectory = generate_trajectory(wpt, params);
```

The optimal Q and R depend on your application requirements, vehicle constraints, and trajectory characteristics. Experimentation and Monte Carlo analysis help identify robust tuning.

### Modify Vehicle Parameters
Edit `./vehicle/quadrotor_linear_6dof.m`:
```matlab
params.m = 1.0;      % Mass (kg)
params.L = 0.30;     % Arm length (m)
params.Ixx = 0.01;   % Inertia (kg·m²)
```

---

## 📊 Output

### Generated Plots
- 3D trajectory (reference vs actual)
- Position tracking (X, Y, Z)
- Attitude tracking (roll, pitch, yaw)
- Control inputs (thrust and torques)
- Velocity profiles

### Saved Data
- `./results/simulation_results.mat` contains:
  - Time history
  - State history
  - Control history
  - Performance metrics
  - Parameters used

---

## 🎓 Theory

### LQR Controller

**Linear Quadratic Regulator (LQR)** is an optimal control technique that finds the feedback gain matrix K by minimizing a quadratic cost function:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

**Cost Function**

The cost function balances two competing objectives:
- **xᵀQx**: Penalizes deviations from the reference state (tracking error)
  - Larger Q weights → tighter tracking, more aggressive response
  - The Q matrix lets us prioritize certain states (e.g., position over velocity)
- **uᵀRu**: Penalizes control effort (energy/actuator usage)
  - Larger R weights → smoother control, less aggressive
  - Prevents excessive actuator commands and conserves energy

**How LQR Derives K:**

1. **Algebraic Riccati Equation (ARE)**: MATLAB's `lqr()` solves the continuous-time ARE:
   ```
   AᵀS + SA - SBR⁻¹BᵀS + Q = 0
   ```
   for the matrix S (solution to the Riccati equation).

2. **Optimal Gain**: Once S is found, the optimal feedback gain is:
   ```
   K = R⁻¹BᵀS
   ```

3. **Control Law**: The resulting control minimizes the cost function:
   ```
   u = u_hover - K(x - x_ref)
   ```
   - **u_hover**: Nominal control to maintain equilibrium (hover thrust)
   - **-K(x - x_ref)**: Feedback correction proportional to tracking error
   - The negative sign ensures feedback **opposes** the error

**Key Properties:**
- **Guaranteed stability**: Closed-loop system (A - BK) has all poles in left half-plane
- **Optimal for linear systems**: Provably minimizes the cost function
- **Trade-off tuning**: Q and R matrices provide intuitive tuning knobs
- **Robust margins**: Provides guaranteed gain/phase margins (60°/6dB)

**Why It Works for Quadrotors:**

Even though the quadrotor has **nonlinear dynamics**, LQR works well because:
1. The system is linearized around hover (where we spend most time)
2. For small deviations, the linear approximation is accurate
3. The LQR gain provides inherent robustness to modeling errors
4. Quadrotor dynamics are "mildly nonlinear" near hover

**Limitations:**
- Performance degrades far from the linearization point
- No guarantees for large angle maneuvers or aggressive flight
- Extensions like gain scheduling or nonlinear control improve performance

### Trajectory Generation
Shape-preserving cubic interpolation (pchip) ensures:
- C¹ continuity (smooth position and velocity)
- Shape preservation (no overshoots between waypoints)
- Local control (changes affect only nearby segments)
- Numerically stable differentiation for acceleration

### Coordinate Frames
**Body Frame to Inertial (NED):**
```
R_b2i = Rz(ψ) * Ry(θ) * Rx(φ)
```
Standard ZYX Euler angle sequence (yaw-pitch-roll).

---

## 🔬 Future Work

### Monte Carlo Analysis (Planned)
- Parameter uncertainty analysis
- Disturbance rejection testing
- Robustness quantification
- Statistical performance metrics

### Extensions
- Multiple controller comparison (PID, MPC, etc.)
- True minimum-snap trajectory optimization
- Observer/estimator design
- Hardware deployment to embedded system

---

## 📝 File Descriptions

| File | Purpose |
|------|---------|
| `simulate_quadrotor_pure.m` | Main orchestrator, runs simulation |
| `quick_test_simulation.m` | Quick test script with visualization |
| `quadrotor_linear_6dof.m` | Vehicle model and LQR design |
| `load_waypoints.m` | Waypoint file loader (JSON .wpt format) |
| `generate_trajectory.m` | Smooth trajectory generation (pchip interpolation) |
| `compute_lqr_control.m` | LQR control law with saturation |
| `get_reference_state.m` | Reference state lookup with interpolation |
| `quadrotor_closed_loop_dynamics.m` | ODE wrapper function |
| `quadrotor_dynamics_pure.m` | Nonlinear 6DOF dynamics (NED coordinates) |
| `Constants.m` | Shared constants classdef |
| `setup_test_environment.m` | Test path configuration |
| `test_*.m` | Unit test suites. These can aid in understanding how various functions work and facilitate identifying breaking changes |
| `*.wpt` (waypoints) | Waypoint definitions (JSON format) |

---

## 🛠️ Requirements

- MATLAB R2019b or later
- Control System Toolbox (for `lqr()` function)
- No Simulink required
- No additional toolboxes needed

---

## 📚 References

- **LQR Theory**: Anderson, B. D., & Moore, J. B. (1990). *Optimal Control: Linear Quadratic Methods*
- **Quadrotor Dynamics**: Bouabdallah, S. (2007). *Design and Control of Quadrotors with Application to Autonomous Flying*
- **Trajectory Optimization**: Mellinger, D., & Kumar, V. (2011). *Minimum Snap Trajectory Generation and Control for Quadrotors*
- **NED Coordinates**: Standard aerospace convention (ISO 8855, SAE J670)

---