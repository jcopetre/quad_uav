# Quadrotor 6DOF LQR Control - Pure MATLAB Implementation

A complete simulation framework for trajectory tracking control of a 6 degree-of-freedom quadrotor UAV using Linear Quadratic Regulator (LQR) control.

**Master's Project - Research Lab**

---

## 📋 Project Objectives

1. ✅ **Basic Framework**: Reference trajectory tracking with LQR control
2. ✅ **Simulation**: 6DOF linearized quadrotor model with nonlinear dynamics
3. ✅ **Trajectory Generation**: Control-optimal trajectories (minimum-snap)
4. ✅ **Performance Evaluation**: Quantitative metrics and visualization
5. 🔄 **Monte Carlo Analysis**: Robustness and sensitivity analysis (future)
6. 🔄 **Hardware Deployment**: Embedded system implementation (optional)

---

## 📁 Project Structure

```
<root>/
│
├── README.md                            [This file]
├── simulate_quadrotor_pure.m            [Main simulation script - RUN THIS]
│
├── vehicle/
│   └── quadrotor_linear_6dof.m          [Vehicle model, parameters, LQR design]
│
├── trajectories/
│   ├── load_waypoints.m                 [Waypoint file loader]
│   ├── generate_trajectory.m            [Minimum-snap trajectory generation]
│   └── *.csv                            [Waypoint definition files]
│
├── control/
│   ├── compute_lqr_control.m            [LQR control law implementation]
│   ├── get_reference_state.m            [Reference state lookup]
│   └── quadrotor_closed_loop_dynamics.m [Closed-loop system for ODE solver]
│
├── dynamics/
│   └── quadrotor_dynamics_pure.m        [Nonlinear 6DOF dynamics]
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
│                     [Main Orchestrator]                      │
└────────┬──────────────────────────────────────────┬─────────┘
         │                                          │
         ↓                                          ↓
┌─────────────────────┐                   ┌──────────────────┐
│ quadrotor_linear_   │                   │ load_waypoints   │
│ 6dof.m              │                   │ generate_        │
│                     │                   │ trajectory       │
│ • Physical params   │                   │                  │
│ • Linearized model  │                   │ • CSV in         │
│ • LQR gains (K)     │                   │ • Smooth path    │
└──────────┬──────────┘                   │   out            │
           │                               └────────┬─────────┘
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
              │                              │
              └──────────────┬───────────────┘
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
       ├─> Build: A, B matrices (linearized)
       ├─> Design: LQR gains K from (A,B,Q,R)
       └─> Return: params struct

2. TRAJECTORY GENERATION
   └─> load_waypoints(wpt_file)
       ├─> Parse .wpt file (JSON format)
       ├─> Load metadata and waypoints
       ├─> Handle null for auto yaw calculation
       ├─> Validate waypoint data
       └─> Return structure with labels and waypoint data
   
   └─> generate_trajectory(wpt, params)
       ├─> Process yaw: explicit values or auto from velocity
       ├─> 5th-order polynomial interpolation
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
  Attitude:        φ, θ, ψ            [rad] (roll, pitch, yaw)
  Linear Velocity: ẋ, ẏ, ż           [m/s]
  Angular Rate:    p, q, r            [rad/s]
```

## 🎮 Control Input Definition

```
u = [F, τ_φ, τ_θ, τ_ψ]ᵀ  (4 inputs)

Where:
  Thrust:  F                [N]
  Torques: τ_φ, τ_θ, τ_ψ    [N·m]
```

---

## 🚀 Quick Start

### 1. Setup Directories
```matlab
mkdir('./trajectories');
mkdir('./control');
mkdir('./dynamics');
mkdir('./results');
```

### 2. Place All Files
- Copy each `.m` file to its designated folder (see structure above)
- Ensure `quadrotor_linear_6dof.m` is in `./vehicle/`
- Place waypoint `.wpt` files in `./trajectories/`

### 3. Run Simulation
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

## 🔧 Customization

### Define Trajectories

**Method 1: Waypoint Files (Recommended)**

Create a `.wpt` file in `./trajectories/`:

```json
{
  "metadata": {
    "name": "Basic Test Flight",
    "description": "Takeoff, maneuver, and land",
    "created": "2025-01-08",
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
waypoints = load_waypoints('./trajectories/my_trajectory.wpt');
trajectory = generate_trajectory(waypoints, params);
```

**Waypoint File Format:**
- **label**: Descriptive name for waypoint (for documentation)
- **x, y, z**: Position in meters
- **yaw**: Heading angle in radians
- **time**: Time to reach waypoint in seconds

**Method 2: Direct Definition (Quick Testing)**

For rapid prototyping, define inline:
```matlab
% In simulate_quadrotor_pure.m
waypoints = [
    0,   0,   0,   0,   0;      % [time, x, y, z, yaw]
    3,   2,   0,   1,   NaN;
    10,  0,   0,   0,   0;
];
trajectory = generate_trajectory(waypoints, params);
```

**Benefits of CSV Files:**
- ✅ Reuse trajectories across experiments
- ✅ Version control trajectory designs separately
- ✅ Easy batch processing for Monte Carlo
- ✅ Documentation: trajectory becomes data artifact
- ✅ Non-programmers can define flight paths

### Tune LQR Controller
```matlab
% Before calling quadrotor_linear_6dof()
Q_custom = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);
R_custom = diag([0.5 2 2 2]);
params = quadrotor_linear_6dof(Q_custom, R_custom);
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

% Smooth operation (aerial photography)
Q_smooth = diag([50 50 100 5 5 2 5 5 10 0.5 0.5 0.1]);
R_smooth = diag([1 5 5 5]);

% Energy efficient (long flight time)
Q_efficient = diag([80 80 100 8 8 1 8 8 10 1 1 0.2]);
R_efficient = diag([5 2 2 2]);
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
Minimizes cost function:
```
J = ∫(xᵀQx + uᵀRu)dt
```

Optimal control law:
```
u = u_hover - K(x - x_ref)
```

Where K is computed from the linearized system using MATLAB's `lqr()` function.

### Trajectory Generation
5th-order polynomial ensures:
- C² continuity (smooth position, velocity, acceleration)
- Zero velocity/acceleration at waypoints
- Minimum snap for quadrotor dynamics

---

## 🔬 Future Work

### Monte Carlo Analysis (Planned)
- Parameter uncertainty analysis
- Disturbance rejection testing
- Robustness quantification
- Statistical performance metrics

### Extensions
- Multiple controller comparison (PID, MPC, etc.)
- Optimal trajectory generation (min control effort)
- Observer/estimator design
- Hardware deployment to embedded system

---

## 📝 File Descriptions

| File | Purpose |
|------|---------|
| `simulate_quadrotor_pure.m` | Main orchestrator, runs simulation |
| `quadrotor_linear_6dof.m` | Vehicle model and LQR design |
| `load_waypoints.m` | Waypoint file loader (JSON .wpt format) |
| `generate_trajectory.m` | Minimum-snap trajectory generation |
| `compute_lqr_control.m` | LQR control law |
| `get_reference_state.m` | Reference state lookup |
| `quadrotor_closed_loop_dynamics.m` | ODE wrapper function |
| `quadrotor_dynamics_pure.m` | Nonlinear 6DOF dynamics |
| `*.wpt` (trajectories) | Waypoint definitions |

**Total: ~535 lines of clean, documented code**

---

## ✅ Design Principles

1. **Modularity**: Each component in separate file
2. **Single Responsibility**: Each function does one thing well
3. **Readability**: Clear variable names, comprehensive comments
4. **Testability**: Easy to test components independently
5. **Extensibility**: Simple to add new features
6. **MATLAB Best Practices**: Vectorized operations, efficient ODE integration

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

---

## 📧 Project Info

**Type**: Master's Thesis Project  
**Goal**: Demonstrate competency in control systems and provide discussion-worthy results  
**Status**: Active Development  
**Architecture**: Pure MATLAB (Simulink port optional for hardware deployment)

---

## 🎉 Advantages of This Implementation

✅ **Fast iteration** - change and run in seconds  
✅ **Easy debugging** - full MATLAB tooling  
✅ **Clean code** - publication quality  
✅ **Flexible** - easy to extend and modify  
✅ **Reproducible** - no compilation, consistent results  
✅ **Thesis-ready** - generates all necessary figures and metrics  

---

*Last Updated: [Current Date]*